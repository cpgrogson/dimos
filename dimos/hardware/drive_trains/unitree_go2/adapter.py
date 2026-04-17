# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unitree Go2 TwistBase adapter — merged SDK2 control plane for the quadruped.

Implements the TwistBaseAdapter protocol (3 DOF: vx, vy, wz) on top of
unitree_sdk2py. Covers the full high-level control plane in a single class:

  - TwistBase surface (connect/read/write/enable) for the tick loop
  - MotionSwitcher inspection/control (check_mode, switch_mode, safe release)
  - Raw SportModeState_ readout for diagnostics
  - Defined-but-unwired low-level hooks (subscribe_low_state / publish_low_cmd)
    — real implementation lives in adapter_lowlevel.py

Required init sequence on current Go2 firmware:

  ChannelFactoryInitialize → MotionSwitcher.CheckMode/SelectMode("normal")
  → SportClient.Init → StandUp → FreeWalk → Move(vx, vy, wz)

The sport service boots dormant on recent firmware — every SportClient RPC
returns 3102 (RPC_ERR_CLIENT_SEND) until MotionSwitcher has selected a mode.

MotionSwitcher modes recognized by the SDK:
  - "normal"   — factory sport controller; SportClient high-level API
  - "ai"       — AI sport controller with built-in autonomous behaviors
  - "advanced" — locomotion stopped; required for low-level LowCmd_ publishing
  - "mcf"      — Motion Control Framework variant; SportClient races with
                 the onboard planner on some firmwares
  - ""         — no controller active (robot falls if standing)

Mode switching while a controller is running drops all 12 joint servos —
ReleaseMode() / SelectMode() while upright will cause the robot to fall
(incident 2026-04-11). Use stand_down_and_release() as the canonical safe
path, or pass i_have_sat_the_robot=True to switch_mode() to bypass the gate.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import threading
import time
from typing import TYPE_CHECKING

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
        MotionSwitcherClient,
    )
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import (
        LowCmd_,
        LowState_,
        SportModeState_,
    )

    from dimos.hardware.drive_trains.registry import TwistBaseAdapterRegistry

logger = setup_logger()


@dataclass
class _Session:
    """Active connection state for a Go2.

    The session object is created by connect() and set on the adapter under
    _session_lock. All mutable state that can be touched by both the DDS
    callback thread and the control thread lives here, guarded by `lock`.
    """

    client: SportClient
    motion_switcher: MotionSwitcherClient
    lock: threading.Lock
    state_sub: ChannelSubscriber | None = None
    latest_state: SportModeState_ | None = None
    enabled: bool = False
    locomotion_ready: bool = False


class UnitreeGo2TwistAdapter:
    """TwistBaseAdapter for the Unitree Go2 quadruped via unitree_sdk2py (DDS).

    3 DOF velocity: [vx, vy, wz].
      - vx: forward/backward linear velocity (m/s)
      - vy: lateral (left positive) linear velocity (m/s)
      - wz: yaw rate (rad/s)

    Thread model:
      - _session_lock guards the self._session reference across threads.
      - session.lock guards latest_state and SportClient RPC serialization.
      Never take _session_lock while holding session.lock — the DDS callback
      already holds session.lock briefly during state updates.

    Args:
        dof: Must be 3 for Go2. ValueError otherwise.
        speed_level: SportClient speed envelope. -1 = slow, 0 = normal,
            1 = fast (default, max). Applied once after FreeWalk succeeds.
            May be ignored in non-'normal' modes (mcf runs its own planner).
            Change at runtime via set_speed_level().
        preferred_mode: If set (default 'normal'), connect() will
            automatically switch into this mode during startup — sits the
            robot down, releases the running controller, selects the
            preferred one, then stands back up. Pass None to disable
            (accept whatever mode the robot booted in).

    TODO(network_interface): multi-NIC hosts may need an explicit DDS
    interface name passed through to ChannelFactoryInitialize(0, iface).
    Dropped for now — current setup assumes exactly one Go2-reachable
    NIC is up. Re-add as an adapter_kwarg (plumbed via
    coordinator._create_twist_base_adapter) if/when DDS discovery
    picks the wrong card.
    """

    # Firmware-variant fallbacks. "normal" is the canonical Go2 sport mode;
    # the others are accepted as valid if the robot is already in them so we
    # don't force a release (which would drop servo) just to match a name.
    _SPORT_MODE_CANDIDATES: tuple[str, ...] = ("normal", "ai", "advanced", "mcf")

    def __init__(
        self,
        dof: int = 3,
        speed_level: int = 1,
        preferred_mode: str | None = "normal",
        **_: object,
    ) -> None:
        if dof != 3:
            raise ValueError(f"Go2 only supports 3 DOF (vx, vy, wz), got {dof}")

        self._session: _Session | None = None
        self._session_lock = threading.Lock()
        self._speed_level = speed_level
        self._preferred_mode = preferred_mode

    # =========================================================================
    # TwistBaseAdapter protocol
    # =========================================================================

    def connect(self) -> bool:
        """Connect to Go2, select a sport mode, stand up, enter FreeWalk.

        Sequence:
          1. ChannelFactoryInitialize(0) — default domain, default NIC.
          2. Subscribe rt/sportmodestate for telemetry.
          3. MotionSwitcher.Init + _activate_sport_mode() — accepts
             whatever controller is currently running.
          4. SportClient.Init.
          5. If preferred_mode is set and current != preferred:
             stand_down_and_release() + switch_mode(preferred).
          6. _initialize_locomotion(): StandUp + FreeWalk + SpeedLevel.

        Fails fast on persistent 3102 (RPC_ERR_CLIENT_SEND) — logs
        actionable guidance ("exit AI mode with L2+B, close Unitree app,
        verify the host can see the Go2 on the network") rather than
        silently proceeding.

        Returns:
            True on success. False on connect/init/locomotion failure.
            Does not raise on missing SDK — logs and returns False.
        """
        with self._session_lock:
            if self._session is not None:
                logger.warning("[Go2] Already connected — disconnect first")
                return False

        try:
            from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
                MotionSwitcherClient,
            )
            from unitree_sdk2py.core.channel import (
                ChannelFactoryInitialize,
                ChannelSubscriber,
            )
            from unitree_sdk2py.go2.sport.sport_client import SportClient
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

            logger.info("[Go2] Initializing DDS (domain=0, default NIC)...")
            ChannelFactoryInitialize(0)

            logger.info("[Go2] Connecting MotionSwitcherClient...")
            motion_switcher = MotionSwitcherClient()
            motion_switcher.SetTimeout(5.0)
            motion_switcher.Init()
            time.sleep(1.5)  # DDS discovery settle

            # SportClient.Init runs AFTER sport mode is activated so that
            # RPCs issued during Init land on a live peer.
            client = SportClient()
            client.SetTimeout(10.0)

            session = _Session(
                client=client,
                motion_switcher=motion_switcher,
                lock=threading.Lock(),
            )

            def state_callback(msg: SportModeState_) -> None:
                with session.lock:
                    session.latest_state = msg

            state_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
            state_sub.Init(state_callback, 10)
            session.state_sub = state_sub

            with self._session_lock:
                self._session = session

            if not self._activate_sport_mode():
                logger.error("[Go2] Failed to activate sport mode")
                self.disconnect()
                return False

            logger.info("[Go2] Initializing SportClient...")
            client.Init()
            time.sleep(2.0)

            logger.info("[Go2] Connected ✓")

            # If the robot came up in a mode other than our preferred one,
            # transition safely: stand down, release, select preferred.
            # This is the full safe path — switch_mode's standing-robot
            # guard is bypassed because stand_down_and_release() has just
            # put the joints into a damped / idle state.
            if self._preferred_mode is not None:
                current = self.check_mode()
                if current and current != self._preferred_mode:
                    logger.info(
                        f"[Go2] Booted in '{current}' but preferred_mode="
                        f"'{self._preferred_mode}' — switching safely..."
                    )
                    if not self.stand_down_and_release():
                        logger.error(
                            "[Go2] Could not safely stand down; aborting preferred-mode switch"
                        )
                        self.disconnect()
                        return False
                    if not self.switch_mode(self._preferred_mode, i_have_sat_the_robot=True):
                        logger.error(f"[Go2] switch_mode('{self._preferred_mode}') failed")
                        self.disconnect()
                        return False

            if not self._initialize_locomotion():
                logger.error("[Go2] Failed to initialize locomotion mode")
                self.disconnect()
                return False

            self.print_status()
            return True

        except Exception as e:
            logger.error(f"[Go2] Failed to connect: {e}")
            with self._session_lock:
                self._session = None
            return False

    def disconnect(self) -> None:
        """Stop motion, stand the robot down, and tear down DDS resources.

        Safe to call multiple times. Explicitly Close()s the state
        subscriber to prevent DDS reader leaks across reconnects.
        """
        with self._session_lock:
            session = self._session
            self._session = None

        if session is None:
            return

        try:
            session.client.StopMove()
            time.sleep(0.2)
            session.client.StandDown()
            time.sleep(0.3)
        except Exception as e:
            logger.error(f"[Go2] Error during disconnect: {e}")

        if session.state_sub is not None:
            try:
                session.state_sub.Close()
            except Exception as e:
                logger.error(f"[Go2] Error closing state subscriber: {e}")

    def is_connected(self) -> bool:
        """True iff a session exists. Read under _session_lock."""
        with self._session_lock:
            return self._session is not None

    def get_dof(self) -> int:
        """Always 3 for Go2 (vx, vy, wz)."""
        return 3

    def read_velocities(self) -> list[float]:
        """Measured velocities [vx, vy, wz] from SportModeState_.

        Sources:
          vx, vy: state.velocity[0], state.velocity[1]
          wz:     state.imu_state.gyroscope[2]

        Returns [0.0, 0.0, 0.0] during the startup gap before the first
        DDS callback has populated latest_state.
        """
        session = self._get_session()
        with session.lock:
            if session.latest_state is None:
                return [0.0, 0.0, 0.0]
            try:
                state = session.latest_state
                return [
                    float(state.velocity[0]),
                    float(state.velocity[1]),
                    float(state.imu_state.gyroscope[2]),
                ]
            except Exception as e:
                logger.warning(f"[Go2] Error reading velocities: {e}")
                return [0.0, 0.0, 0.0]

    def read_odometry(self) -> list[float] | None:
        """Measured pose [x, y, theta] from SportModeState_.

        Sources:
          x, y:  state.position[0], state.position[1]
          theta: state.imu_state.rpy[2]  (yaw)

        Returns None if no state message has arrived yet.
        """
        session = self._get_session()
        with session.lock:
            if session.latest_state is None:
                return None
            try:
                state = session.latest_state
                return [
                    float(state.position[0]),
                    float(state.position[1]),
                    float(state.imu_state.rpy[2]),
                ]
            except Exception as e:
                logger.error(f"[Go2] Error reading odometry: {e}")
                return None

    def write_velocities(self, velocities: list[float]) -> bool:
        """Send Twist command [vx, vy, wz] via SportClient.Move().

        Refuses (returns False) if:
          - len(velocities) != 3
          - session not enabled (write_enable(True) not called)
          - locomotion not ready (StandUp/FreeWalk incomplete)
        """
        if len(velocities) != 3:
            return False

        session = self._get_session()

        if not session.enabled:
            logger.warning("[Go2] Not enabled, ignoring velocity command")
            return False

        if not session.locomotion_ready:
            logger.warning("[Go2] Locomotion not ready, ignoring velocity command")
            return False

        vx, vy, wz = velocities
        return self._send_velocity(vx, vy, wz)

    def write_stop(self) -> bool:
        """Stop motion via SportClient.StopMove(). Leaves robot standing."""
        session = self._get_session()
        try:
            with session.lock:
                session.client.StopMove()
            return True
        except Exception as e:
            logger.error(f"[Go2] Error stopping: {e}")
            return False

    def write_enable(self, enable: bool) -> bool:
        """Enable/disable velocity command path.

        enable=True: ensures locomotion is ready (re-initializes if needed),
                     then flips session.enabled.
        enable=False: calls write_stop() and clears session.enabled. Does
                     NOT stand the robot down — use disconnect() for that.
        """
        session = self._get_session()

        if enable:
            if not session.locomotion_ready:
                logger.info("[Go2] Locomotion not ready, initializing...")
                if not self._initialize_locomotion():
                    logger.error("[Go2] Failed to initialize locomotion")
                    return False
            session.enabled = True
            logger.info("[Go2] Enabled")
            return True

        self.write_stop()
        session.enabled = False
        logger.info("[Go2] Disabled")
        return True

    def read_enabled(self) -> bool:
        """True iff session exists AND session.enabled. Reads under
        _session_lock so it never returns stale True after disconnect()."""
        with self._session_lock:
            return self._session is not None and self._session.enabled

    # =========================================================================
    # Mode inspection / control (beyond TwistBaseAdapter)
    # =========================================================================

    def check_mode(self) -> str | None:
        """Return the current MotionSwitcher mode name, or None on RPC fail.

        Wraps MotionSwitcher.CheckMode(). Empty string means no controller
        active; None means the RPC itself failed (usually 3102).
        """
        session = self._get_session()
        try:
            code, data = session.motion_switcher.CheckMode()
        except Exception as e:
            logger.warning(f"[Go2] CheckMode failed: {e}")
            return None

        if code == 0 and isinstance(data, dict):
            return (data.get("name") or "").strip()
        return None

    def switch_mode(
        self,
        name: str,
        *,
        i_have_sat_the_robot: bool = False,
    ) -> bool:
        """Switch MotionSwitcher to `name` (normal/ai/advanced/mcf).

        Refuses by default if a locomotion controller is running —
        SelectMode/ReleaseMode drops all 12 joint servos, and the robot
        falls if upright (incident 2026-04-11). The caller must either:

          - Call stand_down_and_release() first, THEN switch_mode(name), or
          - Pass i_have_sat_the_robot=True after verifying physically.

        Does not rely on heuristic standing detection (position[2] or
        foot_force are not reliable gates).

        Returns True if the requested mode is active after the switch
        (verified via CheckMode polling up to 5s).
        """
        session = self._get_session()
        current = self.check_mode()

        if current and not i_have_sat_the_robot:
            logger.error(
                f"[Go2] Refusing switch_mode('{name}'): controller '{current}' "
                "is running. Releasing it would drop all 12 joint servos and "
                "the robot would fall. Call stand_down_and_release() first, "
                "or pass i_have_sat_the_robot=True after verifying physically."
            )
            return False

        try:
            # Release current controller (if any) before selecting a new one.
            if current:
                try:
                    rel_code, _ = session.motion_switcher.ReleaseMode()
                    logger.info(f"[Go2] ReleaseMode() -> {rel_code}")
                except Exception as e:
                    logger.warning(f"[Go2] ReleaseMode raised: {e}")
                time.sleep(1.0)

            if not name:
                # Empty name = release only.
                active = self._poll_mode(want_nonempty=False, timeout=5.0)
                if active in ("", None):
                    session.locomotion_ready = False
                    session.enabled = False
                    logger.info("[Go2] ✓ Mode released (no controller active)")
                    return True
                logger.error(f"[Go2] ReleaseMode did not take effect (still '{active}')")
                return False

            sel_code, _ = session.motion_switcher.SelectMode(name)
            logger.info(f"[Go2] SelectMode('{name}') -> code={sel_code}")

            if sel_code == 3102:
                logger.error(f"[Go2] SelectMode('{name}') failed with RPC 3102")
                return False

            active = self._poll_mode(want_nonempty=True, timeout=5.0)
            if active == name:
                logger.info(f"[Go2] ✓ Switched to mode '{name}'")
                session.locomotion_ready = False
                session.enabled = False
                time.sleep(2.0)
                return True

            logger.error(f"[Go2] SelectMode('{name}') did not take effect (active='{active}')")
            return False

        except Exception as e:
            logger.error(f"[Go2] switch_mode error: {e}")
            return False

    def stand_down_and_release(self) -> bool:
        """Canonical safe path from running locomotion to servo-free state.

        Sequence:
          1. StopMove() — zero velocity command.
          2. StandDown() — robot sits.
          3. Poll SportModeState.mode until damped/idle for >= 1s.
          4. ReleaseMode() — controller stops, joints fully passive.

        After this returns True, switch_mode('advanced') or low-level
        publishing is safe. Returns False if any step fails; the robot
        state on failure depends on which step failed — check the logs.
        """
        session = self._get_session()

        try:
            # Step 1: stop motion (best-effort; StopMove may fail if already
            # stopped — not fatal).
            try:
                with session.lock:
                    session.client.StopMove()
            except Exception as e:
                logger.warning(f"[Go2] StopMove raised during stand-down: {e}")
            time.sleep(0.3)

            # Step 2: stand down.
            logger.info("[Go2] Standing down...")
            try:
                with session.lock:
                    ret = session.client.StandDown()
            except Exception as e:
                logger.error(f"[Go2] StandDown raised: {e}")
                return False
            if ret != 0:
                logger.error(f"[Go2] StandDown() failed with code {ret}")
                return False

            # Step 3: poll state.mode until it holds stable for >=1s. The
            # specific `mode` integer is firmware-dependent; we only need
            # to detect "no longer changing," which implies the controller
            # has settled in the damped/idle posture.
            deadline = time.monotonic() + 6.0
            stable_since: float | None = None
            last_mode: int | None = None
            while time.monotonic() < deadline:
                with session.lock:
                    state = session.latest_state
                if state is not None:
                    try:
                        m = int(state.mode)
                    except Exception:
                        m = None
                    if m is not None and m == last_mode:
                        if stable_since is None:
                            stable_since = time.monotonic()
                        elif time.monotonic() - stable_since >= 1.0:
                            break
                    else:
                        stable_since = time.monotonic() if m is not None else None
                        last_mode = m
                time.sleep(0.1)
            else:
                logger.warning(
                    "[Go2] state.mode did not stabilize within 6s; proceeding "
                    "with ReleaseMode anyway"
                )

            # Step 4: release mode.
            logger.info("[Go2] Releasing mode...")
            try:
                rel_code, _ = session.motion_switcher.ReleaseMode()
                logger.info(f"[Go2] ReleaseMode() -> {rel_code}")
            except Exception as e:
                logger.error(f"[Go2] ReleaseMode raised: {e}")
                return False

            active = self._poll_mode(want_nonempty=False, timeout=5.0)
            if active is None:
                logger.warning("[Go2] CheckMode failed during release poll; assuming released")
            elif active != "":
                logger.error(f"[Go2] Release did not take effect (still '{active}')")
                return False

            session.locomotion_ready = False
            session.enabled = False
            logger.info("[Go2] ✓ Stood down and released — safe for mode switch / low-level")
            return True

        except Exception as e:
            logger.error(f"[Go2] stand_down_and_release error: {e}")
            return False

    def get_sport_state(self) -> SportModeState_ | None:
        """Return the latest SportModeState_ snapshot for diagnostics.

        Returned object is the live SDK message — do not mutate it. None
        if no state message has arrived.
        """
        session = self._get_session()
        with session.lock:
            return session.latest_state

    def get_status(self) -> dict:
        """One-shot snapshot of adapter + robot state.

        Does NOT raise if disconnected — returns a minimal dict instead.
        Known mode list is documentation (the SDK's documented names), not
        a probe of the specific firmware. Some firmwares ship without
        'normal'; there is no non-disruptive way to detect that.

        Returns:
            {
              'connected': bool,
              'mode': str | None,          — current MotionSwitcher mode
              'preferred_mode': str | None,
              'enabled': bool,
              'locomotion_ready': bool,
              'speed_level': int,
              'has_state': bool,           — first SportModeState received?
              'velocity': [vx, vy, wz] | None,
              'position': [x, y, theta] | None,
              'body_height': float | None,
              'sport_mode_num': int | None, — SportModeState.mode integer
              'known_modes': tuple[str, ...],
            }
        """
        known = self._SPORT_MODE_CANDIDATES

        with self._session_lock:
            session = self._session

        if session is None:
            return {
                "connected": False,
                "mode": None,
                "preferred_mode": self._preferred_mode,
                "enabled": False,
                "locomotion_ready": False,
                "speed_level": self._speed_level,
                "has_state": False,
                "velocity": None,
                "position": None,
                "body_height": None,
                "sport_mode_num": None,
                "known_modes": known,
            }

        # Current mode via MotionSwitcher (cheap RPC; tolerate failure).
        try:
            mode = self.check_mode()
        except Exception:
            mode = None

        # Snapshot state under session lock.
        with session.lock:
            state = session.latest_state
            enabled = session.enabled
            locomotion_ready = session.locomotion_ready

        velocity: list[float] | None = None
        position: list[float] | None = None
        body_height: float | None = None
        sport_mode_num: int | None = None

        if state is not None:
            try:
                velocity = [
                    float(state.velocity[0]),
                    float(state.velocity[1]),
                    float(state.imu_state.gyroscope[2]),
                ]
                position = [
                    float(state.position[0]),
                    float(state.position[1]),
                    float(state.imu_state.rpy[2]),
                ]
                body_height = float(state.body_height)
                sport_mode_num = int(state.mode)
            except Exception:
                pass

        return {
            "connected": True,
            "mode": mode,
            "preferred_mode": self._preferred_mode,
            "enabled": enabled,
            "locomotion_ready": locomotion_ready,
            "speed_level": self._speed_level,
            "has_state": state is not None,
            "velocity": velocity,
            "position": position,
            "body_height": body_height,
            "sport_mode_num": sport_mode_num,
            "known_modes": known,
        }

    def print_status(self) -> None:
        """Log a human-readable status summary. Safe whether connected or not."""
        s = self.get_status()

        if not s["connected"]:
            logger.info(
                "[Go2] status: DISCONNECTED  "
                f"(preferred_mode={s['preferred_mode']!r}, "
                f"speed_level={s['speed_level']}, "
                f"known_modes={list(s['known_modes'])})"
            )
            return

        mode_str = s["mode"] if s["mode"] is not None else "?"
        mode_marker = "✓" if mode_str == s["preferred_mode"] else "≠"

        vel = s["velocity"]
        pos = s["position"]
        vel_str = f"[{vel[0]:+.2f},{vel[1]:+.2f},{vel[2]:+.2f}]" if vel is not None else "none"
        pos_str = f"[{pos[0]:+.2f},{pos[1]:+.2f},{pos[2]:+.2f}]" if pos is not None else "none"
        bh = f"{s['body_height']:.3f}m" if s["body_height"] is not None else "?"

        logger.info(
            f"[Go2] status: CONNECTED  "
            f"mode='{mode_str}' {mode_marker} preferred='{s['preferred_mode']}'  "
            f"enabled={s['enabled']}  locomotion_ready={s['locomotion_ready']}  "
            f"speed_level={s['speed_level']}  "
            f"sport_mode_num={s['sport_mode_num']}  "
            f"body_height={bh}  "
            f"velocity={vel_str}  position={pos_str}  "
            f"known_modes={list(s['known_modes'])}"
        )

    def set_speed_level(self, level: int) -> bool:
        """Set the SportClient speed envelope at runtime.

        Go2 SDK convention: -1 = slow, 0 = normal, 1 = fast (max).
        Only reliably respected in 'normal' mode; mcf / ai controllers
        may ignore it. Updates self._speed_level so subsequent
        _initialize_locomotion() calls apply the same level.

        Returns True if the RPC returned 0.
        """
        session = self._get_session()
        try:
            with session.lock:
                ret = session.client.SpeedLevel(level)
        except Exception as e:
            logger.error(f"[Go2] SpeedLevel raised: {e}")
            return False

        if ret != 0:
            logger.warning(f"[Go2] SpeedLevel({level}) returned {ret}")
            return False

        self._speed_level = level
        logger.info(f"[Go2] ✓ SpeedLevel set to {level}")
        return True

    # =========================================================================
    # Low-level stubs (defined but unwired — real impl in adapter_lowlevel.py)
    # =========================================================================

    def subscribe_low_state(self, callback: Callable[[LowState_], None]) -> bool:
        """Subscribe to rt/lowstate with the given callback.

        Stub in this adapter — returns False and logs a pointer to
        UnitreeGo2LowLevelAdapter in adapter_lowlevel.py. The tick loop
        does not need low-level state; this exists so advanced users can
        reach raw joint telemetry through the same adapter handle if a
        future tick-loop extension wants it. Kept as a stub so the main
        adapter doesn't carry LowState_ import/DDS overhead.
        """
        logger.warning(
            "[Go2] subscribe_low_state is a stub on UnitreeGo2TwistAdapter. "
            "Use UnitreeGo2LowLevelAdapter for rt/lowstate access."
        )
        return False

    def publish_low_cmd(self, cmd: LowCmd_) -> bool:
        """Publish a single LowCmd_ on rt/lowcmd.

        Stub in this adapter — use UnitreeGo2LowLevelAdapter for real
        low-level control (it provides the watchdog, CRC handling, and
        per-joint helpers that make LowCmd_ actually safe to use).
        """
        logger.warning(
            "[Go2] publish_low_cmd is a stub on UnitreeGo2TwistAdapter. "
            "Use UnitreeGo2LowLevelAdapter for rt/lowcmd publishing."
        )
        return False

    # =========================================================================
    # Internal helpers
    # =========================================================================

    def _get_session(self) -> _Session:
        """Return active session or raise RuntimeError if disconnected."""
        session = self._session
        if session is None:
            raise RuntimeError("Go2 not connected")
        return session

    def _activate_sport_mode(self) -> bool:
        """Bring MotionSwitcher into a usable sport mode.

        Logic:
          1. CheckMode() with up to 6 retries on 3102 (DDS discovery race).
          2. If persistent 3102: log guidance and return False.
             (No "sportmodestate publishing so proceed anyway" fallback —
             that masked real failures.)
          3. If already in 'normal': return True.
          4. If in another non-empty mode (ai/advanced/mcf): accept it
             rather than releasing (would drop servo). Log a warning that
             Move() may race with onboard controller.
          5. If empty: try _SPORT_MODE_CANDIDATES in order, polling
             CheckMode to confirm each SelectMode actually took effect.
        """
        session = self._get_session()

        try:
            code: int | None = None
            data: object = None
            for attempt in range(1, 7):
                try:
                    code, data = session.motion_switcher.CheckMode()
                except Exception as e:
                    logger.warning(f"[Go2] CheckMode attempt {attempt} raised: {e}")
                    code, data = None, None
                logger.info(f"[Go2] CheckMode attempt {attempt} -> code={code} data={data}")
                if code == 3102:
                    time.sleep(1.0)
                    continue
                break

            if code == 3102:
                logger.error(
                    "[Go2] MotionSwitcher unreachable (3102). Exit AI mode "
                    "(L2+B on remote), close the Unitree app, and verify "
                    "this host can reach the Go2 on the network."
                )
                return False

            if code == 0 and isinstance(data, dict):
                current = (data.get("name") or "").strip()
                if current == "normal":
                    logger.info("[Go2] Already in mode 'normal'")
                    return True
                if current:
                    logger.warning(
                        f"[Go2] In non-'normal' mode '{current}' (likely mcf). "
                        "Move() may race with onboard controller. NOT "
                        "auto-switching (robot would fall). To switch: "
                        "stand_down_and_release() then switch_mode('normal', "
                        "i_have_sat_the_robot=True)."
                    )
                    return True

            # No mode active — try candidates.
            for name in self._SPORT_MODE_CANDIDATES:
                logger.info(f"[Go2] SelectMode('{name}')...")
                try:
                    sel_code, _ = session.motion_switcher.SelectMode(name)
                except Exception as e:
                    logger.warning(f"[Go2] SelectMode('{name}') raised: {e}")
                    continue
                logger.info(f"[Go2] SelectMode('{name}') -> code={sel_code}")

                if sel_code == 3102:
                    logger.error(
                        "[Go2] SelectMode RPC failed (3102) — check network reachability to Go2."
                    )
                    return False

                active = self._poll_mode(want_nonempty=True, timeout=4.0)
                if active:
                    logger.info(f"[Go2] ✓ Sport mode '{active}' active")
                    time.sleep(2.0)
                    return True

                logger.warning(f"[Go2] SelectMode('{name}') did not activate — trying next")
                try:
                    session.motion_switcher.ReleaseMode()
                except Exception:
                    pass
                time.sleep(0.5)

            logger.error(
                f"[Go2] None of {self._SPORT_MODE_CANDIDATES} activated. "
                "Exit AI mode (L2+B), close the Unitree app."
            )
            return False

        except Exception as e:
            logger.error(f"[Go2] _activate_sport_mode error: {e}")
            return False

    def _poll_mode(
        self,
        want_nonempty: bool = True,
        timeout: float = 4.0,
    ) -> str | None:
        """Poll CheckMode() until the mode name becomes (non-)empty.

        Returns the final name string (possibly empty) or None on RPC
        failure. Used to confirm SelectMode / ReleaseMode transitions.
        """
        session = self._get_session()
        deadline = time.monotonic() + timeout
        last_name: str | None = None
        while time.monotonic() < deadline:
            try:
                code, data = session.motion_switcher.CheckMode()
                if code == 0 and isinstance(data, dict):
                    last_name = (data.get("name") or "").strip()
                    if want_nonempty and last_name:
                        return last_name
                    if not want_nonempty and not last_name:
                        return last_name
            except Exception:
                pass
            time.sleep(0.3)
        return last_name

    def _initialize_locomotion(self) -> bool:
        """StandUp (retry 3102) → 3s settle → FreeWalk → 2s settle.

        Called from connect() and from write_enable(True) if locomotion
        was not yet ready. Assumes _activate_sport_mode() has already run.
        """
        session = self._get_session()

        if not self._activate_sport_mode():
            return False

        try:
            logger.info("[Go2] Standing up...")
            ret: int | None = None
            for attempt in range(1, 6):
                with session.lock:
                    ret = session.client.StandUp()
                if ret == 0:
                    break
                if ret == 3102:
                    logger.warning(f"[Go2] StandUp attempt {attempt} got 3102, retrying")
                    time.sleep(1.0)
                    continue
                break

            if ret != 0:
                logger.error(f"[Go2] StandUp failed with code {ret}")
                return False
            time.sleep(3)

            logger.info("[Go2] Activating FreeWalk...")
            with session.lock:
                ret = session.client.FreeWalk()
            if ret != 0:
                logger.error(f"[Go2] FreeWalk failed with code {ret}")
                return False
            time.sleep(2)

            # Apply speed profile. Non-fatal if it fails — mcf / ai modes
            # may ignore SpeedLevel, which is fine; Move() still works.
            try:
                with session.lock:
                    sl_ret = session.client.SpeedLevel(self._speed_level)
                if sl_ret == 0:
                    logger.info(f"[Go2] ✓ SpeedLevel({self._speed_level}) applied")
                else:
                    logger.warning(
                        f"[Go2] SpeedLevel({self._speed_level}) returned "
                        f"{sl_ret} — likely ignored by non-'normal' controller"
                    )
            except Exception as e:
                logger.warning(f"[Go2] SpeedLevel raised (non-fatal): {e}")

            session.locomotion_ready = True
            logger.info("[Go2] ✓ Locomotion ready")
            return True

        except Exception as e:
            logger.error(f"[Go2] _initialize_locomotion error: {e}")
            return False

    def _send_velocity(self, vx: float, vy: float, wz: float) -> bool:
        """Send raw SportClient.Move(vx, vy, wz) under session.lock.

        Returns False on SDK exception or non-zero return code.
        """
        session = self._get_session()
        try:
            with session.lock:
                ret = session.client.Move(vx, vy, wz)
            if ret != 0:
                logger.warning(f"[Go2] Move() returned code {ret}")
                return False
            return True
        except Exception as e:
            logger.error(f"[Go2] _send_velocity error: {e}")
            return False


def register(registry: TwistBaseAdapterRegistry) -> None:
    """Register this adapter with the TwistBaseAdapterRegistry under
    the name 'unitree_go2'."""
    registry.register("unitree_go2", UnitreeGo2TwistAdapter)


__all__ = ["UnitreeGo2TwistAdapter"]
