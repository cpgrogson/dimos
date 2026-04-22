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

"""Direct-joint (LowCmd_ / LowState_) adapter for the Unitree Go2.

Standalone class — instantiated manually, NOT registered through the
TwistBaseAdapterRegistry. The registry only auto-discovers modules named
adapter.py, so this file is safely invisible to discovery.

Prerequisites (the caller is responsible for these):
  1. Robot is sat down or damped before connect() is called.
  2. No sport controller is running — MotionSwitcher.CheckMode returns
     an empty name. rt/lowcmd is accepted at the motor-controller
     firmware layer regardless of MotionSwitcher; the "no active
     controller" requirement is only to prevent a fight with whatever
     mcf / normal / ai is already publishing. Release via the physical
     RC (L2+A) or the Unitree app before connect().

What this adapter does:
  - Publishes LowCmd_ on rt/lowcmd at up to 500 Hz, per-joint {q, dq, tau,
    kp, kd, mode=enable}, CRC32 set on every message.
  - Subscribes LowState_ on rt/lowstate for joint feedback.
  - Runs a safety watchdog thread that auto-damps if flush() is not
    called for > _WATCHDOG_TIMEOUT_S. LowCmd_ has no built-in watchdog:
    a dead publisher leaves the last torque latched.

Joint layout follows Unitree's canonical order:
    FR hip=0, thigh=1, calf=2
    FL hip=3, thigh=4, calf=5
    RR hip=6, thigh=7, calf=8
    RL hip=9, thigh=10, calf=11
MotorCmd_ has 20 slots (Go2 uses 12); slots 12..19 are written as
disabled with zero gains.
"""

from __future__ import annotations

import threading
import time
from typing import Any

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
    MotionSwitcherClient,
)
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# Canonical Go2 joint indexing for rt/lowcmd's MotorCmd_[20] array.
GO2_JOINT_INDEX: dict[tuple[str, str], int] = {
    ("FR", "hip"): 0,
    ("FR", "thigh"): 1,
    ("FR", "calf"): 2,
    ("FL", "hip"): 3,
    ("FL", "thigh"): 4,
    ("FL", "calf"): 5,
    ("RR", "hip"): 6,
    ("RR", "thigh"): 7,
    ("RR", "calf"): 8,
    ("RL", "hip"): 9,
    ("RL", "thigh"): 10,
    ("RL", "calf"): 11,
}

GO2_NUM_MOTORS: int = 20  # LowCmd_.motor_cmd array length
GO2_NUM_ACTIVE_JOINTS: int = 12  # Go2 uses 12 of the 20 slots

# If no flush() happens within this window the watchdog fires
_WATCHDOG_TIMEOUT_S: float = 0.2

# Damping gains used by emergency_damp() and watchdog. kd only — no kp,
_DAMP_KP: float = 0.0
_DAMP_KD: float = 1.0

# Foot-force sum (N) above which we treat the robot as still standing.
_FOOT_FORCE_DAMPED_THRESHOLD_N: float = 50.0

# PMSM motor enable byte (Unitree convention: 0x01 = enabled, 0x00 = disabled).
_MOTOR_MODE_ENABLE: int = 0x01
_MOTOR_MODE_DISABLE: int = 0x00


class UnitreeGo2LowLevelAdapter:
    """Direct LowCmd_ / LowState_ adapter for the Go2.

    Args:
        assume_dds_initialized: If True, skip ChannelFactoryInitialize —
            use this when running alongside UnitreeGo2TwistAdapter in the
            same process (ChannelFactoryInitialize is not idempotent).
            Default False.

    NOT thread-safe for concurrent writes from multiple threads. Expected
    usage: one control thread calling write_joint_* at a steady rate,
    plus the internal watchdog thread.
    """

    def __init__(self, assume_dds_initialized: bool = False) -> None:
        self._assume_dds_initialized = assume_dds_initialized

        self._connected = False
        self._lock = threading.Lock()  # serializes _cmd + publisher.Write
        self._state_lock = threading.Lock()  # guards _latest_low_state

        self._latest_low_state: LowState_ | None = None
        self._cmd: LowCmd_ | None = None
        self._publisher: ChannelPublisher | None = None
        self._subscriber: ChannelSubscriber | None = None
        self._motion_switcher: MotionSwitcherClient | None = None
        self._crc_helper: CRC | None = None
        self._last_user_flush_ts: float = 0.0

        self._watchdog_stop = threading.Event()
        self._watchdog_thread: threading.Thread | None = None

    # =========================================================================
    # Lifecycle
    # =========================================================================

    def connect(self) -> bool:
        """Bring up DDS publisher/subscriber and verify preconditions.

        Steps:
          1. ChannelFactoryInitialize(0) if assume_dds_initialized is False.
          2. Subscribe rt/lowstate → _on_lowstate() callback.
          3. Wait for first LowState_ message (timeout ~3s).
          4. Verify MotionSwitcher reports no active sport controller.
             Refuse with guidance if one is running — we don't want to
             fight mcf / normal / ai on the motor rail.
          5. Verify foot_force averages are near zero (robot is sat / damped).
             If not, refuse — publishing LowCmd_ to a standing robot whose
             controller just got killed will cause lurching.
          6. Create publisher on rt/lowcmd.
          7. Build LowCmd_ defaults (head, level_flag, gpio, bandwidth).
          8. Start watchdog thread.

        Returns True on success. On failure, all resources set up so far
        are torn down before returning.
        """
        try:
            if not self._assume_dds_initialized:
                ChannelFactoryInitialize(0)

            self._subscriber = ChannelSubscriber("rt/lowstate", LowState_)
            self._subscriber.Init(self._on_lowstate, 10)

            got_state = False
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                with self._state_lock:
                    if self._latest_low_state is not None:
                        got_state = True
                        break
                time.sleep(0.05)
            if not got_state:
                logger.error(
                    "[Go2 LowLevel] No LowState_ message in 3s — robot offline or wrong DDS domain?"
                )
                self._teardown()
                return False

            # MotionSwitcher precondition.
            self._motion_switcher = MotionSwitcherClient()
            self._motion_switcher.SetTimeout(5.0)
            self._motion_switcher.Init()
            time.sleep(1.0)

            if not self._verify_no_sport_controller():
                self._teardown()
                return False

            # Damped-state precondition.
            if not self._verify_robot_damped():
                self._teardown()
                return False

            # Publisher on rt/lowcmd.
            self._publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
            self._publisher.Init()

            self._cmd = self._build_lowcmd_defaults()

            self._connected = True
            self._last_user_flush_ts = time.monotonic()

            self._watchdog_stop.clear()
            self._watchdog_thread = threading.Thread(
                target=self._watchdog_loop,
                name="go2-lowlevel-watchdog",
                daemon=True,
            )
            self._watchdog_thread.start()

            logger.info("[Go2 LowLevel] Connected")
            return True

        except Exception as e:
            logger.error(f"[Go2 LowLevel] connect error: {e}")
            self._teardown()
            return False

    def disconnect(self) -> None:
        """Stop the watchdog, emergency_damp() once, then tear down DDS.

        Explicitly Close()s the LowState_ subscriber to prevent reader
        leaks across reconnects.
        """
        self._teardown()

    def is_connected(self) -> bool:
        """True iff connect() completed successfully and disconnect() has
        not been called."""
        return self._connected

    # =========================================================================
    # Joint command API
    # =========================================================================

    def write_joint_cmd(
        self,
        idx: int,
        *,
        q: float,
        dq: float = 0.0,
        tau: float = 0.0,
        kp: float,
        kd: float,
    ) -> bool:
        """Stage a command for one joint in the internal LowCmd_ buffer.

        Does NOT publish until flush() is called (or another write_joint_*
        variant flushes as part of its call — see write_joint_array).

        Does NOT reset the watchdog timer — staging alone does not push
        data to the robot. Call flush() to actually publish AND keep the
        watchdog happy, or use write_joint_array which auto-flushes.

        Args:
            idx: Joint index 0..11 (use GO2_JOINT_INDEX for named lookup).
            q: Target joint position (rad).
            dq: Target joint velocity (rad/s).
            tau: Feedforward torque (N·m).
            kp: Position PD gain. Use 0.0 for torque-only control.
            kd: Velocity PD gain. Use >0 for damping even with kp=0.

        Returns False if not connected or idx out of range. Does not
        validate gain ranges — caller must pass safe values.
        """
        if not self._connected or self._cmd is None:
            return False
        if idx < 0 or idx >= GO2_NUM_ACTIVE_JOINTS:
            logger.warning(
                f"[Go2 LowLevel] joint idx {idx} out of range [0, {GO2_NUM_ACTIVE_JOINTS})"
            )
            return False

        with self._lock:
            m = self._cmd.motor_cmd[idx]
            m.mode = _MOTOR_MODE_ENABLE
            m.q = float(q)
            m.dq = float(dq)
            m.tau = float(tau)
            m.kp = float(kp)
            m.kd = float(kd)
        return True

    def write_joint_array(
        self,
        qs: list[float],
        dqs: list[float],
        taus: list[float],
        kps: list[float],
        kds: list[float],
    ) -> bool:
        """Batch form: set all 12 joints and publish in one call.

        Each list must have length 12 (active joints only). Auto-flushes
        after staging. Resets the watchdog timer on success.

        Returns False if any list has wrong length, or if not connected.
        """
        n = GO2_NUM_ACTIVE_JOINTS
        if any(len(v) != n for v in (qs, dqs, taus, kps, kds)):
            logger.warning(f"[Go2 LowLevel] write_joint_array expected length {n} per arg")
            return False
        if not self._connected or self._cmd is None:
            return False

        with self._lock:
            for i in range(n):
                m = self._cmd.motor_cmd[i]
                m.mode = _MOTOR_MODE_ENABLE
                m.q = float(qs[i])
                m.dq = float(dqs[i])
                m.tau = float(taus[i])
                m.kp = float(kps[i])
                m.kd = float(kds[i])

        return self.flush()

    def flush(self) -> bool:
        """Compute CRC on the staged LowCmd_ and publish to rt/lowcmd.

        Resets the watchdog timer on success. Explicit flush is available
        for callers that want to stage multiple joints with write_joint_cmd
        and then publish once per tick.
        """
        if not self._connected or self._cmd is None or self._publisher is None:
            return False

        with self._lock:
            self._compute_crc(self._cmd)
            self._publisher.Write(self._cmd)
        self._last_user_flush_ts = time.monotonic()
        return True

    # =========================================================================
    # Joint state readout
    # =========================================================================

    def read_joint_state(self) -> dict[str, Any]:
        """Snapshot of the most recent LowState_ message.

        Returns:
            {
              'motors': [ {'q': float, 'dq': float, 'tau_est': float,
                           'temperature': int}, ... 12 entries ],
              'imu': {'quaternion': [w,x,y,z], 'gyroscope': [x,y,z],
                      'accelerometer': [x,y,z], 'rpy': [r,p,y]},
              'foot_force': [fr, fl, rr, rl],
              'tick': int,
            }
            Or {} if no LowState_ message has arrived yet.
        """
        with self._state_lock:
            state = self._latest_low_state
        if state is None:
            return {}

        motors = [
            {
                "q": float(state.motor_state[i].q),
                "dq": float(state.motor_state[i].dq),
                "tau_est": float(state.motor_state[i].tau_est),
                "temperature": int(state.motor_state[i].temperature),
            }
            for i in range(GO2_NUM_ACTIVE_JOINTS)
        ]
        return {
            "motors": motors,
            "imu": {
                "quaternion": [float(x) for x in state.imu_state.quaternion],
                "gyroscope": [float(x) for x in state.imu_state.gyroscope],
                "accelerometer": [float(x) for x in state.imu_state.accelerometer],
                "rpy": [float(x) for x in state.imu_state.rpy],
            },
            "foot_force": [float(x) for x in state.foot_force],
            "tick": int(state.tick),
        }

    # =========================================================================
    # Safety
    # =========================================================================

    def emergency_damp(self) -> bool:
        """Immediately publish a LowCmd_ that damps all joints.

        All 12 joints: q=<current q from LowState_>, dq=0, tau=0,
        kp=_DAMP_KP (0.0), kd=_DAMP_KD (mild).

        Safe to call from any thread — does not wait on external locks.
        Called by disconnect() and by the watchdog when a publisher goes
        silent. Returns False if no LowState_ has arrived yet (no current
        q to hold) — in that case the caller should abort cleanly rather
        than publish a zero-q command (would fight gravity hard).

        Uses a fresh LowCmd_ buffer so it does not clobber user-staged
        commands in the main _cmd buffer.
        """
        if not self._connected or self._publisher is None:
            return False

        with self._state_lock:
            state = self._latest_low_state
        if state is None:
            logger.error("[Go2 LowLevel] emergency_damp: no LowState_ yet, cannot hold current q")
            return False

        damp_cmd = self._build_lowcmd_defaults()
        for i in range(GO2_NUM_ACTIVE_JOINTS):
            m = damp_cmd.motor_cmd[i]
            m.mode = _MOTOR_MODE_ENABLE
            m.q = float(state.motor_state[i].q)
            m.dq = 0.0
            m.tau = 0.0
            m.kp = _DAMP_KP
            m.kd = _DAMP_KD
        with self._lock:
            self._compute_crc(damp_cmd)
            self._publisher.Write(damp_cmd)
        return True

    # =========================================================================
    # Internal
    # =========================================================================

    def _teardown(self) -> None:
        """Stop watchdog, final damp, close DDS resources. Idempotent."""
        if self._watchdog_thread is not None:
            self._watchdog_stop.set()
            self._watchdog_thread.join(timeout=2.0)
            self._watchdog_thread = None

        # Best-effort final damp while resources are still up.
        if self._connected and self._publisher is not None:
            try:
                self.emergency_damp()
            except Exception as e:
                logger.warning(f"[Go2 LowLevel] emergency_damp during teardown: {e}")

        self._connected = False

        if self._subscriber is not None:
            try:
                self._subscriber.Close()
            except Exception as e:
                logger.error(f"[Go2 LowLevel] Error closing subscriber: {e}")
            self._subscriber = None

        # ChannelPublisher has no explicit Close in SDK2 — release refs.
        self._publisher = None
        self._motion_switcher = None
        self._cmd = None

    def _verify_no_sport_controller(self) -> bool:
        """Check MotionSwitcher reports empty (no sport controller).

        Go2 low-level (rt/lowcmd) works whenever no sport controller is
        running — nothing fights our motor commands at the motor-controller
        firmware layer. Returns False if 'normal' / 'mcf' / 'ai' / any
        other name is still active; caller should release first.
        """
        if self._motion_switcher is None:
            return False
        code, data = self._motion_switcher.CheckMode()

        if code == 3102:
            logger.error("[Go2 LowLevel] MotionSwitcher unreachable (3102)")
            return False
        if code != 0 or not isinstance(data, dict):
            logger.error(f"[Go2 LowLevel] CheckMode returned code={code}")
            return False

        current = (data.get("name") or "").strip()
        if current == "":
            logger.info("[Go2 LowLevel] No sport controller active — OK for low-level")
            return True

        logger.error(
            f"[Go2 LowLevel] Sport controller '{current}' is running — "
            "low-level is unsafe. Release it via the physical RC (L2+A) or "
            "the Unitree app so CheckMode returns an empty name, then retry."
        )
        return False

    def _verify_robot_damped(self, timeout_s: float = 2.0) -> bool:
        """Check LowState_.foot_force averages are near zero.

        Used as a precondition in connect() — publishing LowCmd_ to a
        still-standing robot whose controller was just killed is unsafe.
        """
        deadline = time.monotonic() + timeout_s
        samples: list[list[float]] = []
        while time.monotonic() < deadline:
            with self._state_lock:
                state = self._latest_low_state
            if state is not None:
                samples.append([float(x) for x in state.foot_force])
            time.sleep(0.05)

        if not samples:
            logger.error("[Go2 LowLevel] No foot_force samples to verify damped state")
            return False

        # Average the last ~10 samples (or fewer if we got less).
        last = samples[-10:] if len(samples) >= 10 else samples
        avg_per_foot = [sum(s[i] for s in last) / len(last) for i in range(4)]
        total = sum(abs(f) for f in avg_per_foot)

        if total > _FOOT_FORCE_DAMPED_THRESHOLD_N:
            logger.error(
                f"[Go2 LowLevel] Robot appears to be standing "
                f"(foot_force sum={total:.1f}N > "
                f"{_FOOT_FORCE_DAMPED_THRESHOLD_N}N threshold). "
                "Sit the robot down first."
            )
            return False

        logger.info(f"[Go2 LowLevel] Robot damped (foot_force sum={total:.1f}N)")
        return True

    def _on_lowstate(self, msg: LowState_) -> None:
        """DDS callback — store the latest LowState_ under _state_lock."""
        with self._state_lock:
            self._latest_low_state = msg

    def _build_lowcmd_defaults(self) -> LowCmd_:
        """Construct a LowCmd_ with head/level_flag/gpio/bandwidth set to
        the values from Unitree's go2_stand_example.py reference. All 20
        motor slots start with mode=0 (disabled) and zero gains."""
        cmd = unitree_go_msg_dds__LowCmd_()
        cmd.head[0] = 0xFE
        cmd.head[1] = 0xEF
        cmd.level_flag = 0xFF  # LOWLEVEL
        cmd.gpio = 0
        for i in range(GO2_NUM_MOTORS):
            m = cmd.motor_cmd[i]
            m.mode = _MOTOR_MODE_DISABLE
            m.q = 0.0
            m.dq = 0.0
            m.tau = 0.0
            m.kp = 0.0
            m.kd = 0.0
        return cmd

    def _compute_crc(self, cmd: LowCmd_) -> int:
        """Compute and set cmd.crc using unitree_sdk2py.utils.crc.CRC32.

        Must be called under self._lock (CRC helper may have internal
        state that is not thread-safe).
        """
        if self._crc_helper is None:
            self._crc_helper = CRC()
        cmd.crc = self._crc_helper.Crc(cmd)
        return int(cmd.crc)

    def _watchdog_loop(self) -> None:
        """Background thread body.

        Runs at ~100 Hz. If now - _last_user_flush_ts > _WATCHDOG_TIMEOUT_S
        and the adapter is still connected, calls emergency_damp(). Logs
        at most once per second so a silent publisher doesn't flood the
        logs. Exits when _watchdog_stop is set.
        """
        last_log_ts: float = 0.0
        while not self._watchdog_stop.wait(0.01):
            if not self._connected:
                return
            elapsed = time.monotonic() - self._last_user_flush_ts
            if elapsed > _WATCHDOG_TIMEOUT_S:
                now = time.monotonic()
                if now - last_log_ts > 1.0:
                    logger.warning(
                        f"[Go2 LowLevel] Watchdog: no user flush in {elapsed:.3f}s, holding damp"
                    )
                    last_log_ts = now
                try:
                    self.emergency_damp()
                except Exception as e:
                    logger.error(f"[Go2 LowLevel] Watchdog emergency_damp failed: {e}")


__all__ = [
    "GO2_JOINT_INDEX",
    "GO2_NUM_ACTIVE_JOINTS",
    "GO2_NUM_MOTORS",
    "UnitreeGo2LowLevelAdapter",
]
