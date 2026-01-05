# Copyright 2025 Dimensional Inc.
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

from __future__ import annotations

import math
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING, Optional

import mujoco
import mujoco.viewer as viewer

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable, Sequence

logger = setup_logger()
"""
All function and variables called from xarm_driver
Calling API
- port                  # not needed
- is_radian             # added as is_radian
- do_not_open           #
- check_tcp_limit       # not needed
- check_joint_limit     # added as check_joint_limit
- check_cmdnum_limit    # not needed
- check_robot_sn        # not needed
- check_is_ready        # not needed but default is True
- check_is_pause        # not needed
- max_cmdnum            # not needed
- num_joints             # added
- debug                 #
- report_type           # added as report_type
Additional parameters:
- joint_state_rate: float,   # publishing rate in Hz
- control_frequency: float,  # control frequency in Hz
- model_path: str,           # path to the Mujoco model


Methods
- release_connect_changed_callback(self, enable: bool) -> None:
- release_report_callback(self, enable: bool) -> None:
- register_connect_changed_callback(self, callback: Callable[[bool, bool], None]) -> None:
- register_report_callback(self, callback: Callable[[dict], None]) -> None:
- get_err_warn_code(self, err_warn: list[int]) -> int:

- connect(self) -> None:
- disconnect(self) -> int:
- set_mode(self, mode: int) -> int:
- set_state(self, state: int) -> int:
- get_cmdnum(self) -> tuple[int, int]:
- get_version(self) -> tuple[int, str]:
- get_joint_states(self, is_radian=None) -> tuple[int, list[list[float]]]:
- get_servo_angle(self, is_radian: bool = True) -> tuple[int, Sequence[float]]:
- set_servo_angle_j(self, angles: Sequence[float], is_radian: bool = True, **kwargs) -> int:
- vc_set_joint_velocity(self, speeds: Sequence[float], is_radian: bool = True, duration: Optional[float] = None, **kwargs) -> int:


"""


class XArmSimBridge:
    """
    Lightweight, in-process backend that mimics the subset of the UFACTORY xArm
    SDK used by ``XArmDriver``.

    The bridge keeps an internal joint state, emits periodic report callbacks,
    and provides best-effort implementations for the large SDK surface so the
    rest of the driver stack can operate without modification.
    """

    def __init__(
        self,
        is_radian: bool,  # use radians (True) or degrees (False)
        check_joint_limit: bool,
        num_joints: int,  # number of joints
        report_type: str,  # report type
        joint_state_rate: float,  # publishing rate in Hz
        control_frequency: float,  # control frequency in Hz
    ):
        # Path: dimos/hardware/manipulators/xarm/ -> dimos/simulation/manipulation/data/xarm7/
        self._model_path = (
            Path(__file__).parent.parent.parent.parent
            / "simulation"
            / "manipulation"
            / "data"
            / "xarm7"
            / "scene.xml"
        )

        self._is_radian = is_radian  # Store the unit preference (matches XArmAPI behavior)
        self._num_joints = num_joints
        self._report_type = 100 if report_type == "dev" else 5
        self._joint_state_rate = joint_state_rate if joint_state_rate > 0 else 0.01
        self._control_frequency = control_frequency if control_frequency > 0 else 0.01

        # --- mujoco model & data --- #
        self._model = mujoco.MjModel.from_xml_path(str(self._model_path))
        self._data = mujoco.MjData(self._model)

        # --- state variables --- #
        self._connected: bool = False
        self._mode = 0
        self._state = 0
        self._cmdnum = 0
        self._err_warn_code = 0
        self._warn_code = 0
        self._motion_enabled = True

        # --- joint targets and measured states --- #
        self._lock = threading.Lock()  # lock for thread safety during state updates
        self._stop_event = threading.Event()
        self._sim_thread: threading.Thread | None = None
        self._report_thread: threading.Thread | None = None
        self._connect_callback: Callable[[bool, bool], None] | None = None
        self._report_callback: Callable[[dict], None] | None = None

        # Initialize position targets from current model state (hold current position)
        # This prevents the arm from moving to zero when motion is first enabled
        self._joint_position_targets = [0.0] * self._num_joints
        self._joint_velocity_targets = [
            0.0
        ] * self._num_joints  # Target velocities for velocity control
        self._joint_positions = [0.0] * self._num_joints
        self._joint_velocities = [0.0] * self._num_joints
        self._joint_efforts = [0.0] * self._num_joints

        self._velocity_control = False
        self._velocity_control_positions = [0.0] * self._num_joints
        # For velocity control: track integrated position targets (accumulate velocity over time)
        # Since XML uses position-controlled actuators, we integrate velocities to positions
        self._hold_positions = [0.0] * self._num_joints

        # Initialize position targets to current joint positions at startup
        # This locks the joints at their initial pose, preventing falling due to gravity
        for i in range(min(self._num_joints, self._model.nq)):
            current_pos = float(self._data.qpos[i])
            self._joint_position_targets[i] = current_pos
            self._hold_positions[i] = current_pos
            self._velocity_control_positions[i] = current_pos

        # Force/torque sensor data (simulated)
        self._ft_ext_force = [0.0] * 6
        self._ft_raw_force = [0.0] * 6

        # Version number (simulated firmware version)
        self._version_number = (1, 8, 103)  # Simulate newer firmware

        # Core object for optional methods
        self._core = self._CoreProxy(self)

    # ------------------------------------------------------------------ #
    # Properties (matching XArmAPI interface)
    # ------------------------------------------------------------------ #
    @property
    def connected(self) -> bool:
        return self._connected

    @connected.setter
    def connected(self, value: bool) -> None:
        self._connected = value

    @property
    def version(self) -> str:
        """Firmware version string (matches XArmAPI.version)."""
        return "XArmSimBridge v1.0.0"

    @property
    def version_number(self) -> tuple[int, int, int]:
        """Firmware version number (major, minor, patch)."""
        return self._version_number

    @property
    def mode(self) -> int:
        """Current control mode (matches XArmAPI.mode)."""
        return self._mode

    @property
    def state(self) -> int:
        """Current robot state (matches XArmAPI.state)."""
        return self._state

    @property
    def error_code(self) -> int:
        """Current error code (matches XArmAPI.error_code). 0 = no error."""
        return 0

    @property
    def warn_code(self) -> int:
        """Current warning code (matches XArmAPI.warn_code). 0 = no warning."""
        return self._warn_code

    @property
    def cmd_num(self) -> int:
        """Number of commands in queue (matches XArmAPI.cmd_num)."""
        return self._cmdnum

    @property
    def ft_ext_force(self) -> list[float]:
        """External force/torque (compensated) - 6 elements [fx, fy, fz, tx, ty, tz]."""
        with self._lock:
            return list(self._ft_ext_force)

    @property
    def ft_raw_force(self) -> list[float]:
        """Raw force/torque sensor data - 6 elements [fx, fy, fz, tx, ty, tz]."""
        with self._lock:
            return list(self._ft_raw_force)

    @property
    def core(self):
        """Core API proxy for advanced methods."""
        return self._core

    class _CoreProxy:
        """Proxy object for core API methods."""

        def __init__(self, bridge):
            self._bridge = bridge

        def servo_get_dbmsg(self, dbg_msg: list[int]) -> None:
            """Get servo debug messages (stub for simulation)."""
            # In simulation, no servo errors
            for i in range(len(dbg_msg)):
                dbg_msg[i] = 0

    # ------------------------------------------------------------------ #
    # Lifecycle callbacks
    # ------------------------------------------------------------------ #
    def release_connect_changed_callback(self, enable: bool) -> None:
        if enable:
            self._connect_callback = None

    def release_report_callback(self, enable: bool) -> None:
        if enable:
            self._report_callback = None

    def register_connect_changed_callback(self, callback: Callable[[bool, bool], None]) -> None:
        self._connect_callback = callback
        if self.connected:
            callback(True, True)

    def register_report_callback(self, callback: Callable[[dict], None]) -> None:
        self._report_callback = callback

    # ------------------------------------------------------------------ #
    # Connection management
    # ------------------------------------------------------------------ #
    def connect(self) -> None:
        logger.info("XArmSimBridge: connect()")
        with self._lock:  # is this needed?
            self.connected = True
            self._stop_event.clear()
            self._last_update_time = time.time()

        if self._connect_callback:
            self._connect_callback(True, True)

        # start sim thread
        if self._sim_thread is None or not self._sim_thread.is_alive():
            self._sim_thread = threading.Thread(
                target=self._sim_loop, name="XArmSimBridgeSim", daemon=True
            )
            self._sim_thread.start()

        # start report thread
        if self._report_thread is None or not self._report_thread.is_alive():
            self._report_thread = threading.Thread(
                target=self._report_loop, name="XArmSimBridgeReport", daemon=True
            )
            self._report_thread.start()

    def disconnect(self) -> int:
        logger.info("XArmSimBridge: disconnect()")
        with self._lock:  # is this needed?
            self.connected = False
            self._stop_event.set()  # set the stop event to signal the threads to stop

        if self._sim_thread and self._sim_thread.is_alive():
            self._sim_thread.join(timeout=1.0)  #

        if self._report_thread and self._report_thread.is_alive():
            self._report_thread.join(timeout=1.0)

        if self._connect_callback:
            self._connect_callback(False, True)

        return 0

    # ------------------------------------------------------------------ #
    # Core state helpers
    # ------------------------------------------------------------------ #

    # can be further modified to return actual error and warning codes
    def get_err_warn_code(self, err_warn: list[int]) -> int:
        err_warn[0] = 0
        err_warn[1] = 0
        return 0

    def clean_error(self) -> int:
        return 0

    def clean_warn(self) -> int:
        return 0

    def motion_enable(self, enable: bool = True) -> tuple[int, str]:
        self._motion_enabled = bool(enable)
        return (0, "Simulated motion {}".format("enabled" if enable else "disabled"))

    # mode definition wrt mujoco sim (used modes = 0,1,4)
    def set_mode(self, mode: int) -> int:
        if mode == 0:
            pass
        elif mode == 1:
            pass
        elif mode == 4:
            pass
        else:
            raise ValueError(f"No definitions for mode: {mode}")

        self._mode = int(mode)
        return 0

    # state definitions accordingly with sim (used states = 0)
    def set_state(self, state: int) -> int:
        self._state = int(state)
        return 0

    def get_state(self) -> tuple[int, int]:
        return (0, self._state)

    def get_cmdnum(self) -> tuple[int, int]:
        return (0, self._cmdnum)

    def get_version(self) -> tuple[int, str]:
        return (0, "XArmSimBridge v1.0.0")

    def get_servo_version(self) -> tuple[int, list[str]]:
        """Get servo version info (matches XArmAPI.get_servo_version).

        Returns:
            Tuple of (error_code, [serial_number, ...])
        """
        return (0, ["SIM-XARM-001"])

    # ------------------------------------------------------------------ #
    # Joint state helpers
    # ------------------------------------------------------------------ #

    def _sim_loop(self) -> None:
        logger.info("XArmSimBridge: sim loop started")
        dt = 1.0 / self._control_frequency

        with viewer.launch_passive(
            self._model, self._data, show_left_ui=False, show_right_ui=False
        ) as m_viewer:
            while m_viewer.is_running() and not self._stop_event.is_set():
                # Get current targets and control mode
                with self._lock:
                    if self._velocity_control:
                        vel_targets = list(self._joint_velocity_targets)
                    else:
                        pos_targets = list(self._joint_position_targets)
                    motion_enabled = self._motion_enabled

                # Apply control
                # Always apply position control at startup to lock joints, even if motion is disabled
                # This prevents the robot from falling due to gravity when first launched
                if motion_enabled or (not hasattr(self, "_startup_complete")):
                    if not hasattr(self, "_startup_complete"):
                        # First iteration: ensure position targets are set to current positions
                        with self._lock:
                            for i in range(min(self._num_joints, self._model.nq)):
                                current_pos = float(self._data.qpos[i])
                                if abs(self._joint_position_targets[i]) < 1e-6:
                                    # If target is still zero, set it to current position
                                    self._joint_position_targets[i] = current_pos
                                    self._hold_positions[i] = current_pos
                        self._startup_complete = True

                    if self._velocity_control:
                        # Velocity control: integrate velocity targets to get position targets
                        # Since XML uses 'general' actuators (position control), we integrate
                        # velocities to positions: position = position + velocity * dt
                        for i in range(self._num_joints):
                            if i < self._model.nu:
                                # Check if velocity is effectively zero (stopped)
                                if abs(vel_targets[i]) < 1e-6:
                                    # When stopped, continuously sync to current position to hold
                                    # This prevents drift due to gravity or simulation dynamics
                                    if i < self._model.nq:
                                        self._velocity_control_positions[i] = self._hold_positions[
                                            i
                                        ]

                                else:
                                    # Integrate velocity: accumulate position over time
                                    self._velocity_control_positions[i] += vel_targets[i] * dt
                                    self._hold_positions[i] = self._velocity_control_positions[i]
                                # Set the integrated position as the control target
                                self._data.ctrl[i] = self._velocity_control_positions[i]
                                # self._data.ctrl[i+7] = vel_targets[i]
                    else:
                        # Position control: Set target positions for actuators
                        for i in range(self._num_joints):
                            if i < self._model.nu:
                                self._data.ctrl[i] = pos_targets[i]
                                # Sync velocity control positions when in position mode
                                # This ensures smooth transition when switching back to velocity mode
                                # if i < self._model.nq:
                                #     self._velocity_control_positions[i] = self._data.qpos[i]
                        self._hold_positions = list(pos_targets)

                # Step simulation
                mujoco.mj_step(self._model, self._data)
                m_viewer.sync()

                # Update joint state from simulation
                with self._lock:
                    for i in range(self._num_joints):
                        if i < self._model.nq:  # no of generalized coordinates
                            self._joint_positions[i] = float(self._data.qpos[i])
                        if i < self._model.nv:  # no of generalized velocities
                            self._joint_velocities[i] = float(self._data.qvel[i])
                        # Efforts from actuators
                        if i < self._model.nu:  # no of actuators
                            self._joint_efforts[i] = float(
                                self._data.qfrc_actuator[i]
                                if i < len(self._data.qfrc_actuator)
                                else 0.0
                            )

                time.sleep(dt)
        logger.info("XArmSimBridge: sim loop stopped")

    def _notify_report(self) -> None:
        callback = self._report_callback
        if not callback:
            return

        with self._lock:
            joints = list(self._joint_positions)
            data = {
                "state": self._state,
                "mode": self._mode,
                "error_code": 0,
                "warn_code": 0,
                "cmdnum": self._cmdnum,
                "cartesian": self._estimate_cartesian_pose(joints),
                "tcp_offset": [0.0] * 6,
                "joints": joints,
                "mtbrake": 0,
                "mtable": int(self._motion_enabled),
            }

        try:
            callback(data)
        except Exception as exc:
            logger.debug(f"XArmSimBridge report callback error: {exc}")

    def _report_loop(self) -> None:
        logger.info("XArmSimBridge: report loop started")
        self._report_period = 1.0 / self._report_type
        while not self._stop_event.is_set():
            self._notify_report()
            time.sleep(self._report_period)
        logger.info("XArmSimBridge: report loop stopped")

    # ------------------------------------------------------------------ #
    # Joint / motion commands
    # ------------------------------------------------------------------ #

    def get_servo_angle(self, is_radian: bool | None = None) -> tuple[int, Sequence[float]]:
        """Get current joint angles (matches XArmAPI.get_servo_angle).

        Args:
            is_radian: If True, return radians; if False, return degrees.
                       If None, uses the instance default (set in constructor).

        Returns:
            Tuple of (error_code, angles). Error code 0 = success.
        """
        if is_radian is None:
            is_radian = self._is_radian
        with self._lock:
            # Internal positions are in radians (MuJoCo uses radians)
            positions = list(self._joint_positions)
        if not is_radian:
            # Convert to degrees if requested
            positions = [math.degrees(p) for p in positions]
        return (0, positions)

    def get_joint_states(self, is_radian=None) -> tuple[int, list[list[float]]]:
        with self._lock:
            positions = list(self._joint_positions)
            velocities = list(self._joint_velocities)
            efforts = list(self._joint_efforts)
        return (0, [positions, velocities, efforts])

    def get_joint_speeds(self, is_radian: bool | None = None) -> tuple[int, Sequence[float]]:
        """Get current joint velocities (matches XArmAPI.get_joint_speeds).

        Returns:
            Tuple of (error_code, speeds). Error code 0 = success.
        """
        if is_radian is None:
            is_radian = self._is_radian
        with self._lock:
            # Internal velocities are in rad/s (MuJoCo uses radians)
            velocities = list(self._joint_velocities)
        if not is_radian:
            # Convert to deg/s if requested
            velocities = [math.degrees(v) for v in velocities]
        return (0, velocities)

    def get_joint_torques(self) -> tuple[int, Sequence[float]]:
        """Get current joint torques (matches XArmAPI.get_joint_torques).

        Returns:
            Tuple of (error_code, torques). Error code 0 = success.
        """
        with self._lock:
            efforts = list(self._joint_efforts)
        return (0, efforts)

    def set_servo_angle_j(
        self,
        angles: Sequence[float],
        is_radian: bool | None = None,
        **kwargs,
    ) -> int:
        """Set target joint angles for position control.

        Args:
            angles: Target joint angles
            is_radian: If True, angles are in radians; if False, in degrees.
                       If None, uses the instance default (set in constructor).
        """
        if is_radian is None:
            is_radian = self._is_radian
        with self._lock:
            values = list(angles)[: self._num_joints]
            if not is_radian:
                # Convert degrees to radians (MuJoCo uses radians internally)
                values = [math.radians(a) for a in values]
            # DEBUG: Log incoming commands
            logger.info(f"set_servo_angle_j: targets={[f'{v:.3f}' for v in values]} rad")
            # Set targets (not current positions) - sim loop will control to reach these
            for i, value in enumerate(values):
                self._joint_position_targets[i] = float(value)
            # Switch to position control mode
            self._velocity_control = False
            self._cmdnum += 1
        return 0

    def vc_set_joint_velocity(
        self,
        speeds: Sequence[float],
        is_radian: bool | None = None,
        duration: float | None = None,
        **kwargs,
    ) -> int:
        """
        Set target joint velocities for velocity control.

        Args:
            speeds: Target velocities for each joint
            is_radian: If True, speeds are in rad/s; if False, in deg/s.
                       If None, uses the instance default (set in constructor).
            duration: Optional duration hint (handled by trajectory generator, not used here)

        Note:
            The duration parameter is handled by the trajectory generator which stops
            sending commands after the duration. This method just sets the velocity targets.
            When all velocities are zero, the robot will hold its current position.
        """
        if is_radian is None:
            is_radian = self._is_radian
        self._velocity_control = True
        with self._lock:
            values = list(speeds)[: self._num_joints]
            if not is_radian:
                # Convert deg/s to rad/s (MuJoCo uses radians internally)
                values = [math.radians(a) for a in values]
            # Set targets (not current velocities) - sim loop will integrate these
            for i, value in enumerate(values):
                self._joint_velocity_targets[i] = float(value)
            # If all velocities are zero, sync integrated positions to current positions
            # This prevents drift and ensures smooth stopping
            if all(abs(v) < 1e-6 for v in values):
                for i in range(min(self._num_joints, self._model.nq)):
                    # Use thread-safe joint positions (updated by sim loop)
                    self._velocity_control_positions[i] = self._joint_positions[i]
            self._cmdnum += 1
        return 0

    # ------------------------------------------------------------------ #
    # Cartesian helpers (approximate placeholders)
    # ------------------------------------------------------------------ #
    def _estimate_cartesian_pose(self, joints: Sequence[float]) -> list[float]:
        pose = [0.0] * 6
        for i in range(min(3, len(joints))):
            pose[i] = joints[i]
        for i in range(3, min(6, len(joints))):
            pose[i] = joints[i]
        return pose

    def get_position(self, is_radian: bool = True) -> tuple[int, Sequence[float]]:
        with self._lock:
            pose = self._estimate_cartesian_pose(self._joint_positions)
        if not is_radian:
            pose = pose[:3] + [math.degrees(v) for v in pose[3:6]]
        return (0, pose)

    def get_position_aa(self, is_radian: bool = True) -> tuple[int, Sequence[float]]:
        with self._lock:
            magnitude = sum(self._joint_positions[:3]) if self._joint_positions else 0.0
        angle = magnitude if is_radian else math.degrees(magnitude)
        return (0, [0.0, 0.0, 1.0, angle])

    def set_position(self, *pose, is_radian: bool = True, wait: bool = False, **kwargs):
        """Set target Cartesian position (stub - would need IK to convert to joint angles)."""
        # Note: This is a simplified implementation. A full implementation would:
        # 1. Convert Cartesian pose to joint angles using inverse kinematics
        # 2. Set joint targets accordingly
        # For now, we'll just update the cartesian estimate (which is used in reports)
        with self._lock:
            vals = list(pose)
            if not is_radian and len(vals) >= 6:
                vals[3:6] = [math.radians(v) for v in vals[3:6]]
            # In a real implementation, we'd use IK here to convert to joint targets
            # For now, we'll just increment cmdnum to indicate a command was received
            self._cmdnum += 1
        return 0

    def move_gohome(self, wait: bool = False, is_radian: bool = True):
        home = [0.0] * self._num_joints
        return self.set_servo_angle_j(home, is_radian=is_radian)

    # ------------------------------------------------------------------ #
    # Force / torque sensor stubs
    # ------------------------------------------------------------------ #
    def get_ft_sensor_data(self) -> tuple[int, Sequence[float]]:
        return (0, [0.0] * 6)

    def get_ft_sensor_error(self) -> tuple[int, int]:
        return (0, 0)

    def get_ft_sensor_app_get(self) -> tuple[int, int]:
        return (0, 0)

    # ------------------------------------------------------------------ #
    # Emergency / resets
    # ------------------------------------------------------------------ #
    def emergency_stop(self) -> int:
        with self._lock:
            self._state = 4
            # Velocities are managed by MuJoCo, but we can zero them for safety
            self._joint_velocities = [0.0] * self._num_joints
        self._notify_report()
        return 0

    def clean_conf(self) -> int:
        return 0

    def save_conf(self) -> int:
        return 0

    def reload_dynamics(self) -> int:
        return 0

    # ------------------------------------------------------------------ #
    # Generic stubs for remaining SDK surface
    # ------------------------------------------------------------------ #
    def _simple_ok(self, *args, **kwargs) -> int:
        return 0

    def _tuple_ok_none(self, *args, **kwargs) -> tuple[int, None]:
        return (0, None)

    def _tuple_ok_zero(self, *args, **kwargs) -> tuple[int, int]:
        return (0, 0)

    def __getattr__(self, name: str):
        """
        Provide best-effort implementations for the large surface of the
        hardware SDK that the simulation may not support yet.
        """
        if name.startswith("set_") or name.startswith("clean_") or name.endswith("_enable"):
            return self._simple_ok
        if name.startswith("open_") or name.startswith("close_") or name.startswith("stop_"):
            return self._simple_ok
        if name.startswith("get_"):
            if any(token in name for token in ("digital", "analog", "state", "status", "error")):
                return self._tuple_ok_zero
            return self._tuple_ok_none
        raise AttributeError(f"{type(self).__name__!r} object has no attribute {name!r}")
