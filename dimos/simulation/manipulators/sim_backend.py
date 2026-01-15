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

"""Robot-agnostic MuJoCo simulation backend."""

import logging
import math

from dimos.hardware.manipulators.spec import ControlMode, JointLimits, ManipulatorInfo
from dimos.simulation.manipulators import MujocoSimBackend


class SimBackend:
    """Backend wrapper for a generic MuJoCo simulation."""

    def __init__(
        self,
        robot: str,
        config_path: str | None = None,
        headless: bool = False,
        dof: int | None = None,
    ) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self._robot = robot
        self._config_path = config_path
        self._headless = headless
        self._native: MujocoSimBackend | None = None
        self._dof = int(dof) if dof is not None else 0
        self._connected = False
        self._servos_enabled = False
        self._control_mode = ControlMode.POSITION
        self._error_code = 0
        self._error_message = ""

    # =========================================================================
    # Connection
    # =========================================================================

    def connect(self) -> bool:
        """Connect to the MuJoCo simulation backend."""
        if not self._robot:
            raise ValueError("robot is required for MuJoCo simulation loading")
        try:
            self.logger.info("Connecting to MuJoCo Sim...")
            self._native = MujocoSimBackend(
                robot=self._robot,
                config_path=self._config_path,
                headless=self._headless,
            )
            self._native.connect()

            if self._native.connected:
                self._connected = True
                self._servos_enabled = True
                if self._dof <= 0:
                    self._dof = int(self._native.num_joints)
                self.logger.info("Successfully connected to MuJoCo Sim", extra={"dof": self._dof})
                return True

            self.logger.error("Failed to connect to MuJoCo Sim")
            return False
        except Exception as exc:
            self.logger.error(f"Sim connection failed: {exc}")
            return False

    def disconnect(self) -> None:
        """Disconnect from simulation."""
        if self._native:
            try:
                self._native.disconnect()
            finally:
                self._connected = False
                self._native = None

    def is_connected(self) -> bool:
        return bool(self._connected and self._native and self._native.connected)

    # =========================================================================
    # Info
    # =========================================================================

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(
            vendor="MuJoCo",
            model=self._robot,
            dof=self._dof,
            firmware_version=None,
            serial_number=None,
        )

    def get_dof(self) -> int:
        return self._dof

    def get_limits(self) -> JointLimits:
        if not self._native:
            lower = [-math.pi] * self._dof
            upper = [math.pi] * self._dof
            max_vel_rad = math.radians(180.0)
            return JointLimits(
                position_lower=lower,
                position_upper=upper,
                velocity_max=[max_vel_rad] * self._dof,
            )
        ranges = getattr(self._native.model, "jnt_range", None)
        if ranges is None or len(ranges) == 0:
            lower = [-math.pi] * self._dof
            upper = [math.pi] * self._dof
        else:
            limit = min(len(ranges), self._dof)
            lower = [float(ranges[i][0]) for i in range(limit)]
            upper = [float(ranges[i][1]) for i in range(limit)]
            if limit < self._dof:
                lower.extend([-math.pi] * (self._dof - limit))
                upper.extend([math.pi] * (self._dof - limit))
        max_vel_rad = math.radians(180.0)
        return JointLimits(
            position_lower=lower,
            position_upper=upper,
            velocity_max=[max_vel_rad] * self._dof,
        )

    # =========================================================================
    # Control Mode
    # =========================================================================

    def set_control_mode(self, mode: ControlMode) -> bool:
        self._control_mode = mode
        return True

    def get_control_mode(self) -> ControlMode:
        return self._control_mode

    # =========================================================================
    # State Reading
    # =========================================================================

    def read_joint_positions(self) -> list[float]:
        if self._native:
            return self._native.joint_positions[: self._dof]
        return [0.0] * self._dof

    def read_joint_velocities(self) -> list[float]:
        if self._native:
            return self._native.joint_velocities[: self._dof]
        return [0.0] * self._dof

    def read_joint_efforts(self) -> list[float]:
        if self._native:
            return self._native.joint_efforts[: self._dof]
        return [0.0] * self._dof

    def read_state(self) -> dict[str, int]:
        velocities = self.read_joint_velocities()
        is_moving = any(abs(v) > 1e-4 for v in velocities)
        mode_int = list(ControlMode).index(self._control_mode)
        return {
            "state": 1 if is_moving else 0,
            "mode": mode_int,
        }

    def read_error(self) -> tuple[int, str]:
        return self._error_code, self._error_message

    # =========================================================================
    # Motion Control (Joint Space)
    # =========================================================================

    def write_joint_positions(
        self,
        positions: list[float],
        velocity: float = 1.0,
    ) -> bool:
        _ = velocity
        if not self._servos_enabled or not self._native:
            return False
        self._control_mode = ControlMode.POSITION
        self._native.set_joint_position_targets(positions[: self._dof])
        return True

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        if not self._servos_enabled or not self._native:
            return False
        self._control_mode = ControlMode.VELOCITY
        dt = 1.0 / self._native.control_frequency
        current = self._native.joint_positions
        targets = [current[i] + velocities[i] * dt for i in range(min(len(velocities), self._dof))]
        self._native.set_joint_position_targets(targets)
        return True

    def write_stop(self) -> bool:
        if not self._native:
            return False
        self._native.hold_current_position()
        return True

    # =========================================================================
    # Servo Control
    # =========================================================================

    def write_enable(self, enable: bool) -> bool:
        self._servos_enabled = enable
        return True

    def read_enabled(self) -> bool:
        return self._servos_enabled

    def write_clear_errors(self) -> bool:
        self._error_code = 0
        self._error_message = ""
        return True

    # =========================================================================
    # Optional Interfaces
    # =========================================================================

    def read_cartesian_position(self) -> dict[str, float] | None:
        return None

    def write_cartesian_position(
        self,
        pose: dict[str, float],
        velocity: float = 1.0,
    ) -> bool:
        _ = pose
        _ = velocity
        return False

    def read_gripper_position(self) -> float | None:
        return None

    def write_gripper_position(self, position: float) -> bool:
        _ = position
        return False

    def read_force_torque(self) -> list[float] | None:
        return None


class SimSDKWrapper(SimBackend):
    """Backward-compatible alias for SimBackend."""


__all__ = [
    "SimBackend",
    "SimSDKWrapper",
]
