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

"""MuJoCo-native xArm simulation driver and SDK wrapper."""

import logging
import math
from typing import Any

from dimos.hardware.manipulators.base import (
    BaseManipulatorDriver,
    StandardMotionComponent,
    StandardServoComponent,
    StandardStatusComponent,
)
from dimos.hardware.manipulators.base.sdk_interface import BaseManipulatorSDK, ManipulatorInfo
from dimos.simulation.manipulators.mujoco_sim.xarm_sim_bridge import XArmSimBridge

logger = logging.getLogger(__name__)


def _ok(result: Any) -> bool:
    if isinstance(result, tuple):
        return result[0] == 0
    return result == 0


class XArmSimSDKWrapper(BaseManipulatorSDK):
    """SDK wrapper for xArm simulation using the MuJoCo bridge."""

    def __init__(self) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.native_sdk: Any = None
        self.dof = 7
        self._connected = False

    # ============= Connection Management =============

    def connect(self, config: dict[str, Any]) -> bool:
        """Connect to the MuJoCo xArm simulation backend."""
        try:
            self.dof = config.get("dof", 7)
            control_rate = config.get("control_rate", 100)
            report_type = config.get("report_type", "dev")
            joint_state_rate = config.get("joint_state_rate", control_rate)

            self.logger.info(f"Connecting to XArm Sim (DOF: {self.dof})...")
            self.native_sdk = XArmSimBridge(
                is_radian=False,
                check_joint_limit=True,
                num_joints=self.dof,
                report_type=str(report_type),
                joint_state_rate=float(joint_state_rate),
                control_frequency=float(control_rate),
            )
            self.native_sdk.connect()

            if self.native_sdk.connected:
                self.native_sdk.motion_enable(True)
                self.native_sdk.set_mode(1)
                self.native_sdk.set_state(0)
                self._connected = True
                self.logger.info(
                    f"Successfully connected to XArm Sim (version: {self.native_sdk.version})"
                )
                return True

            self.logger.error("Failed to connect to XArm Sim")
            return False
        except Exception as exc:
            self.logger.error(f"Sim connection failed: {exc}")
            return False

    def disconnect(self) -> None:
        """Disconnect from simulation."""
        if self.native_sdk:
            try:
                self.native_sdk.disconnect()
            finally:
                self._connected = False
                self.native_sdk = None

    def is_connected(self) -> bool:
        return bool(self._connected and self.native_sdk and self.native_sdk.connected)

    # ============= Joint State Query =============

    def get_joint_positions(self) -> list[float]:
        code, angles = self.native_sdk.get_servo_angle()
        if code != 0:
            raise RuntimeError(f"XArm Sim error getting positions: {code}")
        return [math.radians(angle) for angle in angles[: self.dof]]

    def get_joint_velocities(self) -> list[float]:
        if hasattr(self.native_sdk, "get_joint_speeds"):
            code, speeds = self.native_sdk.get_joint_speeds()
            if code == 0:
                return [math.radians(speed) for speed in speeds[: self.dof]]
        return [0.0] * self.dof

    def get_joint_efforts(self) -> list[float]:
        if hasattr(self.native_sdk, "get_joint_torques"):
            code, torques = self.native_sdk.get_joint_torques()
            if code == 0:
                return list(torques[: self.dof])
        return [0.0] * self.dof

    # ============= Joint Motion Control =============

    def set_joint_positions(
        self,
        positions: list[float],
        _velocity: float = 1.0,
        _acceleration: float = 1.0,
        _wait: bool = False,
    ) -> bool:
        degrees = [math.degrees(pos) for pos in positions]
        code = self.native_sdk.set_servo_angle_j(degrees, speed=100, mvacc=500, wait=False)
        return _ok(code)

    def set_joint_velocities(self, velocities: list[float]) -> bool:
        if not hasattr(self.native_sdk, "vc_set_joint_velocity"):
            self.logger.warning("Velocity control not supported in this XArm Sim version")
            return False

        deg_velocities = [math.degrees(vel) for vel in velocities]
        if self.native_sdk.mode != 4:
            self.native_sdk.set_mode(4)
        code = self.native_sdk.vc_set_joint_velocity(deg_velocities)
        return _ok(code)

    def set_joint_efforts(self, efforts: list[float]) -> bool:
        self.logger.warning("Torque control not supported in XArm Sim bridge")
        _ = efforts
        return False

    def stop_motion(self) -> bool:
        code = self.native_sdk.emergency_stop()
        if _ok(code):
            self.native_sdk.set_state(0)
            self.native_sdk.motion_enable(True)
        return _ok(code)

    # ============= Servo Control =============

    def enable_servos(self) -> bool:
        code1 = self.native_sdk.motion_enable(True)
        code2 = self.native_sdk.set_state(0)
        code3 = self.native_sdk.set_mode(1)
        return _ok(code1) and _ok(code2) and _ok(code3)

    def disable_servos(self) -> bool:
        code = self.native_sdk.motion_enable(False)
        return _ok(code)

    def are_servos_enabled(self) -> bool:
        return bool(self.native_sdk.mode == 1 and self.native_sdk.mode != 4)

    # ============= System State =============

    def get_robot_state(self) -> dict[str, Any]:
        return {
            "state": self.native_sdk.state,
            "mode": self.native_sdk.mode,
            "error_code": self.native_sdk.error_code,
            "warn_code": self.native_sdk.warn_code,
            "is_moving": self.native_sdk.state == 3,
            "cmd_num": self.native_sdk.cmd_num,
        }

    def get_error_code(self) -> int:
        return int(self.native_sdk.error_code)

    def get_error_message(self) -> str:
        if self.native_sdk.error_code == 0:
            return ""
        return f"Simulated error {self.native_sdk.error_code}"

    def clear_errors(self) -> bool:
        code = self.native_sdk.clean_error()
        if _ok(code):
            self.native_sdk.set_state(0)
        return _ok(code)

    def emergency_stop(self) -> bool:
        code = self.native_sdk.emergency_stop()
        return _ok(code)

    # ============= Information =============

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(
            vendor="UFACTORY",
            model=f"xArm{self.dof}",
            dof=self.dof,
            firmware_version=self.native_sdk.version if self.native_sdk else None,
            serial_number=self.native_sdk.get_servo_version()[1][0] if self.native_sdk else None,
        )

    def get_joint_limits(self) -> tuple[list[float], list[float]]:
        if self.dof == 7:
            lower_deg = [-360, -118, -360, -233, -360, -97, -360]
            upper_deg = [360, 118, 360, 11, 360, 180, 360]
        elif self.dof == 6:
            lower_deg = [-360, -118, -225, -11, -360, -97]
            upper_deg = [360, 118, 11, 225, 360, 180]
        else:
            lower_deg = [-360, -118, -225, -97, -360]
            upper_deg = [360, 118, 11, 180, 360]

        lower_rad = [math.radians(d) for d in lower_deg[: self.dof]]
        upper_rad = [math.radians(d) for d in upper_deg[: self.dof]]
        return (lower_rad, upper_rad)

    def get_velocity_limits(self) -> list[float]:
        max_vel_rad = math.radians(180.0)
        return [max_vel_rad] * self.dof

    def get_acceleration_limits(self) -> list[float]:
        max_acc_rad = math.radians(1145.0)
        return [max_acc_rad] * self.dof

    # ============= Optional Methods =============

    def get_cartesian_position(self) -> dict[str, float] | None:
        code, pose = self.native_sdk.get_position(is_radian=True)
        if code != 0:
            return None
        return {
            "x": float(pose[0]),
            "y": float(pose[1]),
            "z": float(pose[2]),
            "roll": float(pose[3]),
            "pitch": float(pose[4]),
            "yaw": float(pose[5]),
        }

    def set_cartesian_position(
        self,
        pose: dict[str, float],
        velocity: float = 1.0,
        acceleration: float = 1.0,
        wait: bool = False,
    ) -> bool:
        _ = velocity
        _ = acceleration
        target = [
            pose["x"],
            pose["y"],
            pose["z"],
            pose["roll"],
            pose["pitch"],
            pose["yaw"],
        ]
        code = self.native_sdk.set_position(*target, is_radian=True, wait=wait)
        return _ok(code)

    def get_force_torque(self) -> list[float] | None:
        if hasattr(self.native_sdk, "get_ft_sensor_data"):
            code, ft_data = self.native_sdk.get_ft_sensor_data()
            if code == 0:
                return list(ft_data)
        return None


class XArmSimDriver(BaseManipulatorDriver):
    """xArm driver backed by the MuJoCo simulation bridge."""

    def __init__(self, **kwargs: Any) -> None:
        config: dict[str, Any] = kwargs.pop("config", {})

        driver_params = [
            "dof",
            "has_gripper",
            "has_force_torque",
            "control_rate",
            "monitor_rate",
            "report_type",
            "joint_state_rate",
        ]
        for param in driver_params:
            if param in kwargs:
                config[param] = kwargs.pop(param)

        logger.info(f"Initializing XArmSimDriver with config: {config}")

        sdk = XArmSimSDKWrapper()
        sdk.dof = int(config.get("dof", sdk.dof))
        components = [
            StandardMotionComponent(sdk),
            StandardServoComponent(sdk),
            StandardStatusComponent(sdk),
        ]

        kwargs.pop("sdk", None)
        kwargs.pop("components", None)
        kwargs.pop("name", None)

        super().__init__(
            sdk=sdk,
            components=components,
            config=config,
            name="XArmSimDriver",
            **kwargs,
        )

        logger.info("XArmSimDriver initialized successfully")


def get_blueprint() -> dict[str, Any]:
    return {
        "name": "XArmSimDriver",
        "class": XArmSimDriver,
        "config": {
            "dof": 7,
            "has_gripper": False,
            "has_force_torque": False,
            "control_rate": 100,
            "monitor_rate": 10,
            "report_type": "dev",
            "joint_state_rate": 100,
        },
        "inputs": {
            "joint_position_command": "JointCommand",
            "joint_velocity_command": "JointCommand",
        },
        "outputs": {
            "joint_state": "JointState",
            "robot_state": "RobotState",
        },
    }


xarm_sim_driver = XArmSimDriver.blueprint

__all__ = [
    "XArmSimDriver",
    "XArmSimSDKWrapper",
    "get_blueprint",
    "xarm_sim_driver",
]
