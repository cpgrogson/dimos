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

"""MuJoCo-native xArm simulation backend and module wrapper."""

from dataclasses import dataclass
import math
from typing import Any

from dimos.core import rpc
from dimos.hardware.manipulators.spec import JointLimits, ManipulatorInfo
from dimos.simulation.manipulators.sim_backend import SimBackend
from dimos.simulation.manipulators.sim_driver import SimDriver, SimDriverConfig


class XArmSimBackend(SimBackend):
    """Backend wrapper for xArm simulation using the MuJoCo simulation backend."""

    def __init__(
        self,
        robot: str,
        dof: int = 7,
        config_path: str | None = None,
        headless: bool = False,
    ) -> None:
        super().__init__(
            robot=robot,
            config_path=config_path,
            headless=headless,
            dof=dof,
        )

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(
            vendor="UFACTORY",
            model=f"xArm{self._dof}",
            dof=self._dof,
            firmware_version=None,
            serial_number=None,
        )

    def get_limits(self) -> JointLimits:
        if self._dof == 7:
            lower_deg = [-360, -118, -360, -233, -360, -97, -360]
            upper_deg = [360, 118, 360, 11, 360, 180, 360]
        elif self._dof == 6:
            lower_deg = [-360, -118, -225, -11, -360, -97]
            upper_deg = [360, 118, 11, 225, 360, 180]
        else:
            lower_deg = [-360, -118, -225, -97, -360]
            upper_deg = [360, 118, 11, 180, 360]

        lower_rad = [math.radians(d) for d in lower_deg[: self._dof]]
        upper_rad = [math.radians(d) for d in upper_deg[: self._dof]]
        max_vel_rad = math.radians(180.0)
        return JointLimits(
            position_lower=lower_rad,
            position_upper=upper_rad,
            velocity_max=[max_vel_rad] * self._dof,
        )


class XArmSimSDKWrapper(XArmSimBackend):
    """Backward-compatible alias for XArmSimBackend."""


@dataclass
class XArmSimDriverConfig(SimDriverConfig):
    pass


class XArmSimDriver(SimDriver):
    """xArm simulation driver module using MuJoCo backend."""

    default_config = XArmSimDriverConfig
    config: XArmSimDriverConfig

    def _create_backend(self) -> SimBackend:
        return XArmSimBackend(
            robot=self.config.robot or "",
            config_path=self.config.config_path,
            headless=self.config.headless,
        )

    @rpc
    def start(self) -> None:
        if not self.config.robot:
            raise ValueError("robot is required for XArmSimDriver")
        super().start()


def get_blueprint() -> dict[str, Any]:
    return {
        "name": "XArmSimDriver",
        "class": XArmSimDriver,
        "config": {
            "robot": None,
            "config_path": None,
            "headless": False,
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
    "XArmSimBackend",
    "XArmSimDriver",
    "XArmSimDriverConfig",
    "XArmSimSDKWrapper",
    "get_blueprint",
    "xarm_sim_driver",
]
