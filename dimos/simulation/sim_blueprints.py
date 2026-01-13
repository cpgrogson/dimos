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

"""
Blueprints for xArm manipulator control using component-based architecture.

This module provides declarative blueprints for configuring xArm with the new
generalized component-based driver architecture.

Usage:
    # Run via CLI:
    dimos run xarm-servo           # Driver only
    dimos run xarm-trajectory      # Driver + Joint trajectory controller
    dimos run xarm-cartesian       # Driver + Cartesian motion controller

    # Or programmatically:
    from dimos.hardware.manipulators.xarm.xarm_blueprints import xarm_trajectory
    coordinator = xarm_trajectory.build()
    coordinator.loop()
"""

from typing import Any

from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.manipulation.control import cartesian_motion_controller, joint_trajectory_controller
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import (  # type: ignore[attr-defined]
    JointCommand,
    JointState,
    RobotState,
)
from dimos.msgs.trajectory_msgs import JointTrajectory
from dimos.simulation.manipulators.mujoco_sim.xarmSimDriver import (
    xarm_sim_driver as xarm_driver_blueprint,
)


# Create a blueprint wrapper for the component-based driver
def xarm_driver(**config: Any) -> Any:
    """Create a blueprint for XArmSimDriver.

    Args:
        **config: Configuration parameters passed to XArmDriver
            - dof: Degrees of freedom - 5, 6, or 7 (default: 6)
            - has_gripper: Whether gripper is attached (default: False)
            - has_force_torque: Whether F/T sensor is attached (default: False)
            - control_rate: Control loop + joint feedback rate in Hz (default: 100)
            - monitor_rate: Robot state monitoring rate in Hz (default: 10)

    Returns:
        Blueprint configuration for XArmSimDriver
    """
    # Set defaults
    config.setdefault("dof", 6)
    config.setdefault("has_gripper", False)
    config.setdefault("has_force_torque", False)
    config.setdefault("control_rate", 100)
    config.setdefault("monitor_rate", 10)

    # Return the xarm_driver blueprint with the config
    return xarm_driver_blueprint(**config)


xarm7_trajectory_sim = autoconnect(
    xarm_driver(
        dof=7,  # XArm7
        has_gripper=False,
        has_force_torque=False,
        control_rate=100,
        monitor_rate=10,
    ),
    joint_trajectory_controller(
        control_frequency=100.0,
    ),
).transports(
    {
        ("joint_state", JointState): LCMTransport("/xarm/joint_states", JointState),
        ("robot_state", RobotState): LCMTransport("/xarm/robot_state", RobotState),
        ("joint_position_command", JointCommand): LCMTransport(
            "/xarm/joint_position_command", JointCommand
        ),
        ("trajectory", JointTrajectory): LCMTransport("/trajectory", JointTrajectory),
    }
)


__all__ = [
    "xarm7_trajectory_sim",
]

if __name__ == "__main__":
    xarm7_trajectory_sim.build().loop()
