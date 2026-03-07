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

"""Unitree G1 trajectory playback blueprint.

Composes the ControlCoordinator (500 Hz, 29-DOF whole-body adapter) with the
G1Playback module that replays a recorded JSON trajectory.

Architecture:
    G1Playback  --joint_command-->  ControlCoordinator
                 <--joint_state--       | DDS (500 Hz)
                                  UnitreeG1LowLevelAdapter

Usage:
    TRAJECTORY_FILE=macarena.json ROBOT_INTERFACE=enp60s0 dimos run unitree-g1-playback
"""

from __future__ import annotations

import os

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_humanoid_joints,
)
from dimos.control.coordinator import TaskConfig, control_coordinator
from dimos.control.examples.g1_playback import g1_playback
from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs import JointState

_g1_joints = make_humanoid_joints("g1")

_coordinator = control_coordinator(
    tick_rate=500.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="g1",
            hardware_type=HardwareType.WHOLE_BODY,
            joints=_g1_joints,
            adapter_type="unitree_g1",
            address=os.getenv("ROBOT_INTERFACE", "enp86s0"),
            auto_enable=True,
        ),
    ],
    tasks=[
        TaskConfig(
            name="servo_g1",
            type="servo",
            joint_names=_g1_joints,
            priority=10,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("joint_command", JointState): LCMTransport("/g1/joint_command", JointState),
    }
)

_playback = g1_playback().transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("joint_command", JointState): LCMTransport("/g1/joint_command", JointState),
    }
)

unitree_g1_playback = autoconnect(_coordinator, _playback)

__all__ = ["unitree_g1_playback"]
