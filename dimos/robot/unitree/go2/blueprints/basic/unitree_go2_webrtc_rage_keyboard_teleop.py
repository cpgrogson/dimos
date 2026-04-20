#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
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

# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Unitree Go2 keyboard teleop via WebRTC with Rage Mode enabled.

Same topology as unitree-go2-webrtc-keyboard-teleop (GO2Connection +
ControlCoordinator with transport_lcm adapter), but GO2Connection is
configured with rage_mode=True so FsmRageMode is toggled on after
BalanceStand at connection start. Velocity commands flow through the
existing WIRELESS_CONTROLLER joystick channel (driven by Twist → move()),
which Rage's policy consumes directly — no separate publisher thread
needed because the WebRTC connection already owns that channel.

Use this when you want Rage's ~2.5 m/s envelope but don't want to run
the direct-DDS adapter (e.g. because you're driving from a laptop that
isn't on the Go2's DDS LAN, or because onboard NUC power constraints
rule out the direct path). See data/notes/go2_firmware_modes.md.

Usage:
    dimos run unitree-go2-webrtc-rage-keyboard-teleop
"""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

_go2_joints = make_twist_base_joints("go2")

unitree_go2_webrtc_rage_keyboard_teleop = (
    autoconnect(
        GO2Connection.blueprint(rage_mode=True),
        ControlCoordinator.blueprint(
            hardware=[
                HardwareComponent(
                    hardware_id="go2",
                    hardware_type=HardwareType.BASE,
                    joints=_go2_joints,
                    adapter_type="transport_lcm",
                ),
            ],
            tasks=[
                TaskConfig(
                    name="vel_go2",
                    type="velocity",
                    joint_names=_go2_joints,
                    priority=10,
                ),
            ],
        ),
        KeyboardTeleop.blueprint(),
    )
    .remappings(
        [
            (GO2Connection, "cmd_vel", "go2_cmd_vel"),
            (GO2Connection, "odom", "go2_odom"),
        ]
    )
    .transports(
        {
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
            ("go2_cmd_vel", Twist): LCMTransport("/go2/cmd_vel", Twist),
            ("go2_odom", PoseStamped): LCMTransport("/go2/odom", PoseStamped),
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        }
    )
    .global_config(obstacle_avoidance=False)
)

__all__ = ["unitree_go2_webrtc_rage_keyboard_teleop"]
