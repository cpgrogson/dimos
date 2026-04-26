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

"""Unitree G1 GR00T whole-body-control blueprint — real hardware.

Runs the ControlCoordinator at 500 Hz with two tasks:

  - ``groot_wbc``  (priority 50) claims legs + waist (15 DOF) and runs
    the GR00T balance / walk ONNX policies at 50 Hz.
  - ``servo_arms`` (priority 10) claims the 14 arm joints and holds
    them at the relaxed pose the policy was trained against.

Real-hardware safety profile: the blueprint comes up unarmed and in
dry-run.  The operator opens the dashboard at http://localhost:7779/,
verifies the computed commands look sane, then clicks Activate to
ramp from the current pose to the bent-knee default over 10 s before
handing torque control to the policy.

Architecture:
    dashboard WASD ──▶ WebsocketVisModule ──▶ LCM /g1/cmd_vel
                                                       │
                              coordinator twist_command ──▶ GrootWBCTask
                                                       │
    ControlCoordinator ──joint_state──▶ LCM /coordinator/joint_state
                       ◀─joint_command── LCM /g1/joint_command

Sim is a separate blueprint (``unitree-g1-groot-wbc-sim``) so each
file is statically clear about what it composes — no module-level
config branching.

Usage:
    ROBOT_INTERFACE=enp86s0 dimos run unitree-g1-groot-wbc

Environment:
    ROBOT_INTERFACE   DDS network interface for the real robot
                      (default ``"enp86s0"``).
    DIMOS_DDS_DOMAIN  DDS domain id (default ``0``).
    CYCLONEDDS_HOME   Required at runtime — must point at the
                      cyclonedds C install (e.g. ``~/cyclonedds/install``).
    GROOT_MODEL_DIR   Directory containing ``balance.onnx`` +
                      ``walk.onnx`` (default: pulled via
                      ``get_data("groot")``).
"""

from __future__ import annotations

import os

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Bool import Bool as DimosBool
from dimos.robot.unitree.g1.blueprints.basic._groot_wbc_common import (
    ARM_DEFAULT_POSE,
    G1_GROOT_KD,
    G1_GROOT_KP,
    g1_arms,
    g1_joints,
    g1_legs_waist,
)
from dimos.utils.data import get_data
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

_g1_coordinator = ControlCoordinator.blueprint(
    tick_rate=500.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="g1",
            hardware_type=HardwareType.WHOLE_BODY,
            joints=g1_joints,
            adapter_type="unitree_g1",
            address=os.getenv("ROBOT_INTERFACE", "enp86s0"),
            domain_id=int(os.getenv("DIMOS_DDS_DOMAIN", "0")),
            auto_enable=True,
            kp=G1_GROOT_KP,
            kd=G1_GROOT_KD,
        ),
    ],
    tasks=[
        TaskConfig(
            name="groot_wbc",
            type="groot_wbc",
            joint_names=g1_legs_waist,
            priority=50,
            model_path=os.getenv("GROOT_MODEL_DIR", str(get_data("groot"))),
            hardware_id="g1",
            auto_start=True,
            # Real-hw safety: come up unarmed + dry-run.  Operator
            # arms via the dashboard Activate button after sanity
            # checks; activation ramps over 10 s.
            auto_arm=False,
            auto_dry_run=True,
            default_ramp_seconds=10.0,
        ),
        TaskConfig(
            name="servo_arms",
            type="servo",
            joint_names=g1_arms,
            priority=10,
            default_positions=ARM_DEFAULT_POSE,
            auto_start=True,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("odom", PoseStamped): LCMTransport("/odom", PoseStamped),
        ("joint_command", JointState): LCMTransport("/g1/joint_command", JointState),
        ("twist_command", Twist): LCMTransport("/g1/cmd_vel", Twist),
        ("activate", DimosBool): LCMTransport("/g1/activate", DimosBool),
        ("dry_run", DimosBool): LCMTransport("/g1/dry_run", DimosBool),
    }
)

# Operator dashboard (WASD, Activate, dry-run toggle) at
# http://localhost:7779/.  WebsocketVisModule re-publishes the
# dashboard's events onto the coordinator's LCM ports.
_g1_ws_vis = WebsocketVisModule.blueprint().transports(
    {
        ("cmd_vel", Twist): LCMTransport("/g1/cmd_vel", Twist),
        ("activate", DimosBool): LCMTransport("/g1/activate", DimosBool),
        ("dry_run", DimosBool): LCMTransport("/g1/dry_run", DimosBool),
    },
)

unitree_g1_groot_wbc = autoconnect(_g1_coordinator, _g1_ws_vis)

__all__ = ["unitree_g1_groot_wbc"]
