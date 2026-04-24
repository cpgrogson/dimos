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

"""Unitree G1 GR00T whole-body-control blueprint.

Runs the ControlCoordinator at 500 Hz with two tasks:

  - ``groot_wbc``  (priority 50) claims legs + waist (15 DOF) and runs
    the GR00T balance / walk ONNX policies at 50 Hz.
  - ``servo_arms`` (priority 10) claims the 14 arm joints and holds
    them at a configured relaxed pose.  No timeout — the task holds
    until an external caller sends new arm targets.

Velocity commands come in over LCM on ``/g1/cmd_vel`` as a Twist.  The
coordinator's ``twist_command`` dispatcher routes them into the task.

Architecture:
    Twist/cmd_vel ──▶ coordinator twist_command ──▶ GrootWBCTask
                                                   │
    ControlCoordinator ──joint_state──▶ LCM /coordinator/joint_state
                       ◀─joint_command── LCM /g1/joint_command
                              │
                    WholeBodyAdapter:
                      --simulate      → MujocoG1WholeBodyAdapter (in-process)
                      real hardware   → UnitreeG1LowLevelAdapter (DDS)

Usage:
    dimos run unitree-g1-groot-wbc --simulate          # MuJoCo in dimos, no DDS
    ROBOT_INTERFACE=enp86s0 dimos run unitree-g1-groot-wbc   # real robot

Environment:
    ROBOT_INTERFACE   DDS network interface for real robot (default "enp86s0").
                      Ignored under --simulate.
    DIMOS_DDS_DOMAIN  DDS domain id for real robot (default 0). Ignored
                      under --simulate.
    GROOT_MODEL_DIR   Directory containing balance.onnx + walk.onnx
                      (default "data/groot").
"""

from __future__ import annotations

import os

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_humanoid_joints,
)
from dimos.control.coordinator import TaskConfig, control_coordinator
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs import Twist
from dimos.msgs.sensor_msgs import JointState

_g1_joints = make_humanoid_joints("g1")
_g1_legs_waist = _g1_joints[:15]  # indices 0..14 — legs (12) + waist (3)
_g1_arms = _g1_joints[15:]  # indices 15..28 — left arm (7) + right arm (7)

# Per-joint PD gains, 29 entries in DDS motor order.  Lifted verbatim
# from g1-control-api/configs/g1_groot_wbc.yaml, which itself copies
# GR00T-WBC's g1_29dof_gear_wbc.yaml reference config.  These gains
# were the ones the balance / walk ONNX policies were trained against
# — diverging from them on real hardware risks instability.
_G1_GROOT_KP = [
    150.0,
    150.0,
    150.0,
    200.0,
    40.0,
    40.0,  # left leg
    150.0,
    150.0,
    150.0,
    200.0,
    40.0,
    40.0,  # right leg
    250.0,
    250.0,
    250.0,  # waist
    100.0,
    100.0,
    40.0,
    40.0,
    20.0,
    20.0,
    20.0,  # left arm
    100.0,
    100.0,
    40.0,
    40.0,
    20.0,
    20.0,
    20.0,  # right arm
]
_G1_GROOT_KD = [
    2.0,
    2.0,
    2.0,
    4.0,
    2.0,
    2.0,  # left leg
    2.0,
    2.0,
    2.0,
    4.0,
    2.0,
    2.0,  # right leg
    5.0,
    5.0,
    5.0,  # waist
    5.0,
    5.0,
    2.0,
    2.0,
    2.0,
    2.0,
    2.0,  # left arm
    5.0,
    5.0,
    2.0,
    2.0,
    2.0,
    2.0,
    2.0,  # right arm
]

# Relaxed arms-down pose.  Values taken from
# g1_control/backends/groot_wbc_backend.py:DEFAULT_29[15:] (all zeros),
# which is the zero-offset pose the policy was trained against.
# Operators can override at runtime by publishing joint targets on the
# arms via the joint_command transport.
_ARM_DEFAULT_POSE = [
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,  # left arm (7 DOF)
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,  # right arm (7 DOF)
]

_adapter_type = "mujoco_g1" if global_config.simulation else "unitree_g1"
_address = None if global_config.simulation else os.getenv("ROBOT_INTERFACE", "enp86s0")

unitree_g1_groot_wbc = control_coordinator(
    tick_rate=500.0,
    publish_joint_state=True,
    joint_state_frame_id="coordinator",
    hardware=[
        HardwareComponent(
            hardware_id="g1",
            hardware_type=HardwareType.WHOLE_BODY,
            joints=_g1_joints,
            adapter_type=_adapter_type,
            address=_address,
            domain_id=int(os.getenv("DIMOS_DDS_DOMAIN", "0")),
            auto_enable=True,
            kp=_G1_GROOT_KP,
            kd=_G1_GROOT_KD,
        ),
    ],
    tasks=[
        TaskConfig(
            name="groot_wbc",
            type="groot_wbc",
            joint_names=_g1_legs_waist,
            priority=50,
            model_path=os.getenv("GROOT_MODEL_DIR", "data/groot"),
            hardware_id="g1",
        ),
        TaskConfig(
            name="servo_arms",
            type="servo",
            joint_names=_g1_arms,
            priority=10,
            default_positions=_ARM_DEFAULT_POSE,
        ),
    ],
).transports(
    {
        ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
        ("joint_command", JointState): LCMTransport("/g1/joint_command", JointState),
        ("twist_command", Twist): LCMTransport("/g1/cmd_vel", Twist),
    }
)

__all__ = ["_ARM_DEFAULT_POSE", "unitree_g1_groot_wbc"]
