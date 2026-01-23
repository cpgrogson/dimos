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

"""SDK2 Bridge Controller for MuJoCo simulation.

Bridges unitree_sdk2py DDS messages to MuJoCo actuators, enabling policies
to deploy to simulation or real Unitree robots with zero code changes.

Uses unitree_sdk2py's native ChannelFactory for full SDK2 compatibility.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import TYPE_CHECKING, Any

import mujoco

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelPublisher,
    ChannelSubscriber,
)

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    pass

logger = setup_logger()

# Topic names (Unitree SDK2 convention)
TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_SPORTMODESTATE = "rt/sportmodestate"

# Sensor layout constants
MOTOR_SENSOR_NUM = 3  # position, velocity, torque per motor


@dataclass
class SDK2BridgeConfig:
    """Configuration for SDK2 bridge."""

    domain_id: int = 1  # Unitree convention: 1 for sim, 0 for real
    interface: str = "lo0"  # "lo0" for macOS sim, "lo" for Linux, network interface for real
    robot_type: str = "go2"  # "go2", "g1", etc.


def _get_idl_types(robot_type: str) -> tuple[type, type, type, Any, Any]:
    """Import IDL types based on robot type.

    Returns (LowCmd_, LowState_, SportModeState_, LowState_default_factory, SportModeState_default_factory)
    """
    # Lazy import to avoid requiring unitree_sdk2py when not using SDK2 mode
    if robot_type in ("g1", "h1_2"):
        # Humanoid robots use unitree_hg IDL
        from unitree_sdk2py.idl.default import (
            unitree_hg_msg_dds__LowState_ as LowState_default,
        )
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

        # G1/H1-2 use same SportModeState from unitree_go
        from unitree_sdk2py.idl.default import (
            unitree_go_msg_dds__SportModeState_ as SportModeState_default,
        )
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    else:
        # Quadruped robots (Go2, B2, H1, etc.) use unitree_go IDL
        from unitree_sdk2py.idl.default import (
            unitree_go_msg_dds__LowState_ as LowState_default,
        )
        from unitree_sdk2py.idl.default import (
            unitree_go_msg_dds__SportModeState_ as SportModeState_default,
        )
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import (
            LowCmd_,
            LowState_,
            SportModeState_,
        )

    return LowCmd_, LowState_, SportModeState_, LowState_default, SportModeState_default


class SDK2BridgeController:
    """Bridges SDK2 DDS messages to MuJoCo actuators using unitree_sdk2py's native DDS.

    Subscribes to rt/lowcmd and applies PD control to MuJoCo actuators.
    Publishes rt/lowstate and rt/sportmodestate with sensor feedback.
    """

    def __init__(
        self,
        mj_model: mujoco.MjModel,
        mj_data: mujoco.MjData,
        config: SDK2BridgeConfig,
    ) -> None:
        self.mj_model = mj_model
        self.mj_data = mj_data
        self.config = config
        self.num_motors = mj_model.nu

        # Get IDL types for this robot
        (
            self.LowCmd_,
            self.LowState_,
            self.SportModeState_,
            LowState_default,
            SportModeState_default,
        ) = _get_idl_types(config.robot_type)

        # Initialize state message instances
        self._low_state = LowState_default()
        self._sport_state = SportModeState_default()

        # Build sensor index maps (find sensors by name, not fixed index)
        self._build_sensor_maps()

        # Initialize unitree_sdk2py's native DDS channel factory
        ChannelFactoryInitialize(config.domain_id, config.interface)

        # Publishers for state feedback
        self._state_pub = ChannelPublisher(TOPIC_LOWSTATE, self.LowState_)
        self._state_pub.Init()

        self._sport_pub = ChannelPublisher(TOPIC_SPORTMODESTATE, self.SportModeState_)
        self._sport_pub.Init()

        # Subscriber for motor commands
        self._cmd_sub = ChannelSubscriber(TOPIC_LOWCMD, self.LowCmd_)
        self._cmd_sub.Init(self._lowcmd_callback, 10)

        # Lock for thread-safe access to mj_data
        self._data_lock = threading.Lock()

        logger.info(
            "SDK2 bridge initialized",
            domain_id=config.domain_id,
            interface=config.interface,
            robot_type=config.robot_type,
            num_motors=self.num_motors,
        )

        # Verify keyframe was applied (robot should be at standing pose)
        import numpy as np

        # Debug: show sensor indices and raw data
        logger.info(
            "SDK2 sensor mapping debug",
            joint_pos_idx_0_5=[self._joint_pos_idx.get(i, -1) for i in range(6)],
            sensordata_0_5=self.mj_data.sensordata[0:6].tolist(),
            qpos_7_13=self.mj_data.qpos[7:13].tolist(),
        )

        initial_qpos = np.array([self._get_joint_pos(i) for i in range(self.num_motors)])
        expected = [-0.312, 0, 0, 0.669, -0.363, 0]  # Left leg standing pose
        actual = initial_qpos[:6]
        match = np.allclose(actual, expected, atol=0.01)
        if match:
            logger.info("Initial robot pose: OK (keyframe applied)")
        else:
            logger.warning(
                "Initial pose mismatch - keyframe may not be applied",
                actual=actual.tolist(),
                expected=expected,
            )

        # Store initial joint positions for "hold position" control until commands arrive
        self._initial_joint_pos = initial_qpos.copy()
        self._commands_received = False

        # Latest commanded targets from rt/lowcmd (motor order).
        # NOTE: On the real robot, the low-level motor controller evaluates PD at a high rate
        # using (q, dq, kp, kd, tau). To emulate that with MuJoCo torque actuators we must
        # recompute torques every physics step, not just when a DDS message arrives.
        self._cmd_q = np.zeros(self.num_motors, dtype=np.float32)
        self._cmd_dq = np.zeros(self.num_motors, dtype=np.float32)
        self._cmd_kp = np.zeros(self.num_motors, dtype=np.float32)
        self._cmd_kd = np.zeros(self.num_motors, dtype=np.float32)
        self._cmd_tau = np.zeros(self.num_motors, dtype=np.float32)

        # When MuJoCo is reset (viewer backspace), the simulator state teleports but the external
        # policy runner keeps publishing mid-gait targets. Ignore commands briefly after a reset
        # and fall back to hold-position PD so the policy can re-stabilize.
        self._ignore_commands_until: float = 0.0

        # Default PD gains for holding position
        # IMPORTANT: These should match the policy's trained gains to avoid stiffness discontinuity
        # when switching from hold-position to policy control.
        # Values from MJLab model: hip_pitch=40.18, hip_roll=99.1, knee=99.1, ankle=28.5
        self._default_kp = np.array([
            40.0, 99.0, 40.0, 99.0, 28.5, 28.5,    # Left leg (matching MJLab gains)
            40.0, 99.0, 40.0, 99.0, 28.5, 28.5,    # Right leg
            40.0, 28.5, 28.5,                       # Waist (yaw=40, roll/pitch=28.5)
            14.0, 14.0, 14.0, 14.0, 14.0, 17.0, 17.0,  # Left arm (from MJLab)
            14.0, 14.0, 14.0, 14.0, 14.0, 17.0, 17.0,  # Right arm
        ], dtype=np.float32)
        self._default_kd = np.array([
            2.5, 6.3, 2.5, 6.3, 1.8, 1.8,   # Left leg (matching MJLab damping)
            2.5, 6.3, 2.5, 6.3, 1.8, 1.8,   # Right leg
            2.5, 1.8, 1.8,                   # Waist
            0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1,  # Left arm
            0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1,  # Right arm
        ], dtype=np.float32)

        # Gain scaling factor for torque-controlled actuators
        # IMPORTANT: The policy was TRAINED with specific kp/kd gains (e.g., kp=40, kd=2.5).
        # Using different gains changes the dynamics and causes instability.
        # We MUST use the same gains the policy was trained with (no scaling).
        self._kp_scale = 1.0
        self._kd_scale = 1.0

        # Apply initial hold-position control
        self._apply_hold_position_control()
        logger.info(
            "SDK2 bridge: holding initial position until commands arrive",
            kp_scale=self._kp_scale,
            kd_scale=self._kd_scale,
        )

    def _build_sensor_maps(self) -> None:
        """Build sensor index maps by looking up sensors by name.

        This allows the bridge to work with any MuJoCo model that has
        appropriately named sensors, rather than requiring a specific layout.
        """
        # Map joint names to their sensor indices
        self._joint_pos_idx: dict[int, int] = {}  # motor_idx -> sensordata_idx
        self._joint_vel_idx: dict[int, int] = {}
        self._joint_torque_idx: dict[int, int] = {}

        # Get actuator/joint names and find corresponding sensors
        for i in range(self.num_motors):
            actuator_name = mujoco.mj_id2name(
                self.mj_model, mujoco._enums.mjtObj.mjOBJ_ACTUATOR, i
            )
            if actuator_name is None:
                continue

            # Try common sensor naming patterns
            base_name = actuator_name.replace("_joint", "")
            pos_names = [f"{base_name}_pos", f"{actuator_name}_pos", f"{base_name}_joint_pos"]
            vel_names = [f"{base_name}_vel", f"{actuator_name}_vel", f"{base_name}_joint_vel"]
            torque_names = [f"{base_name}_torque", f"{actuator_name}_torque"]

            self._joint_pos_idx[i] = self._find_sensor(pos_names)
            self._joint_vel_idx[i] = self._find_sensor(vel_names)
            self._joint_torque_idx[i] = self._find_sensor(torque_names)

        # IMU sensors
        self._imu_quat_idx = self._find_sensor(["imu_quat", "orientation"])
        self._imu_gyro_idx = self._find_sensor(["imu_gyro", "gyro"])
        self._imu_accel_idx = self._find_sensor(["imu_acc", "accelerometer"])

        # Frame sensors for odometry
        self._frame_pos_idx = self._find_sensor(["frame_pos", "position"])
        self._frame_vel_idx = self._find_sensor(["frame_vel", "global_linvel"])

        logger.debug(
            "SDK2 bridge sensor maps built",
            joint_pos_found=sum(1 for v in self._joint_pos_idx.values() if v >= 0),
            joint_vel_found=sum(1 for v in self._joint_vel_idx.values() if v >= 0),
            has_imu=self._imu_quat_idx >= 0,
            has_frame=self._frame_pos_idx >= 0,
        )

    def _find_sensor(self, names: list[str]) -> int:
        """Find sensor index by trying multiple possible names. Returns -1 if not found."""
        for name in names:
            sensor_id = mujoco.mj_name2id(
                self.mj_model, mujoco._enums.mjtObj.mjOBJ_SENSOR, name
            )
            if sensor_id >= 0:
                # Get the starting index in sensordata for this sensor
                return int(self.mj_model.sensor_adr[sensor_id])
        return -1

    def _get_joint_pos(self, motor_idx: int) -> float:
        """Get joint position for a motor, using sensor or qpos fallback."""
        sensor_idx = self._joint_pos_idx.get(motor_idx, -1)
        if sensor_idx >= 0:
            return float(self.mj_data.sensordata[sensor_idx])
        # Fallback: use qpos (skip first 7 values for freejoint)
        return float(self.mj_data.qpos[7 + motor_idx])

    def _get_joint_vel(self, motor_idx: int) -> float:
        """Get joint velocity for a motor, using sensor or qvel fallback."""
        sensor_idx = self._joint_vel_idx.get(motor_idx, -1)
        if sensor_idx >= 0:
            return float(self.mj_data.sensordata[sensor_idx])
        # Fallback: use qvel (skip first 6 values for freejoint)
        return float(self.mj_data.qvel[6 + motor_idx])

    def _get_joint_torque(self, motor_idx: int) -> float:
        """Get joint torque for a motor, using sensor or actuator_force fallback."""
        sensor_idx = self._joint_torque_idx.get(motor_idx, -1)
        if sensor_idx >= 0:
            return float(self.mj_data.sensordata[sensor_idx])
        # Fallback: use actuator force
        return float(self.mj_data.actuator_force[motor_idx])

    def _apply_hold_position_control(self) -> None:
        """Apply PD control to hold the robot at initial position.

        This is used before SDK2PolicyRunner sends commands to prevent the robot from falling.
        """
        for i in range(self.num_motors):
            q_actual = self._get_joint_pos(i)
            dq_actual = self._get_joint_vel(i)
            q_target = self._initial_joint_pos[i]

            # PD control: ctrl = kp*(q_target - q) - kd*dq
            self.mj_data.ctrl[i] = (
                self._default_kp[i] * (q_target - q_actual)
                - self._default_kd[i] * dq_actual
            )

    def _lowcmd_callback(self, msg: Any) -> None:
        """Receive SDK2 LowCmd and cache targets for per-step PD evaluation.

        IMPORTANT: We intentionally do NOT compute torques in this DDS callback.
        DDS callbacks run at the policy/control rate (e.g. 50Hz), but MuJoCo steps
        faster (e.g. 500Hz). Computing torques only on message arrival would hold
        a stale torque for ~20ms, which is *not* equivalent to the robot's internal
        PD loop and can destabilize the policy.
        """
        with self._data_lock:
            # If MuJoCo was just reset, drop commands briefly to avoid applying mid-gait targets
            # to a freshly reset (standing) robot state.
            if self._ignore_commands_until > time.time():
                return

            # Mark that we've received commands - stop holding position
            if not self._commands_received:
                self._commands_received = True
                logger.info("SDK2 bridge: first command received, switching to policy control")

            # Debug: log first few commands
            if not hasattr(self, "_cmd_count"):
                self._cmd_count = 0
            self._cmd_count += 1

            if self._cmd_count <= 3:
                logger.info(
                    f"SDK2 lowcmd received #{self._cmd_count}",
                    q_target_0_5=[msg.motor_cmd[i].q for i in range(6)],
                    kp_policy_0_5=[msg.motor_cmd[i].kp for i in range(6)],
                    kp_scaled_0_5=[msg.motor_cmd[i].kp * self._kp_scale for i in range(6)],
                    q_actual_0_5=[self._get_joint_pos(i) for i in range(6)],
                )

            # Cache last command targets (motor order).
            for i in range(self.num_motors):
                self._cmd_q[i] = float(msg.motor_cmd[i].q)
                self._cmd_dq[i] = float(msg.motor_cmd[i].dq)
                self._cmd_kp[i] = float(msg.motor_cmd[i].kp)
                self._cmd_kd[i] = float(msg.motor_cmd[i].kd)
                self._cmd_tau[i] = float(msg.motor_cmd[i].tau)

    def on_mujoco_reset(self, *, grace_period_s: float = 0.2) -> None:
        """Handle MuJoCo viewer reset (e.g., backspace).

        MuJoCo resets `data` state, but SDK2 runs out-of-process. Without this hook, the bridge
        would immediately apply stale cached targets (mid-gait) to a freshly reset pose, causing
        large transients and falls.
        """
        with self._data_lock:
            self._commands_received = False

            # Re-anchor hold-position target to the newly reset pose.
            import numpy as np

            self._initial_joint_pos = np.array(
                [self._get_joint_pos(i) for i in range(self.num_motors)],
                dtype=np.float32,
            )

            # Drop cached targets so no stale PD is applied.
            self._cmd_q.fill(0.0)
            self._cmd_dq.fill(0.0)
            self._cmd_kp.fill(0.0)
            self._cmd_kd.fill(0.0)
            self._cmd_tau.fill(0.0)

            self._ignore_commands_until = time.time() + float(grace_period_s)

        logger.info(
            "SDK2 bridge: MuJoCo reset detected; returning to hold-position mode",
            grace_period_s=float(grace_period_s),
        )

    def _apply_policy_pd_control(self) -> None:
        """Apply per-step PD control using the latest cached LowCmd targets.

        Control equation (per motor):
            ctrl = tau + kp*(q_des - q) + kd*(dq_des - dq)
        """
        for i in range(self.num_motors):
            q_actual = self._get_joint_pos(i)
            dq_actual = self._get_joint_vel(i)

            # Keep gain scaling hooks (default 1.0). In practice, we want to match the
            # policy's trained gains for stability.
            kp = float(self._cmd_kp[i]) * self._kp_scale
            kd = float(self._cmd_kd[i]) * self._kd_scale
            tau = float(self._cmd_tau[i])

            ctrl = (
                tau
                + kp * (float(self._cmd_q[i]) - q_actual)
                + kd * (float(self._cmd_dq[i]) - dq_actual)
            )

            # Respect actuator ctrlrange if present.
            lo = float(self.mj_model.actuator_ctrlrange[i, 0])
            hi = float(self.mj_model.actuator_ctrlrange[i, 1])
            if lo < hi:
                if ctrl < lo:
                    ctrl = lo
                elif ctrl > hi:
                    ctrl = hi

            self.mj_data.ctrl[i] = ctrl

    def publish_state(self) -> None:
        """Publish LowState and SportModeState from MuJoCo sensor data.

        Should be called at each physics step.
        """
        with self._data_lock:
            # Apply control at the MuJoCo physics rate.
            # - Before commands: hold-position PD (prevents falling during startup).
            # - After commands: PD based on latest cached LowCmd targets (emulates robot low-level loop).
            if self._commands_received:
                self._apply_policy_pd_control()
            else:
                self._apply_hold_position_control()

            self._build_lowstate()
            self._build_sportstate()

        self._state_pub.Write(self._low_state)
        self._sport_pub.Write(self._sport_state)

    def _build_lowstate(self) -> None:
        """Build LowState message from MuJoCo sensor data."""
        # Motor states: position, velocity, torque
        for i in range(self.num_motors):
            self._low_state.motor_state[i].q = self._get_joint_pos(i)
            self._low_state.motor_state[i].dq = self._get_joint_vel(i)
            self._low_state.motor_state[i].tau_est = self._get_joint_torque(i)

        # IMU state
        if self._imu_quat_idx >= 0:
            idx = self._imu_quat_idx
            self._low_state.imu_state.quaternion[0] = self.mj_data.sensordata[idx]
            self._low_state.imu_state.quaternion[1] = self.mj_data.sensordata[idx + 1]
            self._low_state.imu_state.quaternion[2] = self.mj_data.sensordata[idx + 2]
            self._low_state.imu_state.quaternion[3] = self.mj_data.sensordata[idx + 3]

        if self._imu_gyro_idx >= 0:
            idx = self._imu_gyro_idx
            self._low_state.imu_state.gyroscope[0] = self.mj_data.sensordata[idx]
            self._low_state.imu_state.gyroscope[1] = self.mj_data.sensordata[idx + 1]
            self._low_state.imu_state.gyroscope[2] = self.mj_data.sensordata[idx + 2]

        if self._imu_accel_idx >= 0:
            idx = self._imu_accel_idx
            self._low_state.imu_state.accelerometer[0] = self.mj_data.sensordata[idx]
            self._low_state.imu_state.accelerometer[1] = self.mj_data.sensordata[idx + 1]
            self._low_state.imu_state.accelerometer[2] = self.mj_data.sensordata[idx + 2]

    def _build_sportstate(self) -> None:
        """Build SportModeState message from MuJoCo sensor data."""
        if self._frame_pos_idx >= 0:
            idx = self._frame_pos_idx
            self._sport_state.position[0] = self.mj_data.sensordata[idx]
            self._sport_state.position[1] = self.mj_data.sensordata[idx + 1]
            self._sport_state.position[2] = self.mj_data.sensordata[idx + 2]

        if self._frame_vel_idx >= 0:
            idx = self._frame_vel_idx
            self._sport_state.velocity[0] = self.mj_data.sensordata[idx]
            self._sport_state.velocity[1] = self.mj_data.sensordata[idx + 1]
            self._sport_state.velocity[2] = self.mj_data.sensordata[idx + 2]


G1_MOTOR_JOINT_NAMES: list[str] = [
    # Left leg (0-5)
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    # Right leg (6-11)
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    # Waist (12-14)
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    # Left arm (15-21)
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    # Right arm (22-28)
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]


class SDK2MirrorController:
    """Subscriber-only DDS controller for 'mirror' mode.

    Subscribes to the robot's SDK2 topics and writes the received state into MuJoCo `qpos/qvel`
    for visualization. It does NOT publish any SDK2 topics (avoids conflicting with the real robot).
    """

    def __init__(
        self,
        mj_model: mujoco.MjModel,
        mj_data: mujoco.MjData,
        config: SDK2BridgeConfig,
    ) -> None:
        self.mj_model = mj_model
        self.mj_data = mj_data
        self.config = config

        (
            _LowCmd_,
            self.LowState_,
            self.SportModeState_,
            _LowState_default,
            _SportModeState_default,
        ) = _get_idl_types(config.robot_type)

        # Buffers (motor order)
        import numpy as np

        if config.robot_type != "g1":
            raise NotImplementedError("SDK2MirrorController currently supports robot_type='g1' only")

        # The DDS ChannelSubscriber spins its own reader thread. Create the lock + buffers
        # BEFORE calling .Init(...) to avoid a race where the callback fires immediately.
        self._data_lock = threading.Lock()

        self._num_motors = len(G1_MOTOR_JOINT_NAMES)
        self._joint_pos = np.zeros(self._num_motors, dtype=np.float32)
        self._joint_vel = np.zeros(self._num_motors, dtype=np.float32)
        self._imu_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)  # w,x,y,z
        self._base_ang_vel = np.zeros(3, dtype=np.float32)
        self._state_received = False

        # DDS
        ChannelFactoryInitialize(config.domain_id, config.interface)
        self._state_sub = ChannelSubscriber(TOPIC_LOWSTATE, self.LowState_)
        self._state_sub.Init(self._lowstate_callback, 10)
        self._sport_sub = ChannelSubscriber(TOPIC_SPORTMODESTATE, self.SportModeState_)
        self._sport_sub.Init(self._sportstate_callback, 10)

        # Map motor_idx -> qposadr/qveladr (by joint name)
        self._motor_qposadr: list[int] = []
        self._motor_qveladr: list[int] = []
        for name in G1_MOTOR_JOINT_NAMES:
            jid = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                self._motor_qposadr.append(-1)
                self._motor_qveladr.append(-1)
                logger.warning("SDK2 mirror: joint not found in model", joint=name)
                continue
            self._motor_qposadr.append(int(self.mj_model.jnt_qposadr[jid]))
            self._motor_qveladr.append(int(self.mj_model.jnt_dofadr[jid]))

        logger.info(
            "SDK2 mirror initialized",
            domain_id=config.domain_id,
            interface=config.interface,
            robot_type=config.robot_type,
        )

    def _lowstate_callback(self, msg: Any) -> None:
        with self._data_lock:
            for i in range(self._num_motors):
                self._joint_pos[i] = msg.motor_state[i].q
                self._joint_vel[i] = msg.motor_state[i].dq

            self._imu_quat[0] = msg.imu_state.quaternion[0]
            self._imu_quat[1] = msg.imu_state.quaternion[1]
            self._imu_quat[2] = msg.imu_state.quaternion[2]
            self._imu_quat[3] = msg.imu_state.quaternion[3]

            self._base_ang_vel[0] = msg.imu_state.gyroscope[0]
            self._base_ang_vel[1] = msg.imu_state.gyroscope[1]
            self._base_ang_vel[2] = msg.imu_state.gyroscope[2]

            self._state_received = True

    def _sportstate_callback(self, msg: Any) -> None:
        # Currently unused for visualization-only mirror mode.
        _ = msg

    def apply_to_mujoco(self) -> bool:
        """Apply latest robot state to MuJoCo qpos/qvel. Returns True if updated."""
        with self._data_lock:
            if not self._state_received:
                return False
            joint_pos = self._joint_pos.copy()
            joint_vel = self._joint_vel.copy()
            imu_quat = self._imu_quat.copy()
            base_ang_vel = self._base_ang_vel.copy()

        # Update freejoint orientation from IMU (keep position fixed for now).
        self.mj_data.qpos[3:7] = imu_quat
        self.mj_data.qvel[3:6] = base_ang_vel

        for motor_idx in range(self._num_motors):
            qadr = self._motor_qposadr[motor_idx]
            vadr = self._motor_qveladr[motor_idx]
            if qadr >= 0:
                self.mj_data.qpos[qadr] = float(joint_pos[motor_idx])
            if vadr >= 0:
                self.mj_data.qvel[vadr] = float(joint_vel[motor_idx])

        return True


__all__ = ["SDK2BridgeConfig", "SDK2BridgeController", "SDK2MirrorController"]
