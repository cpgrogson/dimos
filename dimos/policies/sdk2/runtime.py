from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import NDArray

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

from dimos.utils.logging_config import setup_logger

from .adapter import PolicyAdapter
from .lowcmd_builder import LowCmdBuilder, LowCmdBuilderConfig
from .types import CommandContext, G1_MOTOR_JOINT_NAMES, JointTargets, RobotState

logger = setup_logger()

TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_SPORTMODESTATE = "rt/sportmodestate"


def _quat_to_projected_gravity(quat_wxyz: NDArray[np.floating]) -> NDArray[np.floating]:
    w, x, y, z = quat_wxyz
    gx = -2.0 * (x * z - w * y)
    gy = -2.0 * (y * z + w * x)
    gz = -1.0 + 2.0 * (x * x + y * y)
    return np.array([gx, gy, gz], dtype=np.float32)


def _world_to_body_velocity(world_vel: NDArray[np.floating], quat_wxyz: NDArray[np.floating]) -> NDArray[np.floating]:
    w, x, y, z = quat_wxyz
    vx = (1 - 2 * (y * y + z * z)) * world_vel[0] + 2 * (x * y + w * z) * world_vel[1] + 2 * (x * z - w * y) * world_vel[2]
    vy = 2 * (x * y - w * z) * world_vel[0] + (1 - 2 * (x * x + z * z)) * world_vel[1] + 2 * (y * z + w * x) * world_vel[2]
    vz = 2 * (x * z + w * y) * world_vel[0] + 2 * (y * z - w * x) * world_vel[1] + (1 - 2 * (x * x + y * y)) * world_vel[2]
    return np.array([vx, vy, vz], dtype=np.float32)


@dataclass
class PolicyRuntimeConfig:
    robot_type: str = "g1"
    domain_id: int = 1
    interface: str = "lo0"
    control_dt: float = 0.02  # policy update rate
    mode_pr: int = 0  # HG: 0=PR, 1=AB


class PolicyRuntime:
    """Runs a PolicyAdapter over SDK2 DDS topics (rt/lowstate -> rt/lowcmd)."""

    def __init__(self, *, adapter: PolicyAdapter, config: PolicyRuntimeConfig) -> None:
        self.adapter = adapter
        self.config = config

        # Joint mapping: motor order -> policy order.
        if config.robot_type == "g1":
            motor_joint_names = G1_MOTOR_JOINT_NAMES
        else:
            raise NotImplementedError(f"PolicyRuntime joint mapping not implemented for {config.robot_type}")

        policy_name_to_idx = {n: i for i, n in enumerate(self.adapter.joint_names)}
        motor_to_policy: list[int] = []
        for motor_name in motor_joint_names:
            if motor_name not in policy_name_to_idx:
                raise ValueError(f"Motor joint '{motor_name}' not found in policy joint_names")
            motor_to_policy.append(policy_name_to_idx[motor_name])
        self._motor_to_policy = np.array(motor_to_policy, dtype=np.int32)

        motor_name_to_idx = {n: i for i, n in enumerate(motor_joint_names)}
        policy_to_motor: list[int] = []
        for policy_name in self.adapter.joint_names:
            if policy_name not in motor_name_to_idx:
                raise ValueError(f"Policy joint '{policy_name}' not found in motor joint names")
            policy_to_motor.append(motor_name_to_idx[policy_name])
        self._policy_to_motor = np.array(policy_to_motor, dtype=np.int32)

        self._num_joints = len(self.adapter.joint_names)
        self._joint_pos_motor = np.zeros(len(motor_joint_names), dtype=np.float32)
        self._joint_vel_motor = np.zeros(len(motor_joint_names), dtype=np.float32)

        self._imu_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self._base_ang_vel = np.zeros(3, dtype=np.float32)
        self._base_lin_vel_world = np.zeros(3, dtype=np.float32)  # from SportModeState in world frame

        self._mode_machine: int = 0
        self._enabled = False
        self._estop = False
        self._ctx = CommandContext()
        # Latest gains/feedforward in motor order (used for disabled "hold pose" behavior).
        self._hold_kp_motor = np.zeros(len(motor_joint_names), dtype=np.float32)
        self._hold_kd_motor = np.zeros(len(motor_joint_names), dtype=np.float32)
        self._hold_tau_motor = np.zeros(len(motor_joint_names), dtype=np.float32)

        # Seed hold gains from adapter defaults if available (e.g. mjlab metadata),
        # so the robot stays stiff even before the policy is ever enabled.
        try:
            default_kp_policy = getattr(self.adapter, "default_kp", None)
            default_kd_policy = getattr(self.adapter, "default_kd", None)
            if default_kp_policy is not None:
                self._hold_kp_motor[self._policy_to_motor] = np.asarray(default_kp_policy, dtype=np.float32)
            if default_kd_policy is not None:
                self._hold_kd_motor[self._policy_to_motor] = np.asarray(default_kd_policy, dtype=np.float32)
        except Exception:
            pass

        self._data_lock = threading.Lock()
        self._state_received = False
        self._last_q_policy: NDArray[np.floating] | None = None

        # DDS init
        ChannelFactoryInitialize(config.domain_id, config.interface)

        # Types
        if config.robot_type in ("g1", "h1_2"):
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as LowCmdHg
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as LowStateHg

            self._LowCmd = LowCmdHg
            self._LowState = LowStateHg
        else:
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_ as LowCmdGo
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ as LowStateGo

            self._LowCmd = LowCmdGo
            self._LowState = LowStateGo

        self._cmd_pub = ChannelPublisher(TOPIC_LOWCMD, self._LowCmd)
        self._cmd_pub.Init()

        self._state_sub = ChannelSubscriber(TOPIC_LOWSTATE, self._LowState)
        self._state_sub.Init(self._lowstate_callback, 10)

        self._sport_sub = ChannelSubscriber(TOPIC_SPORTMODESTATE, SportModeState_)
        self._sport_sub.Init(self._sportstate_callback, 10)

        self._cmd_builder = LowCmdBuilder(LowCmdBuilderConfig(robot_type=config.robot_type, mode_pr=config.mode_pr))

        logger.info(
            "PolicyRuntime initialized",
            robot_type=config.robot_type,
            domain_id=config.domain_id,
            interface=config.interface,
            control_dt=config.control_dt,
        )

    def reset(self) -> None:
        self.adapter.reset()
        with self._data_lock:
            self._state_received = False

    def set_enabled(self, enabled: bool) -> None:
        with self._data_lock:
            self._enabled = bool(enabled)

    def set_estop(self, estop: bool) -> None:
        with self._data_lock:
            self._estop = bool(estop)
            if self._estop:
                self._enabled = False

    def set_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        with self._data_lock:
            self._ctx.cmd_vel[0] = float(vx)
            self._ctx.cmd_vel[1] = float(vy)
            self._ctx.cmd_vel[2] = float(wz)

    def set_policy_params_json(self, params_json: str) -> None:
        """Set adapter-specific params from a JSON blob."""
        try:
            data = json.loads(params_json) if params_json else {}
        except Exception:
            data = {}
        with self._data_lock:
            if isinstance(data, dict):
                # Common fields (best-effort)
                if "stand" in data:
                    self._ctx.stand = int(bool(data["stand"]))
                if "base_height" in data:
                    try:
                        self._ctx.base_height = float(data["base_height"])
                    except Exception:
                        pass
                if "waist_rpy" in data and isinstance(data["waist_rpy"], (list, tuple)) and len(data["waist_rpy"]) == 3:
                    self._ctx.waist_rpy[:] = np.array(data["waist_rpy"], dtype=np.float32)
                if "kp_scale" in data:
                    try:
                        self._ctx.kp_scale = float(data["kp_scale"])
                    except Exception:
                        pass
                if "ee_left_xyz" in data and isinstance(data["ee_left_xyz"], (list, tuple)) and len(data["ee_left_xyz"]) == 3:
                    self._ctx.ee_left_xyz[:] = np.array(data["ee_left_xyz"], dtype=np.float32)
                if "ee_right_xyz" in data and isinstance(data["ee_right_xyz"], (list, tuple)) and len(data["ee_right_xyz"]) == 3:
                    self._ctx.ee_right_xyz[:] = np.array(data["ee_right_xyz"], dtype=np.float32)
                if "ee_yaw_deg" in data:
                    try:
                        self._ctx.ee_yaw_deg = float(data["ee_yaw_deg"])
                    except Exception:
                        pass
                # Keep raw extras for adapters to use
                self._ctx.extra = data

    def _lowstate_callback(self, msg: Any) -> None:
        with self._data_lock:
            for i in range(len(self._joint_pos_motor)):
                self._joint_pos_motor[i] = msg.motor_state[i].q
                self._joint_vel_motor[i] = msg.motor_state[i].dq

            if hasattr(msg, "mode_machine"):
                try:
                    self._mode_machine = int(msg.mode_machine)
                except Exception:
                    pass

            self._imu_quat[0] = msg.imu_state.quaternion[0]
            self._imu_quat[1] = msg.imu_state.quaternion[1]
            self._imu_quat[2] = msg.imu_state.quaternion[2]
            self._imu_quat[3] = msg.imu_state.quaternion[3]

            self._base_ang_vel[0] = msg.imu_state.gyroscope[0]
            self._base_ang_vel[1] = msg.imu_state.gyroscope[1]
            self._base_ang_vel[2] = msg.imu_state.gyroscope[2]

            self._state_received = True

    def _sportstate_callback(self, msg: SportModeState_) -> None:
        with self._data_lock:
            self._base_lin_vel_world[0] = msg.velocity[0]
            self._base_lin_vel_world[1] = msg.velocity[1]
            self._base_lin_vel_world[2] = msg.velocity[2]

    def step(self) -> None:
        """Run one policy step and publish rt/lowcmd."""
        with self._data_lock:
            if not self._state_received:
                return
            enabled = bool(self._enabled)
            estop = bool(self._estop)
            mode_machine = int(self._mode_machine)
            ctx = self._ctx  # CommandContext is mutated under lock; we treat it as immutable snapshot here.

            joint_pos_motor = self._joint_pos_motor.copy()
            joint_vel_motor = self._joint_vel_motor.copy()
            imu_quat = self._imu_quat.copy()
            base_ang_vel = self._base_ang_vel.copy()
            base_lin_vel_world = self._base_lin_vel_world.copy()

        # Reorder into policy joint order.
        q_policy = joint_pos_motor[self._policy_to_motor].astype(np.float32, copy=False)
        dq_policy = joint_vel_motor[self._policy_to_motor].astype(np.float32, copy=False)

        base_lin_vel_body = _world_to_body_velocity(base_lin_vel_world, imu_quat)
        proj_gravity = _quat_to_projected_gravity(imu_quat)

        state = RobotState(
            t_wall_s=time.time(),
            base_lin_vel=base_lin_vel_body,
            base_ang_vel=base_ang_vel,
            imu_quat_wxyz=imu_quat,
            projected_gravity=proj_gravity,
            q=q_policy.copy(),
            dq=dq_policy.copy(),
        )

        # Heuristic reset detection: a MuJoCo viewer reset teleports joint state.
        # Reset adapter history so history-buffered policies (e.g. Falcon) can re-stabilize.
        if self._last_q_policy is not None:
            try:
                max_jump = float(np.max(np.abs(state.q - self._last_q_policy)))
                if max_jump > 1.0:
                    self.adapter.reset()
            except Exception:
                pass
        self._last_q_policy = state.q.copy()

        if estop:
            # Limp: publish enabled=False with zero gains/torques, hold current motor q.
            cmd = self._cmd_builder.build(
                mode_machine=mode_machine,
                enabled=False,
                q=joint_pos_motor,
                dq=np.zeros_like(joint_pos_motor),
                kp=np.zeros_like(joint_pos_motor),
                kd=np.zeros_like(joint_pos_motor),
                tau=np.zeros_like(joint_pos_motor),
            )
            self._cmd_pub.Write(cmd)
            return

        if not enabled:
            # Hold current pose with the latest known gains so the robot doesn't go limp
            # while waiting for the UI to enable the policy (mjlab expected behavior).
            q_target_motor = joint_pos_motor
            cmd = self._cmd_builder.build(
                mode_machine=mode_machine,
                enabled=True,
                q=q_target_motor,
                dq=np.zeros_like(q_target_motor),
                kp=self._hold_kp_motor,
                kd=self._hold_kd_motor,
                tau=self._hold_tau_motor,
            )
            self._cmd_pub.Write(cmd)
            return

        targets = self.adapter.step(state, ctx)

        # Convert targets from policy order -> motor order.
        q_target_motor = np.zeros_like(joint_pos_motor)
        q_target_motor[self._policy_to_motor] = targets.q_target

        def _maybe_reorder(arr: NDArray[np.floating] | None) -> NDArray[np.floating] | None:
            if arr is None:
                return None
            out = np.zeros_like(joint_pos_motor)
            out[self._policy_to_motor] = arr
            return out

        cmd = self._cmd_builder.build(
            mode_machine=mode_machine,
            enabled=True,
            q=q_target_motor,
            dq=_maybe_reorder(targets.dq_target),
            kp=_maybe_reorder(targets.kp),
            kd=_maybe_reorder(targets.kd),
            tau=_maybe_reorder(targets.tau_ff),
        )
        self._cmd_pub.Write(cmd)

        # Cache gains for disabled hold mode (motor order).
        kp_m = _maybe_reorder(targets.kp)
        kd_m = _maybe_reorder(targets.kd)
        tau_m = _maybe_reorder(targets.tau_ff)
        if kp_m is not None:
            self._hold_kp_motor = kp_m.astype(np.float32, copy=False)
        if kd_m is not None:
            self._hold_kd_motor = kd_m.astype(np.float32, copy=False)
        if tau_m is not None:
            self._hold_tau_motor = tau_m.astype(np.float32, copy=False)


