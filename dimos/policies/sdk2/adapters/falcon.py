from __future__ import annotations

# pyright: reportMissingImports=false
# pyright: reportMissingModuleSource=false

from pathlib import Path

import numpy as np

from dimos.robot.unitree.falcon.loco_manip_core import FalconLocoManipCore
from dimos.robot.unitree.sdk2.joints import G1_SDK2_MOTOR_JOINT_NAMES
from dimos.utils.logging_config import setup_logger

from ..types import CommandContext, JointTargets, RobotState

logger = setup_logger()

class FalconLocoManipAdapter:
    """Adapter for FALCON loco_manip ONNX policies.

    This reproduces Falcon's BasePolicy obs buffering/packing:
    - obs_dict values are sorted lexicographically before concatenation
    - obs buffers are history-stacked by shift+append each step
    """

    def __init__(
        self,
        *,
        policy_path: str,
        falcon_yaml_path: str,
        policy_action_scale: float | None = None,
        providers: list[str] | None = None,
    ) -> None:
        # Use vendored, threadless Falcon core (obs packing + IK + action postprocess).
        self._core = FalconLocoManipCore(
            onnx_path=policy_path,
            yaml_path=falcon_yaml_path,
            policy_action_scale=policy_action_scale,
            providers=providers,
        )
        self._joint_names = list(G1_SDK2_MOTOR_JOINT_NAMES)

        # Defaults for runtime hold behavior (motor order).
        self.default_kp = self._core.cfg.motor_kp.astype(np.float32)
        self.default_kd = self._core.cfg.motor_kd.astype(np.float32)

        logger.info("FalconLocoManipAdapter loaded", policy_path=policy_path, falcon_yaml=str(Path(falcon_yaml_path)))

    @property
    def joint_names(self) -> list[str]:
        return list(self._joint_names)

    def reset(self) -> None:
        self._core.reset()

    def _parse_obs(self, current_obs_buffer_dict: dict[str, NDArray[np.floating]]) -> dict[str, NDArray[np.floating]]:
        current_obs_dict: dict[str, NDArray[np.floating]] = {}
        for key in self.cfg.obs_dict:
            obs_list = sorted(self.cfg.obs_dict[key])
            current_obs_dict[key] = np.concatenate(
                [current_obs_buffer_dict[name] * float(self.cfg.obs_scales[name]) for name in obs_list],
                axis=1,
            ).astype(np.float32)
        return current_obs_dict

    def _update_history(self, current_obs_dict: dict[str, NDArray[np.floating]]) -> None:
        for key in self._obs_buf_dict:
            dim = self._obs_dim_dict[key]
            hist = int(self.cfg.history_length_dict[key])
            self._obs_buf_dict[key] = np.concatenate(
                (self._obs_buf_dict[key][:, dim : (dim * hist)], current_obs_dict[key]),
                axis=1,
            ).astype(np.float32)

    def step(self, state: RobotState, ctx: CommandContext) -> JointTargets:
        # Convert Dimos RobotState -> Falcon robot_state_data layout (Falcon uses motor order).
        q = np.zeros(3 + 4 + 29, dtype=np.float64)
        dq = np.zeros(3 + 3 + 29, dtype=np.float64)
        tau_est = np.zeros(3 + 3 + 29, dtype=np.float64)
        ddq = np.zeros(3 + 3 + 29, dtype=np.float64)

        q[3:7] = state.imu_quat_wxyz.astype(np.float64, copy=False)
        q[7 : 7 + 29] = state.q.astype(np.float64, copy=False)
        dq[3:6] = state.base_ang_vel.astype(np.float64, copy=False)
        dq[6 : 6 + 29] = state.dq.astype(np.float64, copy=False)

        robot_state_data = np.array(q.tolist() + dq.tolist() + tau_est.tolist() + ddq.tolist(), dtype=np.float64).reshape(1, -1)

        stand = bool(ctx.extra.get("stand", True)) if "stand" in ctx.extra else bool(ctx.stand)
        upper_body_ik_enabled = bool(ctx.extra.get("upper_body_ik_enabled", False))
        upper_body_collision_check = bool(ctx.extra.get("upper_body_collision_check", True))

        ee_left = np.asarray(ctx.extra.get("ee_left_xyz", ctx.ee_left_xyz), dtype=np.float32)
        ee_right = np.asarray(ctx.extra.get("ee_right_xyz", ctx.ee_right_xyz), dtype=np.float32)
        ee_yaw_deg = float(ctx.extra.get("ee_yaw_deg", ctx.ee_yaw_deg))

        self._core.set_commands(
            cmd_vel_xyw=ctx.cmd_vel,
            stand=stand,
            base_height=ctx.base_height,
            waist_rpy=ctx.waist_rpy,
            ee_left_offset=ee_left,
            ee_right_offset=ee_right,
            ee_yaw_deg=ee_yaw_deg if upper_body_ik_enabled else 0.0,
        )

        q_target, kp, kd = self._core.step(robot_state_data=robot_state_data, upper_body_collision_check=upper_body_collision_check)
        kp = (kp.astype(np.float32) * float(ctx.kp_scale)).astype(np.float32)
        kd = (kd.astype(np.float32) * float(ctx.kp_scale)).astype(np.float32)
        return JointTargets(q_target=q_target.astype(np.float32), kp=kp, kd=kd)


