from __future__ import annotations

# pyright: reportMissingImports=false
# pyright: reportMissingModuleSource=false

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import onnxruntime as ort
import yaml

from dimos.robot.unitree.sdk2.joints import G1_SDK2_MOTOR_JOINT_NAMES

from .utils.math import quat_rotate_inverse_numpy


@dataclass
class FalconCoreConfig:
    robot_type: str
    default_dof_angles: np.ndarray
    motor_kp: np.ndarray
    motor_kd: np.ndarray
    motor_pos_lower: np.ndarray | None
    motor_pos_upper: np.ndarray | None
    obs_dict: dict[str, list[str]]
    obs_dims: dict[str, int]
    obs_scales: dict[str, float]
    history_length_dict: dict[str, int]
    desired_base_height: float
    gait_period: float
    residual_upper_body_action: bool
    use_upper_body_controller: bool
    unitree_mode_pr: int
    unitree_mode_machine: int
    asset_file: str
    asset_root: str


def _calc_obs_dim_dict(obs_dict: dict[str, list[str]], obs_dims: dict[str, int]) -> dict[str, int]:
    out: dict[str, int] = {}
    for key, items in obs_dict.items():
        out[key] = int(sum(int(obs_dims[name]) for name in items))
    return out


class FalconLocoManipCore:
    """A Dimos-friendly, threadless core of Falcon loco_manip.

    This keeps Falcon's obs packing + history buffering semantics and the IK/ref_upper_dof_pos
    update rule, but avoids Falcon's own keyboard/joystick threads and its DDS publishers.
    """

    def __init__(
        self,
        *,
        onnx_path: str,
        yaml_path: str,
        policy_action_scale: float | None = None,
        providers: list[str] | None = None,
    ) -> None:
        yaml_file = Path(yaml_path)
        cfg_raw = yaml.safe_load(yaml_file.read_text(encoding="utf-8"))

        # Resolve URDF/mesh paths relative to the YAML location so MuJoCo profiles can be
        # self-contained (e.g. data/mujoco_sim/unitree_g1_falcon/...).
        base_dir = yaml_file.parent
        asset_root_raw = str(cfg_raw.get("ASSET_ROOT", "") or "")
        asset_file_raw = str(cfg_raw.get("ASSET_FILE", "") or "")
        if asset_root_raw:
            asset_root_path = Path(asset_root_raw)
            if not asset_root_path.is_absolute():
                asset_root_path = (base_dir / asset_root_path).resolve()
            cfg_raw["ASSET_ROOT"] = str(asset_root_path)
        if asset_file_raw:
            asset_file_path = Path(asset_file_raw)
            if not asset_file_path.is_absolute():
                asset_file_path = (base_dir / asset_file_path).resolve()
            cfg_raw["ASSET_FILE"] = str(asset_file_path)

        self.cfg = FalconCoreConfig(
            robot_type=str(cfg_raw["ROBOT_TYPE"]),
            default_dof_angles=np.array(cfg_raw["DEFAULT_DOF_ANGLES"], dtype=np.float32),
            motor_kp=np.array(cfg_raw.get("MOTOR_KP", cfg_raw.get("JOINT_KP")), dtype=np.float32),
            motor_kd=np.array(cfg_raw.get("MOTOR_KD", cfg_raw.get("JOINT_KD")), dtype=np.float32),
            motor_pos_lower=np.array(cfg_raw["motor_pos_lower_limit_list"], dtype=np.float32) if "motor_pos_lower_limit_list" in cfg_raw else None,
            motor_pos_upper=np.array(cfg_raw["motor_pos_upper_limit_list"], dtype=np.float32) if "motor_pos_upper_limit_list" in cfg_raw else None,
            obs_dict=cfg_raw["obs_dict"],
            obs_dims=cfg_raw["obs_dims"],
            obs_scales=cfg_raw["obs_scales"],
            history_length_dict=cfg_raw["history_length_dict"],
            desired_base_height=float(cfg_raw.get("DESIRED_BASE_HEIGHT", 0.75)),
            gait_period=float(cfg_raw.get("GAIT_PERIOD", 0.9)),
            residual_upper_body_action=bool(cfg_raw.get("residual_upper_body_action", True)),
            use_upper_body_controller=bool(cfg_raw.get("use_upper_body_controller", True)),
            unitree_mode_pr=int(cfg_raw.get("UNITREE_LEGGED_CONST", {}).get("MODE_PR", 0)),
            unitree_mode_machine=int(cfg_raw.get("UNITREE_LEGGED_CONST", {}).get("MODE_MACHINE", 0)),
            asset_file=str(cfg_raw.get("ASSET_FILE", "")),
            asset_root=str(cfg_raw.get("ASSET_ROOT", "")),
        )

        self.num_dofs = int(cfg_raw.get("NUM_JOINTS", 29))
        self.num_upper = int(cfg_raw.get("NUM_UPPER_BODY_JOINTS", 14))
        self.upper_dof_names = list(cfg_raw.get("dof_names_upper_body", []))
        self._upper_indices = [G1_SDK2_MOTOR_JOINT_NAMES.index(n) for n in self.upper_dof_names] if self.upper_dof_names else list(range(15, 29))

        self.session = ort.InferenceSession(onnx_path, providers=providers or ort.get_available_providers())
        self.input_names = [i.name for i in self.session.get_inputs()]

        self.obs_dim_dict = _calc_obs_dim_dict(self.cfg.obs_dict, self.cfg.obs_dims)
        self.obs_buf_dict: dict[str, np.ndarray] = {
            key: np.zeros((1, self.obs_dim_dict[key] * int(self.cfg.history_length_dict[key])), dtype=np.float32)
            for key in self.obs_dim_dict
        }

        # Commands (Falcon shapes)
        self.lin_vel_command = np.array([[0.0, 0.0]], dtype=np.float32)
        self.ang_vel_command = np.array([[0.0]], dtype=np.float32)
        self.stand_command = np.array([[0.0]], dtype=np.float32)
        self.base_height_command = np.array([[self.cfg.desired_base_height]], dtype=np.float32)
        self.waist_dofs_command = np.zeros((1, 3), dtype=np.float32)

        self.last_policy_action = np.zeros((1, self.num_dofs), dtype=np.float32)
        self.policy_action_scale = float(policy_action_scale if policy_action_scale is not None else cfg_raw.get("policy_action_scale", 0.25))

        # Upper body reference
        self.ref_upper_dof_pos = np.zeros((1, self.num_upper), dtype=np.float32)
        # Falcon sets upper reference to default upper angles
        self.ref_upper_dof_pos[:] = self.cfg.default_dof_angles[self._upper_indices]

        # Upper body controller (IK)
        self.upper_body_controller: G1_29_ArmIK_NoWrists | None = None
        if self.cfg.use_upper_body_controller:
            # Import IK lazily so the Falcon core can be imported even if IK deps are missing.
            from .arm_ik import G1_29_ArmIK_NoWrists

            self.upper_body_controller = G1_29_ArmIK_NoWrists(
                Unit_Test=False, Visualization=False, robot_config=cfg_raw
            )

        # EE target defaults (from Falcon loco_manip)
        self._degrees = 0.0
        self._theta = 0.0
        self._ee_left_xyz = np.array([0.30, 0.13, 0.08], dtype=np.float32)
        self._ee_right_xyz = np.array([0.30, -0.13, 0.08], dtype=np.float32)
        self._ee_left_R = np.eye(3, dtype=np.float64)
        self._ee_right_R = np.eye(3, dtype=np.float64)
        self._EE_efrc_L = np.array([0, 0, 0, 0, 0, 0], dtype=np.float64)
        self._EE_efrc_R = np.array([0, 0, 0, 0, 0, 0], dtype=np.float64)

        self._ik_initialized = False

    def reset(self) -> None:
        for k in self.obs_buf_dict:
            self.obs_buf_dict[k][:] = 0.0
        self.last_policy_action[:] = 0.0
        self.ref_upper_dof_pos[:] = self.cfg.default_dof_angles[self._upper_indices]
        self._ik_initialized = False

    def set_commands(
        self,
        *,
        cmd_vel_xyw: np.ndarray,
        stand: bool,
        base_height: float | None,
        waist_rpy: np.ndarray,
        ee_left_offset: np.ndarray,
        ee_right_offset: np.ndarray,
        ee_yaw_deg: float,
    ) -> None:
        self.stand_command[0, 0] = 1.0 if stand else 0.0
        self.lin_vel_command[0, 0] = float(cmd_vel_xyw[0]) * self.stand_command[0, 0]
        self.lin_vel_command[0, 1] = float(cmd_vel_xyw[1]) * self.stand_command[0, 0]
        self.ang_vel_command[0, 0] = float(cmd_vel_xyw[2]) * self.stand_command[0, 0]

        if base_height is None:
            self.base_height_command[0, 0] = self.cfg.desired_base_height if stand else 0.0
        else:
            self.base_height_command[0, 0] = float(base_height)

        self.waist_dofs_command[0, :] = waist_rpy.astype(np.float32, copy=False)

        self._degrees = float(ee_yaw_deg)
        self._theta = np.radians(self._degrees)
        self._ee_left_R = np.array(
            [[np.cos(-self._theta), -np.sin(-self._theta), 0], [np.sin(-self._theta), np.cos(-self._theta), 0], [0, 0, 1]],
            dtype=np.float64,
        )
        self._ee_right_R = np.array(
            [[np.cos(self._theta), -np.sin(self._theta), 0], [np.sin(self._theta), np.cos(self._theta), 0], [0, 0, 1]],
            dtype=np.float64,
        )

        self._ee_left_xyz = (np.array([0.30, 0.13, 0.08], dtype=np.float32) + ee_left_offset.astype(np.float32, copy=False))
        self._ee_right_xyz = (np.array([0.30, -0.13, 0.08], dtype=np.float32) + ee_right_offset.astype(np.float32, copy=False))

    def _parse_current_obs_dict(self, current_obs_buffer_dict: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
        current_obs_dict: dict[str, np.ndarray] = {}
        for key in self.cfg.obs_dict:
            obs_list = sorted(self.cfg.obs_dict[key])
            current_obs_dict[key] = np.concatenate(
                [current_obs_buffer_dict[name] * float(self.cfg.obs_scales[name]) for name in obs_list],
                axis=1,
            )
        return current_obs_dict

    def _prepare_obs_for_rl(self, robot_state_data: np.ndarray) -> dict[str, np.ndarray]:
        # Matches Falcon base_policy.get_current_obs_buffer_dict for the channels we use in loco_manip.
        base_quat = robot_state_data[:, 3:7]
        dof_pos = robot_state_data[:, 7 : 7 + self.num_dofs] - self.cfg.default_dof_angles.reshape(1, -1)
        dof_vel = robot_state_data[:, 7 + self.num_dofs + 6 : 7 + self.num_dofs + 6 + self.num_dofs]
        base_ang_vel = robot_state_data[:, 7 + self.num_dofs + 3 : 7 + self.num_dofs + 6]

        v = np.array([[0, 0, -1]], dtype=np.float32)
        projected_gravity = quat_rotate_inverse_numpy(base_quat.astype(np.float32), v.astype(np.float32))

        current_obs_buffer_dict: dict[str, np.ndarray] = {
            "base_ang_vel": base_ang_vel.astype(np.float32),
            "projected_gravity": projected_gravity.astype(np.float32),
            "command_lin_vel": self.lin_vel_command.astype(np.float32),
            "command_ang_vel": self.ang_vel_command.astype(np.float32),
            "command_stand": self.stand_command.astype(np.float32),
            "command_base_height": self.base_height_command.astype(np.float32),
            "command_waist_dofs": self.waist_dofs_command.astype(np.float32),
            "ref_upper_dof_pos": self.ref_upper_dof_pos.astype(np.float32),
            "dof_pos": dof_pos.astype(np.float32),
            "dof_vel": dof_vel.astype(np.float32),
            "actions": self.last_policy_action.astype(np.float32),
        }

        current_obs_dict = self._parse_current_obs_dict(current_obs_buffer_dict)

        # Update history buffers exactly like Falcon
        for key in self.obs_buf_dict:
            dim = self.obs_dim_dict[key]
            hist = int(self.cfg.history_length_dict[key])
            self.obs_buf_dict[key] = np.concatenate(
                (
                    self.obs_buf_dict[key][:, dim : (dim * hist)],
                    current_obs_dict[key],
                ),
                axis=1,
            )

        return {"actor_obs": self.obs_buf_dict["actor_obs"].astype(np.float32)}

    def _update_ref_upper_from_ik(self, upper_body_collision_check: bool) -> None:
        if self.upper_body_controller is None:
            return
        try:
            import pinocchio as pin  # type: ignore[import-not-found]
        except Exception:
            return

        L = pin.SE3(self._ee_left_R, self._ee_left_xyz.astype(np.float64))
        R = pin.SE3(self._ee_right_R, self._ee_right_xyz.astype(np.float64))

        if not self._ik_initialized:
            self.upper_body_controller.set_initial_poses(L.translation, R.translation, L.rotation, R.rotation)
            self._ik_initialized = True

        # NOTE: our vendored `get_q_tau` uses collision_check=True internally.
        upper_body_qpos, _ = self.upper_body_controller.get_q_tau(L, R, self._EE_efrc_L, self._EE_efrc_R)

        arm_reduced_joint_indices = [0, 1, 2, 3, 7, 8, 9, 10]
        for i, idx in enumerate(arm_reduced_joint_indices):
            self.ref_upper_dof_pos[0, idx] = float(upper_body_qpos[i])

        wrist_joint_indices = [19, 20, 21, 26, 27, 28]
        for idx in wrist_joint_indices:
            self.ref_upper_dof_pos[0, idx - 15] = 0.0

    def step(
        self,
        *,
        robot_state_data: np.ndarray,
        upper_body_collision_check: bool = True,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if self.cfg.use_upper_body_controller:
            self._update_ref_upper_from_ik(upper_body_collision_check=upper_body_collision_check)

        obs = self._prepare_obs_for_rl(robot_state_data)

        inp = {self.input_names[0]: obs["actor_obs"]} if len(self.input_names) == 1 else {"actor_obs": obs["actor_obs"]}
        policy_action = self.session.run(None, inp)[0]
        policy_action = np.clip(np.asarray(policy_action, dtype=np.float32), -100, 100)

        if policy_action.ndim == 2:
            policy_action = policy_action[0]
        policy_action = policy_action.reshape(1, -1)

        if policy_action.shape[1] != self.num_dofs:
            pad = self.num_dofs - policy_action.shape[1]
            if pad > 0:
                policy_action = np.concatenate([np.zeros((1, pad), dtype=np.float32), policy_action], axis=1)
            else:
                policy_action = policy_action[:, : self.num_dofs]

        self.last_policy_action = policy_action.copy()
        scaled = policy_action * float(self.policy_action_scale)

        if self.cfg.residual_upper_body_action:
            scaled[:, self._upper_indices] += (self.ref_upper_dof_pos - self.cfg.default_dof_angles[self._upper_indices]).astype(np.float32)

        q_target = (scaled.reshape(-1) + self.cfg.default_dof_angles).astype(np.float32)
        if self.cfg.motor_pos_lower is not None and self.cfg.motor_pos_upper is not None:
            q_target = np.clip(q_target, self.cfg.motor_pos_lower, self.cfg.motor_pos_upper)

        return q_target, self.cfg.motor_kp.astype(np.float32), self.cfg.motor_kd.astype(np.float32)


