from __future__ import annotations

# pyright: reportMissingImports=false
# pyright: reportMissingModuleSource=false

from dataclasses import dataclass
from typing import Any

import numpy as np

from .weighted_moving_filter import WeightedMovingFilter


@dataclass
class G1ArmIKConfig:
    asset_file: str
    asset_root: str
    speed_factor: float = 0.02


class G1ArmIKNoWrists:
    """Pinocchio+Casadi IK solver for G1 arms (no wrists), ported from FALCON.

    Output: 8 joint positions for:
    - left: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow
    - right: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow
    """

    def __init__(self, config: G1ArmIKConfig, visualization: bool = False) -> None:
        # Heavy deps are optional; import lazily.
        import casadi  # type: ignore[import-not-found]
        import pinocchio as pin  # type: ignore[import-not-found]
        from pinocchio import casadi as cpin  # type: ignore[import-not-found]

        self.pin = pin
        self.casadi = casadi
        self.cpin = cpin

        robot_config: dict[str, Any] = {"ASSET_FILE": config.asset_file, "ASSET_ROOT": config.asset_root}
        self._speed_factor = float(config.speed_factor)
        self._visualization = bool(visualization)

        self.robot = pin.RobotWrapper.BuildFromURDF(robot_config["ASSET_FILE"], robot_config["ASSET_ROOT"])

        # Lock lower body joints and waist so we solve only arms.
        joints_to_lock = [
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            "waist_yaw_joint",
            "waist_roll_joint",
            "waist_pitch_joint",
        ]
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=joints_to_lock,
            reference_configuration=np.array([0.0] * self.robot.model.nq),
        )

        # Add custom end-effector frames at the wrists (matches Falcon).
        self.reduced_robot.model.addFrame(
            pin.Frame(
                "L_ee",
                self.reduced_robot.model.getJointId("left_wrist_yaw_joint"),
                pin.SE3(np.eye(3), np.array([0.15, -0.075, 0]).T),
                pin.FrameType.OP_FRAME,
            )
        )
        self.reduced_robot.model.addFrame(
            pin.Frame(
                "R_ee",
                self.reduced_robot.model.getJointId("right_wrist_yaw_joint"),
                pin.SE3(np.eye(3), np.array([0.15, 0.075, 0]).T),
                pin.FrameType.OP_FRAME,
            )
        )

        self.data = self.reduced_robot.model.createData()

        # Casadi symbolic model for optimization
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        self.cTf_l = casadi.SX.sym("tf_l", 4, 4)
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        self.L_hand_id = self.reduced_robot.model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_robot.model.getFrameId("R_ee")

        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.L_hand_id].translation - self.cTf_l[:3, 3],
                    self.cdata.oMf[self.R_hand_id].translation - self.cTf_r[:3, 3],
                )
            ],
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    cpin.log3(self.cdata.oMf[self.L_hand_id].rotation @ self.cTf_l[:3, :3].T),
                    cpin.log3(self.cdata.oMf[self.R_hand_id].rotation @ self.cTf_r[:3, :3].T),
                )
            ],
        )

        # Optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)
        self.param_tf_l = self.opti.parameter(4, 4)
        self.param_tf_r = self.opti.parameter(4, 4)

        translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        regularization_cost = casadi.sumsqr(self.var_q)
        smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)

        self.opti.subject_to(
            self.opti.bounded(
                self.reduced_robot.model.lowerPositionLimit,
                self.var_q,
                self.reduced_robot.model.upperPositionLimit,
            )
        )
        self.opti.minimize(50 * translational_cost + rotation_cost + 0.02 * regularization_cost + 0.1 * smooth_cost)

        opts = {"ipopt": {"print_level": 0, "max_iter": 50, "tol": 1e-6}, "print_time": False, "calc_lam_p": False}
        self.opti.solver("ipopt", opts)

        self.nq = int(self.reduced_robot.model.nq)
        self.nv = int(self.reduced_robot.model.nv)
        self.init_data = np.zeros(self.nq, dtype=np.float64)
        self.smooth_filter = WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), data_size=self.nq)

        # Interpolation state
        self.current_L_tf: np.ndarray | None = None
        self.current_R_tf: np.ndarray | None = None
        self.current_L_orientation = None
        self.current_R_orientation = None

    def set_initial_poses(self, L_tf: np.ndarray, R_tf: np.ndarray, L_orientation: np.ndarray, R_orientation: np.ndarray) -> None:
        pin = self.pin
        self.current_L_tf = np.array(L_tf, dtype=np.float64).copy()
        self.current_R_tf = np.array(R_tf, dtype=np.float64).copy()
        self.current_L_orientation = pin.Quaternion(L_orientation)
        self.current_R_orientation = pin.Quaternion(R_orientation)

    def solve_ik(self, left_wrist: np.ndarray, right_wrist: np.ndarray, current_lr_arm_motor_q: np.ndarray | None = None) -> np.ndarray:
        # Ported from Falcon's G1_29_ArmIK_NoWrists.solve_ik (simplified; no collision check; no torques output).
        if current_lr_arm_motor_q is not None:
            self.init_data = np.asarray(current_lr_arm_motor_q, dtype=np.float64).reshape(-1)[-self.nq :]

        self.opti.set_initial(self.var_q, self.init_data)
        self.opti.set_value(self.param_tf_l, left_wrist)
        self.opti.set_value(self.param_tf_r, right_wrist)
        self.opti.set_value(self.var_q_last, self.init_data)

        try:
            _ = self.opti.solve()
            sol_q = np.asarray(self.opti.value(self.var_q), dtype=np.float64).reshape(-1)
            self.smooth_filter.add_data(sol_q)
            sol_q = np.asarray(self.smooth_filter.filtered_data, dtype=np.float64).reshape(-1)
            self.init_data = sol_q
            return sol_q
        except Exception:
            sol_q = np.asarray(self.opti.debug.value(self.var_q), dtype=np.float64).reshape(-1)
            if sol_q.shape[0] == self.nq:
                self.smooth_filter.add_data(sol_q)
                sol_q = np.asarray(self.smooth_filter.filtered_data, dtype=np.float64).reshape(-1)
                self.init_data = sol_q
                return sol_q
            return self.init_data

    def get_arm_qpos(self, L_target_se3: Any, R_target_se3: Any) -> np.ndarray:
        """Return 8 joint positions (no wrists) for the target EE poses."""
        pin = self.pin
        assert self.current_L_tf is not None and self.current_R_tf is not None

        # Interpolate position/orientation
        self.current_L_tf = (1 - self._speed_factor) * self.current_L_tf + self._speed_factor * np.asarray(L_target_se3.translation)
        self.current_R_tf = (1 - self._speed_factor) * self.current_R_tf + self._speed_factor * np.asarray(R_target_se3.translation)

        self.current_L_orientation = self.current_L_orientation.slerp(self._speed_factor, pin.Quaternion(L_target_se3.rotation))
        self.current_R_orientation = self.current_R_orientation.slerp(self._speed_factor, pin.Quaternion(R_target_se3.rotation))

        L_tf_interpolated = pin.SE3(self.current_L_orientation.toRotationMatrix(), self.current_L_tf)
        R_tf_interpolated = pin.SE3(self.current_R_orientation.toRotationMatrix(), self.current_R_tf)

        sol_q = self.solve_ik(L_tf_interpolated.homogeneous, R_tf_interpolated.homogeneous)

        # Reduced robot q contains 14-ish joints including wrists; we want 8 (no wrists).
        # Falcon uses indices [0,1,2,3,7,8,9,10] within upper-body (14) to set shoulders+elbows.
        # In reduced robot, joint order is the arm chain; this mapping is consistent with their usage:
        # return the first 4 and last 4 excluding wrists.
        if sol_q.shape[0] >= 14:
            return np.concatenate([sol_q[:4], sol_q[7:11]], axis=0)
        # Fallback: best-effort slice
        return sol_q[:8]


