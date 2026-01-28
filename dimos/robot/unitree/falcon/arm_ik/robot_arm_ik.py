import numpy as np
import pinocchio as pin
from pinocchio import Quaternion

from .weighted_moving_filter import WeightedMovingFilter


class G1_29_ArmIK:  # noqa: N801
    """Pinocchio-only dual-arm IK with optional collision checking.

    This replaces Falcon's Casadi/IPOPT optimization with a damped least-squares
    Jacobian method (warm-started), which is substantially easier to install on macOS
    via pip/uv (no pinocchio.casadi bindings required).
    """

    def __init__(self, Unit_Test=False, Visualization=False, robot_config=None):  # noqa: N803
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        self.Unit_Test = Unit_Test
        self.Visualization = Visualization

        self.robot = pin.RobotWrapper.BuildFromURDF(robot_config["ASSET_FILE"], robot_config["ASSET_ROOT"])

        self.mixed_jointsToLockIDs = [
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
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0.0] * self.robot.model.nq),
        )

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

        # Collision geometry (Pinocchio geometry model)
        self.geom_model = pin.buildGeomFromUrdf(
            self.reduced_robot.model,
            robot_config["ASSET_FILE"],
            pin.GeometryType.COLLISION,
            robot_config["ASSET_ROOT"],
        )
        self.geom_model.addAllCollisionPairs()
        adjacent_pairs = {(self.reduced_robot.model.parents[i], i) for i in range(1, self.reduced_robot.model.njoints)}
        filtered_pairs = []
        for cp in self.geom_model.collisionPairs:
            link1 = self.geom_model.geometryObjects[cp.first].parentJoint
            link2 = self.geom_model.geometryObjects[cp.second].parentJoint
            if (link1, link2) not in adjacent_pairs and (link2, link1) not in adjacent_pairs:
                filtered_pairs.append(cp)
        self.geom_model.collisionPairs[:] = filtered_pairs

        self.data = self.reduced_robot.model.createData()
        self.geom_data = pin.GeometryData(self.geom_model)

        self.L_hand_id = self.reduced_robot.model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_robot.model.getFrameId("R_ee")

        self.current_L_tf = None
        self.current_R_tf = None
        self.current_L_orientation = None
        self.current_R_orientation = None
        self.speed_factor = 0.02

        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.smooth_filter = WeightedMovingFilter(
            np.array([0.4, 0.3, 0.2, 0.1], dtype=np.float64),
            data_size=int(self.reduced_robot.model.nq),
        )

        # IK hyperparameters (tunable)
        self.max_iters = 20
        self.damping = 1e-2
        self.alpha = 0.6
        self.pos_weight = 50.0
        self.rot_weight = 1.0
        self.tol = 1e-3

    def check_self_collision(self, q):
        pin.computeCollisions(self.reduced_robot.model, self.data, self.geom_model, self.geom_data, q, False)
        for k in range(len(self.geom_model.collisionPairs)):
            if self.geom_data.collisionResults[k].isCollision():
                return True
        return False

    def _update_kinematics(self, q: np.ndarray) -> None:
        pin.forwardKinematics(self.reduced_robot.model, self.data, q)
        pin.computeJointJacobians(self.reduced_robot.model, self.data, q)
        pin.updateFramePlacements(self.reduced_robot.model, self.data)

    def _frame_error_local(self, fid: int, T_target: pin.SE3) -> np.ndarray:
        # Error in LOCAL frame: log(T_current^{-1} * T_target)
        T_cur = self.data.oMf[fid]
        dT = T_cur.inverse() * T_target
        return np.asarray(pin.log6(dT).vector, dtype=np.float64).reshape(6)

    def _frame_jacobian_local(self, fid: int, q: np.ndarray) -> np.ndarray:
        J = pin.computeFrameJacobian(self.reduced_robot.model, self.data, q, fid, pin.ReferenceFrame.LOCAL)
        return np.asarray(J, dtype=np.float64)

    def solve_ik(self, left_wrist, right_wrist, current_lr_arm_motor_q=None, collision_check=True):
        q = self.init_data.copy() if current_lr_arm_motor_q is None else np.array(current_lr_arm_motor_q, dtype=np.float64).copy()

        T_L = pin.SE3(left_wrist[:3, :3], left_wrist[:3, 3])
        T_R = pin.SE3(right_wrist[:3, :3], right_wrist[:3, 3])

        for _ in range(int(self.max_iters)):
            self._update_kinematics(q)

            eL = self._frame_error_local(self.L_hand_id, T_L)
            eR = self._frame_error_local(self.R_hand_id, T_R)
            eL_w = np.hstack([self.pos_weight * eL[:3], self.rot_weight * eL[3:]])
            eR_w = np.hstack([self.pos_weight * eR[:3], self.rot_weight * eR[3:]])
            e = np.hstack([eL_w, eR_w])  # (12,)

            if float(np.linalg.norm(e)) < float(self.tol):
                break

            JL = self._frame_jacobian_local(self.L_hand_id, q)
            JR = self._frame_jacobian_local(self.R_hand_id, q)
            JL[:3, :] *= self.pos_weight
            JL[3:, :] *= self.rot_weight
            JR[:3, :] *= self.pos_weight
            JR[3:, :] *= self.rot_weight
            J = np.vstack([JL, JR])  # (12, nq)

            A = J.T @ J + (float(self.damping) ** 2) * np.eye(J.shape[1])
            b = J.T @ e
            dq = np.linalg.solve(A, b)

            q = pin.integrate(self.reduced_robot.model, q, float(self.alpha) * dq)
            q = np.minimum(np.maximum(q, self.reduced_robot.model.lowerPositionLimit), self.reduced_robot.model.upperPositionLimit)

        self.smooth_filter.add_data(q)
        q_smooth = np.asarray(self.smooth_filter.filtered_data, dtype=np.float64).copy()

        if collision_check and self.check_self_collision(q_smooth):
            q_smooth = self.init_data.copy()

        self.init_data = q_smooth.copy()
        sol_tauff = np.zeros(self.reduced_robot.model.nv, dtype=np.float64)
        return q_smooth, sol_tauff

    def set_initial_poses(self, L_tf, R_tf, L_orientation, R_orientation):  # noqa: N803
        self.current_L_tf = L_tf.copy()
        self.current_R_tf = R_tf.copy()
        self.current_L_orientation = Quaternion(L_orientation)
        self.current_R_orientation = Quaternion(R_orientation)

    def get_q_tau(self, L_tf_target, R_tf_target, EE_efrc_L, EE_efrc_R):  # noqa: N803
        self.current_L_tf = (1 - self.speed_factor) * self.current_L_tf + self.speed_factor * L_tf_target.translation
        self.current_R_tf = (1 - self.speed_factor) * self.current_R_tf + self.speed_factor * R_tf_target.translation

        self.current_L_orientation = self.current_L_orientation.slerp(self.speed_factor, Quaternion(L_tf_target.rotation))
        self.current_R_orientation = self.current_R_orientation.slerp(self.speed_factor, Quaternion(R_tf_target.rotation))

        L_tf_interpolated = pin.SE3(self.current_L_orientation.toRotationMatrix(), self.current_L_tf)
        R_tf_interpolated = pin.SE3(self.current_R_orientation.toRotationMatrix(), self.current_R_tf)

        sol_q, sol_tauff = self.solve_ik(L_tf_interpolated.homogeneous, R_tf_interpolated.homogeneous, collision_check=True)
        return sol_q, sol_tauff


