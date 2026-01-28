import numpy as np
import pinocchio as pin

from .robot_arm_ik import G1_29_ArmIK
from .weighted_moving_filter import WeightedMovingFilter


class G1_29_ArmIK_NoWrists(G1_29_ArmIK):  # noqa: N801
    """Vendored from FALCON/sim2real/utils/arm_ik/robot_arm_ik_g1_23dof.py (visualization removed)."""

    def __init__(self, Unit_Test=False, Visualization=False, robot_config=None):  # noqa: N803
        super().__init__(Unit_Test=Unit_Test, Visualization=Visualization, robot_config=robot_config)

        # Lock wrist joints (no wrists)
        self.mixed_jointsToLockIDs = self.mixed_jointsToLockIDs + [
            "left_wrist_pitch_joint",
            "left_wrist_roll_joint",
            "left_wrist_yaw_joint",
            "right_wrist_pitch_joint",
            "right_wrist_roll_joint",
            "right_wrist_yaw_joint",
        ]

        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0.0] * self.robot.model.nq),
        )

        # Override EE frames to elbow (matches Falcon no-wrists class)
        self.reduced_robot.model.addFrame(
            pin.Frame(
                "L_ee",
                self.reduced_robot.model.getJointId("left_elbow_joint"),
                pin.SE3(np.eye(3), np.array([0.35, -0.075, 0]).T),
                pin.FrameType.OP_FRAME,
            )
        )
        self.reduced_robot.model.addFrame(
            pin.Frame(
                "R_ee",
                self.reduced_robot.model.getJointId("right_elbow_joint"),
                pin.SE3(np.eye(3), np.array([0.35, 0.075, 0]).T),
                pin.FrameType.OP_FRAME,
            )
        )

        # Rebuild collision geometry for reduced robot
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

        self.nq = int(self.reduced_robot.model.nq)
        self.nv = int(self.reduced_robot.model.nv)
        self.init_data = np.zeros(self.nq)
        self.smooth_filter = WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), self.nq)

        # Frame ids
        self.L_hand_id = self.reduced_robot.model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_robot.model.getFrameId("R_ee")


