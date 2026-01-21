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

from typing import TYPE_CHECKING

import numpy as np
from numpy.linalg import norm, solve
import pinocchio

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs import Pose


class PinocchioIK:
    def __init__(
        self,
        mjcf_path: str,
        ee_joint_id: int,
        eps: float = 1e-4,
        max_iter: int = 1000,
        dt: float = 1e-1,
        damp: float = 1e-12,
    ):
        self.model = pinocchio.buildModelFromMJCF(mjcf_path)
        self.data = self.model.createData()
        self.ee_joint_id = ee_joint_id
        self.eps = eps
        self.max_iter = max_iter
        self.dt = dt
        self.damp = damp

    def forward_kinematics(self, q: np.ndarray) -> pinocchio.SE3:
        pinocchio.forwardKinematics(self.model, self.data, q)
        return self.data.oMi[self.ee_joint_id]

    def solve_ik(
        self,
        target_pose: pinocchio.SE3,
        q_init: np.ndarray | None = None,
        verbose: bool = False,
    ) -> tuple[np.ndarray | None, bool]:
        if q_init is None:
            q = pinocchio.neutral(self.model)
        else:
            q = q_init.copy()

        if verbose:
            print(f"initial: {q.flatten().tolist()}")

        for _i in range(self.max_iter):
            pinocchio.forwardKinematics(self.model, self.data, q)
            iMd = self.data.oMi[self.ee_joint_id].actInv(target_pose)

            err = pinocchio.log(iMd).vector
            if norm(err) < self.eps:
                if verbose:
                    print("Convergence achieved!")
                return q, True

            J = pinocchio.computeJointJacobian(self.model, self.data, q, self.ee_joint_id)
            J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)
            v = -J.T.dot(solve(J.dot(J.T) + self.damp * np.eye(6), err))
            q = pinocchio.integrate(self.model, q, v * self.dt)

        if verbose:
            print("Warning: IK did not converge to desired precision")
        return q, False

    def solve_ik_position_rpy(
        self,
        position: np.ndarray,
        rpy_degrees: tuple[float, float, float],
        q_init: np.ndarray | None = None,
        verbose: bool = False,
    ) -> tuple[np.ndarray | None, bool]:
        roll, pitch, yaw = np.radians(rpy_degrees)
        rotation = pinocchio.rpy.rpyToMatrix(roll, pitch, yaw)
        target_pose = pinocchio.SE3(rotation, position)
        return self.solve_ik(target_pose, q_init, verbose)

    def pose_to_se3(self, pose: "Pose") -> pinocchio.SE3:
        """Convert a Pose to pinocchio SE3."""
        position = np.array([pose.x, pose.y, pose.z])
        rotation = pinocchio.rpy.rpyToMatrix(pose.roll, pose.pitch, pose.yaw)
        return pinocchio.SE3(rotation, position)

    def solve_ik_from_pose(
        self,
        goal_pose: "Pose",
        q_current: np.ndarray,
    ) -> tuple[np.ndarray | None, bool]:
        """Solve IK for a goal Pose, using current joint state as warm-start."""
        target_se3 = self.pose_to_se3(goal_pose)
        return self.solve_ik(target_se3, q_init=q_current)


if __name__ == "__main__":
    mjcf_path = (
        "/home/ruthwik/Documents/dimos/dimos/simulation/manipulators/data/xarm6/xarm6_nohand.xml"
    )
    ik_solver = PinocchioIK(mjcf_path, ee_joint_id=6)

    # First solve for initial position
    init_position = np.array([0.4, 0.0, 0.2])
    rpy_degrees = (180, 0, 0)

    q_init, init_success = ik_solver.solve_ik_position_rpy(
        init_position, rpy_degrees, verbose=False
    )
    print(f"Initial IK converged: {init_success}")
    print(f"Initial q (deg): {np.degrees(q_init).flatten().tolist()}")

    # Goal position: 5cm change in x, y, z
    goal_position = np.array([0.45, 0.05, 0.25])

    q_solution, success = ik_solver.solve_ik_position_rpy(
        goal_position, rpy_degrees, q_init=q_init, verbose=True
    )

    if q_solution is not None:
        print(f"\nresult (rad): {q_solution.flatten().tolist()}")
        print(f"result (deg): {np.degrees(q_solution).flatten().tolist()}")

        # Verify with FK
        ee_pose = ik_solver.forward_kinematics(q_solution)
        print(f"\nFK verification - EE position: {ee_pose.translation.T}")
        print(f"FK verification - EE rotation:\n{ee_pose.rotation}")
