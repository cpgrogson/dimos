# Copyright 2025 Dimensional Inc.
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

# lerobot_kinematics.py
# LeRobot-based kinematics wrapper for MuJoCo simulation

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from lerobot.model.kinematics import RobotKinematics
import numpy as np
from scipy.spatial.transform import Rotation as R

if TYPE_CHECKING:
    from numpy.typing import NDArray


class LerobotKinematics:
    """LeRobot-based kinematics wrapper for MuJoCo simulation.

    - Uses lerobot.model.kinematics.RobotKinematics (placo-based) internally.
    - Provides:
        * FK:    q -> (pos, quat_wxyz)  [q in degrees]
        * IK:    (q_init, target_pos, target_quat_wxyz) -> q_sol  [degrees]
        * J:     jacobian(q)  [q in radians, uses numerical differentiation]
        * dls:   joint_velocity(q, twist) -> dq  [q in radians]

    Note: FK/IK work in degrees (matching lerobot convention), while
    jacobian/joint_velocity work in radians for compatibility with existing code.
    """

    def __init__(
        self,
        urdf_path: str | Path,
        ee_link_name: str,
        *,
        max_iters: int = 100,
        tol: float = 1e-4,
        damping: float = 1e-3,
        joint_names: list[str] | None = None,
    ) -> None:
        """
        Parameters
        ----------
        urdf_path:
            Path to the URDF file.
        ee_link_name:
            Name of the end-effector link as defined in the URDF.
        max_iters:
            Maximum number of iterations for iterative IK (not used by lerobot, kept for compatibility).
        tol:
            Pose error tolerance (not used by lerobot, kept for compatibility).
        damping:
            Damping λ for damped least-squares IK / velocity control.
        joint_names:
            Optional list of joint names. If None, will be inferred from URDF.
        """
        urdf_path = Path(urdf_path)

        self._lerobot_kin = RobotKinematics(
            urdf_path=str(urdf_path),
            target_frame_name=ee_link_name,
            joint_names=joint_names,
        )

        self.joint_names: list[str] = self._lerobot_kin.joint_names
        self.motor_names: list[str] = self.joint_names
        self.dof: int = len(self.joint_names)

        self._max_iters = int(max_iters)
        self._tol = float(tol)
        self._damping = float(damping)

        self._jacobian_eps = 1e-6

    # ------------------------------------------------------------------
    # Forward Kinematics
    # ------------------------------------------------------------------
    def fk(self, q: NDArray[np.float64]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """
        Forward kinematics: joint vector -> (position, quaternion_wxyz).

        Parameters
        ----------
        q:
            Joint configuration in degrees, shape (dof,).

        Returns
        -------
        pos:
            End-effector position in meters, shape (3,).
        quat_wxyz:
            End-effector orientation quaternion (w, x, y, z), shape (4,).
        """
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.shape[0] != self.dof:
            raise ValueError(f"Expected {self.dof} DoF, got {q.shape[0]}")
        T = self._lerobot_kin.forward_kinematics(q)

        pos = T[:3, 3]
        R_matrix = T[:3, :3]
        rot = R.from_matrix(R_matrix)
        quat_xyzw = rot.as_quat()
        quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]], dtype=float)
        return pos, quat_wxyz

    # ------------------------------------------------------------------
    # Inverse Kinematics
    # ------------------------------------------------------------------
    def ik(
        self,
        q_init: NDArray[np.float64],
        target_pos: NDArray[np.float64],
        target_quat_wxyz: NDArray[np.float64],
        position_weight: float = 1.0,
        orientation_weight: float = 1.0,
    ) -> NDArray[np.float64]:
        """
        Inverse kinematics using Jacobian-based iterative solver.

        This uses damped least-squares (DLS) iteration to find joint angles
        that achieve the target pose.

        Parameters
        ----------
        q_init:
            Initial joint configuration in degrees, shape (dof,).
        target_pos:
            Target position in world frame (meters), shape (3,).
        target_quat_wxyz:
            Target orientation quaternion (w, x, y, z), shape (4,).
        position_weight:
            Weight for position error (default 1.0).
        orientation_weight:
            Weight for orientation error (default 1.0).

        Returns
        -------
        q_sol:
            Joint vector in degrees, shape (dof,), in the same order as `joint_names`.
        """
        q_init = np.asarray(q_init, dtype=float).reshape(-1)
        if q_init.shape[0] != self.dof:
            raise ValueError(f"Expected {self.dof} DoF, got {q_init.shape[0]}")
        target_pos = np.asarray(target_pos, dtype=float).reshape(3)
        target_quat_wxyz = np.asarray(target_quat_wxyz, dtype=float).reshape(4)

        target_quat_xyzw = np.array(
            [target_quat_wxyz[1], target_quat_wxyz[2], target_quat_wxyz[3], target_quat_wxyz[0]],
            dtype=float,
        )
        target_rot = R.from_quat(target_quat_xyzw)

        q = np.radians(q_init.copy())

        # Iterative IK using Jacobian
        step_size = 0.5  # Damping factor for stability

        for _ in range(self._max_iters):
            q_deg = np.degrees(q)
            current_pos, current_quat_wxyz = self.fk(q_deg)

            pos_error = target_pos - current_pos
            pos_error_norm = np.linalg.norm(pos_error)

            # Orientation error (axis-angle representation)
            current_quat_xyzw = np.array(
                [
                    current_quat_wxyz[1],
                    current_quat_wxyz[2],
                    current_quat_wxyz[3],
                    current_quat_wxyz[0],
                ],
                dtype=float,
            )
            current_rot = R.from_quat(current_quat_xyzw)

            rot_error = target_rot * current_rot.inv()

            # --- 5DoF orientation projection ---
            w_world = rot_error.as_rotvec()
            w_tool = current_rot.inv().apply(w_world)
            w_tool[0] = 0.0  # block tool-x; keep tool-y + tool-z
            orientation_error = current_rot.apply(w_tool)

            orientation_error_norm = np.linalg.norm(orientation_error)

            if pos_error_norm < self._tol and orientation_error_norm < self._tol:
                break

            twist = np.concatenate(
                [position_weight * pos_error, orientation_weight * orientation_error]
            )
            J = self.jacobian(q)

            JJt = J @ J.T
            A = JJt + (self._damping**2) * np.eye(6, dtype=float)
            dq_active = J.T @ np.linalg.solve(A, twist)

            q = q + step_size * dq_active

        # Safety check: verify the solution accuracy
        q_sol_deg = np.degrees(q)
        final_pos, _ = self.fk(q_sol_deg)
        final_pos_error = np.linalg.norm(target_pos - final_pos)

        if final_pos_error > 0.1:  # 10cm threshold
            raise ValueError(
                "IK solver unable to find satisfactory solution. "
                f"Final position error: {final_pos_error * 100:.2f}cm (threshold: 10cm). "
                f"Target position: {target_pos}, Final position: {final_pos}"
            )

        return q_sol_deg

    def ik_lerobot(
        self,
        q_init: NDArray[np.float64],
        target_pos: NDArray[np.float64],
        target_quat_wxyz: NDArray[np.float64],
        position_weight: float = 50.0,
        orientation_weight: float = 1.0,
    ) -> NDArray[np.float64]:
        """
        Lerobot SDK IK solver.

        Note: This solver had issues with position accuracy ~ sanjaypokkali.
        """
        q_init = np.asarray(q_init, dtype=float).reshape(-1)
        target_pos = np.asarray(target_pos, dtype=float).reshape(3)
        target_quat_wxyz = np.asarray(target_quat_wxyz, dtype=float).reshape(4)

        # Convert quaternion (w, x, y, z) to rotation matrix
        quat_xyzw = np.array(
            [target_quat_wxyz[1], target_quat_wxyz[2], target_quat_wxyz[3], target_quat_wxyz[0]],
            dtype=float,
        )
        rot = R.from_quat(quat_xyzw)
        R_matrix = rot.as_matrix()

        T = np.eye(4, dtype=float)
        T[:3, :3] = R_matrix
        T[:3, 3] = target_pos

        q_sol = self._lerobot_kin.inverse_kinematics(
            current_joint_pos=q_init,
            desired_ee_pose=T,
            position_weight=position_weight,
            orientation_weight=orientation_weight,
        )
        return np.asarray(q_sol, dtype=float)

    # ------------------------------------------------------------------
    # Jacobian & velocity-level control
    # ------------------------------------------------------------------
    def jacobian(self, q: NDArray[np.float64]) -> NDArray[np.float64]:
        """
        Return the 6 x dof geometric Jacobian at configuration q.

        Uses numerical differentiation since lerobot doesn't provide analytical jacobian.

        Parameters
        ----------
        q:
            Joint configuration in radians, shape (dof,).

        Returns
        -------
        J:
            Geometric Jacobian matrix, shape (6, dof).
            First 3 rows: linear velocity (vx, vy, vz)
            Last 3 rows: angular velocity (wx, wy, wz)
        """
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.shape[0] != self.dof:
            raise ValueError(f"Expected {self.dof} DoF, got {q.shape[0]}")
        q_deg = np.degrees(q)

        pos0, quat0_wxyz = self.fk(q_deg)
        quat0_xyzw = np.array([quat0_wxyz[1], quat0_wxyz[2], quat0_wxyz[3], quat0_wxyz[0]])
        rot0 = R.from_quat(quat0_xyzw)
        R0 = rot0.as_matrix()

        J = np.zeros((6, self.dof), dtype=float)
        eps_rad = self._jacobian_eps

        for i in range(self.dof):
            q_pert_deg = q_deg.copy()
            q_pert_deg[i] += np.degrees(eps_rad)

            pos1, quat1_wxyz = self.fk(q_pert_deg)
            J[:3, i] = (pos1 - pos0) / eps_rad

            quat1_xyzw = np.array([quat1_wxyz[1], quat1_wxyz[2], quat1_wxyz[3], quat1_wxyz[0]])
            rot1 = R.from_quat(quat1_xyzw)
            R1 = rot1.as_matrix()

            dR = R1 @ R0.T
            skew = (dR - dR.T) / (2.0 * eps_rad)
            J[3, i] = skew[2, 1]
            J[4, i] = skew[0, 2]
            J[5, i] = skew[1, 0]

        return J

    def joint_velocity(
        self,
        q: NDArray[np.float64],
        twist: NDArray[np.float64],
        active_mask: NDArray[np.bool_] | None = None,
    ) -> NDArray[np.float64]:
        """
        Compute joint velocities dq for a desired 6D twist using DLS.

        Parameters
        ----------
        q:
            Joint configuration in radians, shape (dof,).
        twist:
            6D end-effector twist [vx, vy, vz, wx, wy, wz] (same convention
            as `jacobian(q)`).
        active_mask:
            Optional boolean mask for which joints can move.

        Returns
        -------
        dq:
            Joint velocities in rad/s, shape (dof,).
        """
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.shape[0] != self.dof:
            raise ValueError(f"Expected {self.dof} DoF, got {q.shape[0]}")
        twist = np.asarray(twist, dtype=float).reshape(6)

        if active_mask is None:
            mask = np.ones(self.dof, dtype=bool)
        else:
            mask = np.asarray(active_mask, dtype=bool).reshape(-1)
            if mask.shape[0] != self.dof:
                raise ValueError(f"active_mask must have length {self.dof}, got {mask.shape[0]}")

        # Compute jacobian (works in radians)
        J_full = self.jacobian(q)  # 6 x dof
        J = J_full[:, mask]  # 6 x n_active

        JT = J.T
        JJt = J @ JT
        A = JJt + (self._damping**2) * np.eye(6, dtype=float)

        dq_active = JT @ np.linalg.solve(A, twist)

        dq = np.zeros_like(q)
        dq[mask] = dq_active
        return dq
