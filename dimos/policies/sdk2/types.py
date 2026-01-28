from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
from numpy.typing import NDArray

from dimos.robot.unitree.sdk2.joints import G1_SDK2_MOTOR_JOINT_NAMES

# Back-compat alias (prefer importing from dimos.robot.unitree.sdk2.joints).
G1_MOTOR_JOINT_NAMES: list[str] = G1_SDK2_MOTOR_JOINT_NAMES


@dataclass
class RobotState:
    """Normalized robot state in *policy joint order*."""

    # Time/telemetry
    t_wall_s: float = 0.0

    # Base signals (body frame unless otherwise noted)
    base_lin_vel: NDArray[np.floating] = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    base_ang_vel: NDArray[np.floating] = field(default_factory=lambda: np.zeros(3, dtype=np.float32))

    # IMU
    imu_quat_wxyz: NDArray[np.floating] = field(
        default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    )
    projected_gravity: NDArray[np.floating] = field(default_factory=lambda: np.array([0.0, 0.0, -1.0], dtype=np.float32))

    # Joint state (policy order)
    q: NDArray[np.floating] = field(default_factory=lambda: np.zeros(0, dtype=np.float32))
    dq: NDArray[np.floating] = field(default_factory=lambda: np.zeros(0, dtype=np.float32))


@dataclass
class CommandContext:
    """Command inputs coming from UI/teleop."""

    # Common locomotion command (forward, lateral, yaw rate)
    cmd_vel: NDArray[np.floating] = field(default_factory=lambda: np.zeros(3, dtype=np.float32))

    # Falcon-style auxiliary commands (optional)
    stand: int = 0  # 0/1
    base_height: float | None = None
    waist_rpy: NDArray[np.floating] = field(default_factory=lambda: np.zeros(3, dtype=np.float32))

    # End-effector targets (world or base frame, adapter-defined). Stored as xyz + yaw.
    ee_left_xyz: NDArray[np.floating] = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    ee_right_xyz: NDArray[np.floating] = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    ee_yaw_deg: float = 0.0

    # Gains scaling (optional)
    kp_scale: float = 1.0

    # Raw JSON blob for adapter-specific fields (kept for forward compat)
    extra: dict[str, Any] = field(default_factory=dict)


@dataclass
class JointTargets:
    """Per-joint targets in policy joint order."""

    q_target: NDArray[np.floating]
    dq_target: NDArray[np.floating] | None = None
    kp: NDArray[np.floating] | None = None
    kd: NDArray[np.floating] | None = None
    tau_ff: NDArray[np.floating] | None = None


