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

"""Metric computation for planner benchmarks."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from dimos.manipulation.planning.spec.models import JointPath, WorldRobotID
    from dimos.manipulation.planning.spec.protocols import WorldSpec
    from dimos.msgs.sensor_msgs.JointState import JointState


def compute_path_smoothness(path: JointPath) -> float:
    """Compute path smoothness as mean squared acceleration.

    Lower values indicate smoother paths. Uses finite-difference
    approximation of acceleration at each interior waypoint.

    Args:
        path: List of JointState waypoints.

    Returns:
        Mean squared acceleration (rad/step^2). Returns 0.0 for paths
        with fewer than 3 waypoints.
    """
    if len(path) < 3:
        return 0.0

    total = 0.0
    for i in range(1, len(path) - 1):
        q_prev = np.array(path[i - 1].position, dtype=np.float64)
        q_curr = np.array(path[i].position, dtype=np.float64)
        q_next = np.array(path[i + 1].position, dtype=np.float64)

        accel = q_prev - 2.0 * q_curr + q_next
        total += float(np.dot(accel, accel))

    return total / (len(path) - 2)


def compute_min_clearance(
    world: WorldSpec,
    robot_id: WorldRobotID,
    path: JointPath,
    n_samples: int = 100,
) -> float:
    """Compute minimum clearance (distance to nearest obstacle) along a path.

    Samples points along the path and returns the minimum signed distance.
    Positive values mean collision-free; negative means penetration.

    Args:
        world: World for distance queries.
        robot_id: Robot to check.
        path: Path to evaluate.
        n_samples: Number of points to sample along the path.

    Returns:
        Minimum signed distance in meters. Returns inf for empty paths.
    """
    if not path:
        return float("inf")

    # Sample uniformly along path segments
    min_dist = float("inf")
    total_segments = len(path) - 1
    if total_segments <= 0:
        # Single waypoint — check just that
        with world.scratch_context() as ctx:
            world.set_joint_state(ctx, robot_id, path[0])
            return world.get_min_distance(ctx, robot_id)

    samples_per_segment = max(1, n_samples // total_segments)

    for seg_idx in range(total_segments):
        q_start = np.array(path[seg_idx].position, dtype=np.float64)
        q_end = np.array(path[seg_idx + 1].position, dtype=np.float64)
        joint_names = path[seg_idx].name

        for s in range(samples_per_segment):
            alpha = s / samples_per_segment
            q = q_start + alpha * (q_end - q_start)

            from dimos.msgs.sensor_msgs.JointState import JointState

            state = JointState(name=joint_names, position=q.tolist())
            with world.scratch_context() as ctx:
                world.set_joint_state(ctx, robot_id, state)
                d = world.get_min_distance(ctx, robot_id)
                min_dist = min(min_dist, d)

    # Also check the very last waypoint
    with world.scratch_context() as ctx:
        world.set_joint_state(ctx, robot_id, path[-1])
        d = world.get_min_distance(ctx, robot_id)
        min_dist = min(min_dist, d)

    return min_dist
