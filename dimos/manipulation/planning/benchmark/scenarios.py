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

"""Built-in benchmark scenarios for XArm7.

Provides standardized planning problems for comparing planner performance.
All joint values are in radians and are within XArm7 limits.
"""

from __future__ import annotations

from dimos.manipulation.planning.benchmark.models import (
    BenchmarkScenario,
    PlanningProblem,
)
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState


# XArm7 joint names
_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]


def _js(positions: list[float]) -> JointState:
    """Shorthand for creating JointState with XArm7 joint names."""
    return JointState(name=_JOINT_NAMES, position=positions)


def _pose(x: float, y: float, z: float) -> PoseStamped:
    """Shorthand for creating an axis-aligned pose at (x, y, z)."""
    return PoseStamped(
        position=Vector3(x=x, y=y, z=z),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )


def _box(name: str, x: float, y: float, z: float, w: float, h: float, d: float) -> Obstacle:
    """Create a box obstacle at position (x,y,z) with dimensions (w,h,d)."""
    return Obstacle(
        name=name,
        obstacle_type=ObstacleType.BOX,
        pose=_pose(x, y, z),
        dimensions=(w, h, d),
    )


def get_xarm7_config() -> "RobotModelConfig":
    """Get XArm7 robot config for benchmarking (no gripper, no coordinator)."""
    from pathlib import Path

    from dimos.manipulation.planning.spec.config import RobotModelConfig
    from dimos.utils.data import get_data

    return RobotModelConfig(
        name="arm",
        urdf_path=get_data("xarm_description") / "urdf/xarm_device.urdf.xacro",
        base_pose=_pose(0.0, 0.0, 0.0),
        joint_names=_JOINT_NAMES,
        end_effector_link="link7",
        base_link="link_base",
        package_paths={"xarm_description": get_data("xarm_description")},
        xacro_args={"dof": "7", "limited": "true"},
        auto_convert_meshes=True,
        max_velocity=1.0,
        max_acceleration=2.0,
        home_joints=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )


# ---------------------------------------------------------------------------
# Scenario: Free Space
# ---------------------------------------------------------------------------

def free_space_scenario() -> BenchmarkScenario:
    """No obstacles. Tests pure planning performance and path quality."""
    return BenchmarkScenario(
        name="free_space",
        description="No obstacles — measures baseline planning speed and path smoothness",
        robot_config=get_xarm7_config(),
        obstacles=[],
        problems=[
            PlanningProblem(
                name="home_to_extended",
                start=_js([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                goal=_js([0.0, 0.5, 0.0, 1.0, 0.0, 0.5, 0.0]),
            ),
            PlanningProblem(
                name="left_to_right",
                start=_js([-1.0, 0.3, 0.0, 0.8, 0.0, 0.3, 0.0]),
                goal=_js([1.0, 0.3, 0.0, 0.8, 0.0, 0.3, 0.0]),
            ),
            PlanningProblem(
                name="folded_to_stretched",
                start=_js([0.0, -0.5, 0.0, 2.0, 0.0, -0.5, 0.0]),
                goal=_js([0.0, 1.0, 0.0, 0.3, 0.0, 1.0, 0.0]),
            ),
            PlanningProblem(
                name="diagonal_reach",
                start=_js([-0.5, 0.2, -0.3, 1.2, 0.4, 0.2, -0.3]),
                goal=_js([0.8, 0.8, 0.5, 0.5, -0.4, 0.8, 0.3]),
            ),
            PlanningProblem(
                name="small_displacement",
                start=_js([0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0]),
                goal=_js([0.1, 0.1, 0.1, 0.6, 0.1, 0.1, 0.1]),
            ),
        ],
    )


# ---------------------------------------------------------------------------
# Scenario: Table Top
# ---------------------------------------------------------------------------

def table_top_scenario() -> BenchmarkScenario:
    """Table surface with box obstacles. Simulates pick-and-place tasks."""
    return BenchmarkScenario(
        name="table_top",
        description="Table with 3 box obstacles — simulates tabletop manipulation",
        robot_config=get_xarm7_config(),
        obstacles=[
            # Table surface below arm workspace (z=-0.2 is safe for all configs)
            _box("table", 0.5, 0.0, -0.20, 0.6, 0.02, 0.6),
            # Three small boxes on the table
            _box("box_left", 0.45, 0.15, -0.14, 0.05, 0.08, 0.05),
            _box("box_center", 0.55, 0.0, -0.14, 0.05, 0.08, 0.05),
            _box("box_right", 0.45, -0.15, -0.14, 0.05, 0.08, 0.05),
        ],
        problems=[
            PlanningProblem(
                name="left_to_right",
                start=_js([-1.0, 0.3, 0.0, 0.8, 0.0, 0.3, 0.0]),
                goal=_js([1.0, 0.3, 0.0, 0.8, 0.0, 0.3, 0.0]),
            ),
            PlanningProblem(
                name="home_to_reach",
                start=_js([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                goal=_js([0.0, 0.5, 0.0, 1.0, 0.0, 0.5, 0.0]),
            ),
            PlanningProblem(
                name="reach_across",
                start=_js([-0.5, 0.3, 0.0, 0.8, 0.0, 0.3, 0.0]),
                goal=_js([0.5, 0.3, 0.0, 0.8, 0.0, 0.3, 0.0]),
            ),
        ],
    )


# ---------------------------------------------------------------------------
# Scenario: Shelf Reach
# ---------------------------------------------------------------------------

def shelf_reach_scenario() -> BenchmarkScenario:
    """Bookshelf geometry — reaching into constrained spaces."""
    return BenchmarkScenario(
        name="shelf_reach",
        description="Bookshelf with shelves — reaching into constrained compartments",
        robot_config=get_xarm7_config(),
        obstacles=[
            # Shelf far in front (x=0.7) with wide gap, below workspace
            _box("shelf_back", 0.75, 0.0, -0.10, 0.02, 0.3, 0.5),
            _box("shelf_bottom", 0.6, 0.0, -0.25, 0.3, 0.02, 0.5),
            _box("shelf_top", 0.6, 0.0, 0.05, 0.3, 0.02, 0.5),
            _box("shelf_left", 0.6, 0.30, -0.10, 0.3, 0.3, 0.02),
            _box("shelf_right", 0.6, -0.30, -0.10, 0.3, 0.3, 0.02),
        ],
        problems=[
            PlanningProblem(
                name="home_to_reach",
                start=_js([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                goal=_js([0.0, 0.2, 0.0, 0.6, 0.0, 0.2, 0.0]),
            ),
            PlanningProblem(
                name="left_to_right",
                start=_js([-1.0, 0.3, 0.0, 0.8, 0.0, 0.3, 0.0]),
                goal=_js([1.0, 0.3, 0.0, 0.8, 0.0, 0.3, 0.0]),
            ),
        ],
    )


# ---------------------------------------------------------------------------
# Scenario: Narrow Passage
# ---------------------------------------------------------------------------

def narrow_passage_scenario() -> BenchmarkScenario:
    """Two walls forming a tight gap. Hardest for optimization-based planners."""
    return BenchmarkScenario(
        name="narrow_passage",
        description="Two walls with a narrow gap — tests ability to find tight passages",
        robot_config=get_xarm7_config(),
        obstacles=[
            # Two walls far in front, forming a gap the arm must reach through
            _box("wall_left", 0.5, 0.10, -0.10, 0.2, 0.2, 0.02),
            _box("wall_right", 0.5, -0.10, -0.10, 0.2, 0.2, 0.02),
        ],
        problems=[
            PlanningProblem(
                name="home_to_reach",
                start=_js([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                goal=_js([0.0, 0.3, 0.0, 0.8, 0.0, 0.3, 0.0]),
            ),
            PlanningProblem(
                name="side_to_side",
                start=_js([-1.0, 0.3, 0.0, 0.8, 0.0, 0.3, 0.0]),
                goal=_js([1.0, 0.3, 0.0, 0.8, 0.0, 0.3, 0.0]),
            ),
        ],
    )


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

ALL_SCENARIOS = {
    "free_space": free_space_scenario,
    "table_top": table_top_scenario,
    "shelf_reach": shelf_reach_scenario,
    "narrow_passage": narrow_passage_scenario,
}


def get_scenario(name: str) -> BenchmarkScenario:
    """Get a benchmark scenario by name."""
    if name not in ALL_SCENARIOS:
        available = ", ".join(ALL_SCENARIOS.keys())
        raise ValueError(f"Unknown scenario: {name}. Available: [{available}]")
    return ALL_SCENARIOS[name]()


def get_all_scenarios() -> list[BenchmarkScenario]:
    """Get all benchmark scenarios."""
    return [factory() for factory in ALL_SCENARIOS.values()]
