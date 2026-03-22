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

"""Data models for planner benchmarking."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dimos.manipulation.planning.spec.config import RobotModelConfig
    from dimos.manipulation.planning.spec.models import Obstacle
    from dimos.msgs.sensor_msgs.JointState import JointState


@dataclass
class PlanningProblem:
    """A single start-goal planning query."""

    name: str
    start: JointState
    goal: JointState


@dataclass
class BenchmarkScenario:
    """A benchmark scenario: robot + obstacles + set of problems."""

    name: str
    description: str
    robot_config: RobotModelConfig
    obstacles: list[Obstacle] = field(default_factory=list)
    problems: list[PlanningProblem] = field(default_factory=list)


@dataclass
class PlannerMetrics:
    """Metrics from a single planning run."""

    planner_name: str
    scenario_name: str
    problem_name: str
    run_index: int
    success: bool
    planning_time_s: float
    path_length_rad: float
    path_smoothness: float
    n_waypoints: int
    min_clearance: float
    iterations: int
    message: str
