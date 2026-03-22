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

"""Unit tests for benchmark framework (no Drake required)."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.manipulation.planning.benchmark.metrics import compute_path_smoothness
from dimos.manipulation.planning.benchmark.models import (
    BenchmarkScenario,
    PlannerMetrics,
    PlanningProblem,
)
from dimos.manipulation.planning.benchmark.runner import print_summary
from dimos.manipulation.planning.benchmark.scenarios import (
    ALL_SCENARIOS,
    get_all_scenarios,
    get_scenario,
)
from dimos.msgs.sensor_msgs.JointState import JointState


def _js(positions: list[float]) -> JointState:
    return JointState(name=["j1", "j2", "j3"], position=positions)


class TestPathSmoothness:
    """Test smoothness metric computation."""

    def test_straight_line_is_smooth(self):
        """A linear trajectory should have near-zero acceleration."""
        path = [_js([i * 0.1, i * 0.1, i * 0.1]) for i in range(10)]
        smoothness = compute_path_smoothness(path)
        assert smoothness < 1e-10

    def test_zigzag_is_rough(self):
        """A zigzag trajectory should have high acceleration."""
        path = []
        for i in range(10):
            offset = 0.5 if i % 2 == 0 else -0.5
            path.append(_js([i * 0.1, offset, 0.0]))
        smoothness = compute_path_smoothness(path)
        assert smoothness > 0.1

    def test_short_path_returns_zero(self):
        """Paths with fewer than 3 waypoints have no acceleration."""
        assert compute_path_smoothness([]) == 0.0
        assert compute_path_smoothness([_js([0.0, 0.0, 0.0])]) == 0.0
        assert compute_path_smoothness([_js([0.0, 0.0, 0.0]), _js([1.0, 1.0, 1.0])]) == 0.0


class TestScenarios:
    """Test scenario definitions are valid."""

    def test_all_scenarios_load(self):
        """All scenarios should load without error."""
        scenarios = get_all_scenarios()
        assert len(scenarios) == len(ALL_SCENARIOS)

    def test_scenario_by_name(self):
        """get_scenario should return the correct scenario."""
        scenario = get_scenario("free_space")
        assert scenario.name == "free_space"
        assert len(scenario.problems) > 0

    def test_unknown_scenario_raises(self):
        """get_scenario should raise for unknown names."""
        with pytest.raises(ValueError, match="Unknown scenario"):
            get_scenario("nonexistent")

    def test_scenarios_have_problems(self):
        """All scenarios should have at least one problem."""
        for scenario in get_all_scenarios():
            assert len(scenario.problems) > 0, f"Scenario {scenario.name} has no problems"

    def test_scenario_problems_have_matching_joints(self):
        """Start and goal should have the same joint names."""
        for scenario in get_all_scenarios():
            for problem in scenario.problems:
                assert problem.start.name == problem.goal.name, (
                    f"{scenario.name}/{problem.name}: start/goal joint names mismatch"
                )


class TestPrintSummary:
    """Test that print_summary doesn't crash on various inputs."""

    def test_empty_metrics(self, capsys):
        print_summary([])
        captured = capsys.readouterr()
        assert "No metrics" in captured.out

    def test_with_metrics(self, capsys):
        metrics = [
            PlannerMetrics(
                planner_name="rrt_connect",
                scenario_name="free_space",
                problem_name="test",
                run_index=0,
                success=True,
                planning_time_s=0.05,
                path_length_rad=2.5,
                path_smoothness=0.001,
                n_waypoints=20,
                min_clearance=0.1,
                iterations=100,
                message="ok",
            ),
            PlannerMetrics(
                planner_name="chomp",
                scenario_name="free_space",
                problem_name="test",
                run_index=0,
                success=True,
                planning_time_s=0.1,
                path_length_rad=2.0,
                path_smoothness=0.0001,
                n_waypoints=50,
                min_clearance=0.15,
                iterations=200,
                message="ok",
            ),
        ]
        print_summary(metrics)
        captured = capsys.readouterr()
        assert "rrt_connect" in captured.out
        assert "chomp" in captured.out
        assert "free_space" in captured.out
