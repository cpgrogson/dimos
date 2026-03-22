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

"""Benchmark runner for motion planners."""

from __future__ import annotations

import json
import statistics
from typing import TYPE_CHECKING

from dimos.manipulation.planning.benchmark.metrics import (
    compute_min_clearance,
    compute_path_smoothness,
)
from dimos.manipulation.planning.benchmark.models import (
    BenchmarkScenario,
    PlannerMetrics,
)
from dimos.manipulation.planning.factory import create_world
from dimos.manipulation.planning.utils.path_utils import compute_path_length
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.manipulation.planning.spec.protocols import PlannerSpec

logger = setup_logger()


def run_benchmark(
    planners: dict[str, PlannerSpec],
    scenarios: list[BenchmarkScenario],
    n_runs: int = 50,
    timeout: float = 10.0,
    compute_clearance: bool = True,
) -> list[PlannerMetrics]:
    """Run all planners on all scenarios.

    Args:
        planners: Dict mapping planner name to PlannerSpec instance.
        scenarios: List of benchmark scenarios to run.
        n_runs: Number of runs per (planner, problem) pair for statistical significance.
        timeout: Planning timeout per run (seconds).
        compute_clearance: Whether to compute min clearance (slower but more informative).

    Returns:
        List of PlannerMetrics, one per (planner, scenario, problem, run).
    """
    all_metrics: list[PlannerMetrics] = []

    for scenario in scenarios:
        logger.info("=== Scenario: %s ===", scenario.name)

        # Create world for this scenario
        world = create_world(backend="drake", enable_viz=False)
        robot_id = world.add_robot(scenario.robot_config)
        world.finalize()

        # Add obstacles
        for obs in scenario.obstacles:
            world.add_obstacle(obs)

        for problem in scenario.problems:
            for planner_name, planner in planners.items():
                logger.info(
                    "  %s / %s / %s (%d runs)",
                    scenario.name,
                    problem.name,
                    planner_name,
                    n_runs,
                )

                for run_idx in range(n_runs):
                    result = planner.plan_joint_path(
                        world, robot_id, problem.start, problem.goal, timeout=timeout
                    )

                    # Compute additional metrics for successful plans
                    smoothness = 0.0
                    clearance = float("inf")
                    if result.is_success() and result.path:
                        smoothness = compute_path_smoothness(result.path)
                        if compute_clearance:
                            clearance = compute_min_clearance(
                                world, robot_id, result.path, n_samples=50
                            )

                    metrics = PlannerMetrics(
                        planner_name=planner_name,
                        scenario_name=scenario.name,
                        problem_name=problem.name,
                        run_index=run_idx,
                        success=result.is_success(),
                        planning_time_s=result.planning_time,
                        path_length_rad=result.path_length,
                        path_smoothness=smoothness,
                        n_waypoints=len(result.path),
                        min_clearance=clearance,
                        iterations=result.iterations,
                        message=result.message,
                    )
                    all_metrics.append(metrics)

        # Clean up world
        world.close()

    return all_metrics


def print_summary(metrics: list[PlannerMetrics]) -> None:
    """Print a summary table of benchmark results.

    Groups by (scenario, planner) and reports median planning time,
    success rate, median path length, and median smoothness.
    """
    if not metrics:
        print("No metrics to report.")
        return

    # Group by (scenario, planner)
    groups: dict[tuple[str, str], list[PlannerMetrics]] = {}
    for m in metrics:
        key = (m.scenario_name, m.planner_name)
        groups.setdefault(key, []).append(m)

    # Header
    print()
    print(
        f"{'Scenario':<20} {'Planner':<15} {'Success':>8} "
        f"{'Time(ms)':>10} {'Length':>8} {'Smooth':>10} {'Clearance':>10}"
    )
    print("-" * 91)

    current_scenario = ""
    for (scenario, planner), group in sorted(groups.items()):
        successes = [m for m in group if m.success]
        success_rate = len(successes) / len(group)

        # Compute medians for successful runs
        if successes:
            med_time = statistics.median(m.planning_time_s for m in successes) * 1000
            med_length = statistics.median(m.path_length_rad for m in successes)
            med_smooth = statistics.median(m.path_smoothness for m in successes)
            clearances = [m.min_clearance for m in successes if m.min_clearance < float("inf")]
            med_clearance = statistics.median(clearances) if clearances else float("inf")
        else:
            med_time = med_length = med_smooth = 0.0
            med_clearance = 0.0

        # Visual separator between scenarios
        scenario_label = scenario if scenario != current_scenario else ""
        current_scenario = scenario

        clearance_str = f"{med_clearance:>10.4f}" if med_clearance < float("inf") else "       inf"

        print(
            f"{scenario_label:<20} {planner:<15} {success_rate:>7.0%} "
            f"{med_time:>9.1f} {med_length:>8.3f} {med_smooth:>10.6f} {clearance_str}"
        )

    print()


def export_json(metrics: list[PlannerMetrics], path: str) -> None:
    """Export metrics to a JSON file.

    Args:
        metrics: List of PlannerMetrics to export.
        path: Output file path.
    """
    data = []
    for m in metrics:
        data.append(
            {
                "planner": m.planner_name,
                "scenario": m.scenario_name,
                "problem": m.problem_name,
                "run": m.run_index,
                "success": m.success,
                "planning_time_s": m.planning_time_s,
                "path_length_rad": m.path_length_rad,
                "path_smoothness": m.path_smoothness,
                "n_waypoints": m.n_waypoints,
                "min_clearance": m.min_clearance if m.min_clearance < float("inf") else None,
                "iterations": m.iterations,
                "message": m.message,
            }
        )

    with open(path, "w") as f:
        json.dump(data, f, indent=2)

    logger.info("Exported %d results to %s", len(data), path)
