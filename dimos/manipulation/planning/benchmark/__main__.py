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

"""CLI entry point for planner benchmarks.

Usage:
    # Run all planners on all scenarios (50 runs each)
    python -m dimos.manipulation.planning.benchmark

    # Quick smoke test (1 run)
    python -m dimos.manipulation.planning.benchmark --runs 1

    # Specific planners and scenario
    python -m dimos.manipulation.planning.benchmark --planners rrt_connect chomp --scenario table_top

    # Export results to JSON
    python -m dimos.manipulation.planning.benchmark --output results.json
"""

from __future__ import annotations

import argparse
import sys

from dimos.manipulation.planning.benchmark.runner import (
    export_json,
    print_summary,
    run_benchmark,
)
from dimos.manipulation.planning.benchmark.scenarios import (
    ALL_SCENARIOS,
    get_all_scenarios,
    get_scenario,
)
from dimos.manipulation.planning.factory import create_planner


AVAILABLE_PLANNERS = ["rrt_connect", "chomp"]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Benchmark motion planners on standardized scenarios."
    )
    parser.add_argument(
        "--planners",
        nargs="+",
        default=AVAILABLE_PLANNERS,
        choices=AVAILABLE_PLANNERS,
        help=f"Planners to benchmark (default: all). Choices: {AVAILABLE_PLANNERS}",
    )
    parser.add_argument(
        "--scenario",
        default=None,
        choices=list(ALL_SCENARIOS.keys()),
        help="Run a specific scenario (default: all)",
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=50,
        help="Number of runs per (planner, problem) pair (default: 50)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Planning timeout per run in seconds (default: 10.0)",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Export results to JSON file",
    )
    parser.add_argument(
        "--no-clearance",
        action="store_true",
        help="Skip min clearance computation (faster)",
    )

    args = parser.parse_args()

    # Create planners
    planners = {}
    for name in args.planners:
        planners[name] = create_planner(name)

    # Get scenarios
    if args.scenario:
        scenarios = [get_scenario(args.scenario)]
    else:
        scenarios = get_all_scenarios()

    print(f"Benchmarking {len(planners)} planner(s) on {len(scenarios)} scenario(s)")
    print(f"  Planners: {', '.join(planners.keys())}")
    print(f"  Scenarios: {', '.join(s.name for s in scenarios)}")
    print(f"  Runs per problem: {args.runs}")
    print(f"  Timeout: {args.timeout}s")
    print()

    # Run benchmarks
    metrics = run_benchmark(
        planners=planners,
        scenarios=scenarios,
        n_runs=args.runs,
        timeout=args.timeout,
        compute_clearance=not args.no_clearance,
    )

    # Print summary
    print_summary(metrics)

    # Export if requested
    if args.output:
        export_json(metrics, args.output)


if __name__ == "__main__":
    main()
