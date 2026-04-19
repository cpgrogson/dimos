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

"""E2E integration test: cross-wall planning through DimSim.

Uses the DimSim browser-based sim (headless, auto-detects GPU) with the
Go2 nav blueprint. Tests progressive navigation complexity and measures
movement smoothness.

Phases:
  1. Open corridor — straight drive, no obstacles.
  2. Furniture — navigate around dining table/chairs.
  3. Wall explore — incremental steps toward interior wall.
  4. Kitchen entry — go around to the other side of the wall.
  5. Cross-wall — goal behind wall, planner must use doorway.

Smoothness metrics per waypoint:
  - path_efficiency: straight-line / actual distance (1.0 = perfect)
  - direction_reversals: number of times robot reversed heading (churn)
  - heading_variance: std dev of heading changes in deg (low = smooth)
  - progress_rate: fraction of samples where robot got closer to goal
  - cmd_vel_hz: planner cmd_vel rate (too high = churning)

Run::

    DIMSIM_LOCAL=1 pytest dimos/e2e_tests/test_dimsim_cross_wall.py -v -s -m slow -o "addopts="
"""

from __future__ import annotations

import math
import os
import signal
import socket
import subprocess
import threading
import time
from dataclasses import dataclass, field

import lcm as lcmlib
import pytest

os.environ.setdefault("DISPLAY", ":1")

ODOM_TOPIC = "/odom#geometry_msgs.PoseStamped"
GOAL_TOPIC = "/clicked_point#geometry_msgs.PointStamped"
CMD_VEL_TOPIC = "/cmd_vel#geometry_msgs.Twist"
BRIDGE_PORT = 8090

# Robot spawns at ~(3.0, 2.0) in the living room.
# Waypoints step incrementally so the planner always has map coverage.
WAYPOINTS = [
    # (name, x, y, z, timeout_sec, reach_threshold_m)
    ("p0_open", 5.5, 2.0, 0.0, 45, 2.0),
    ("p1_toward_dining", 3.0, 0.5, 0.0, 60, 2.0),
    ("p2_dining", 1.5, 0.5, 0.0, 60, 2.0),
    ("p3_hallway", 0.0, 0.5, 0.0, 60, 2.0),
    ("p4_near_wall", -1.0, 0.0, 0.0, 60, 2.0),
    ("p5_kitchen_entry", -1.0, -2.5, 0.0, 90, 2.0),
    ("p6_cross_wall", -3.0, -2.0, 0.0, 120, 2.5),
]

WARMUP_SEC = 20.0
ODOM_WAIT_SEC = 180.0


# -- Smoothness metrics -------------------------------------------------------

@dataclass
class WaypointMetrics:
    """Smoothness metrics for a single waypoint navigation."""
    name: str
    reached: bool
    elapsed_sec: float
    start_dist: float
    final_dist: float

    # Path efficiency: straight-line distance / actual distance traveled
    # 1.0 = perfectly straight, lower = more wandering
    path_efficiency: float = 0.0

    # Number of times the robot reversed heading by > 90°
    direction_reversals: int = 0

    # Std dev of heading changes (degrees). Low = smooth turns.
    heading_variance_deg: float = 0.0

    # Fraction of position samples where robot got closer to goal
    progress_rate: float = 0.0

    # cmd_vel messages per second during this waypoint
    cmd_vel_hz: float = 0.0

    # Raw trajectory for debugging
    trajectory: list[tuple[float, float, float]] = field(default_factory=list)


def _compute_metrics(
    name: str,
    reached: bool,
    elapsed: float,
    start_dist: float,
    final_dist: float,
    trajectory: list[tuple[float, float, float]],
    goal_x: float,
    goal_y: float,
    cmd_vel_during: int,
) -> WaypointMetrics:
    """Compute smoothness metrics from a recorded trajectory.

    trajectory: list of (x, y, timestamp) samples.
    """
    m = WaypointMetrics(
        name=name,
        reached=reached,
        elapsed_sec=elapsed,
        start_dist=start_dist,
        final_dist=final_dist,
        trajectory=trajectory,
    )

    if len(trajectory) < 2:
        return m

    # Actual distance traveled (sum of segments)
    actual_dist = 0.0
    for i in range(1, len(trajectory)):
        dx = trajectory[i][0] - trajectory[i - 1][0]
        dy = trajectory[i][1] - trajectory[i - 1][1]
        actual_dist += math.sqrt(dx * dx + dy * dy)

    # Straight-line from start to final position
    straight_dist = math.sqrt(
        (trajectory[-1][0] - trajectory[0][0]) ** 2
        + (trajectory[-1][1] - trajectory[0][1]) ** 2
    )

    m.path_efficiency = straight_dist / actual_dist if actual_dist > 0.01 else 1.0

    # Heading changes
    headings = []
    for i in range(1, len(trajectory)):
        dx = trajectory[i][0] - trajectory[i - 1][0]
        dy = trajectory[i][1] - trajectory[i - 1][1]
        if dx * dx + dy * dy > 0.001:  # skip stationary samples
            headings.append(math.atan2(dy, dx))

    if len(headings) >= 2:
        heading_changes = []
        reversals = 0
        for i in range(1, len(headings)):
            delta = headings[i] - headings[i - 1]
            # Normalize to [-pi, pi]
            while delta > math.pi:
                delta -= 2 * math.pi
            while delta < -math.pi:
                delta += 2 * math.pi
            heading_changes.append(abs(delta))
            if abs(delta) > math.pi / 2:
                reversals += 1

        m.direction_reversals = reversals
        if heading_changes:
            mean_hc = sum(heading_changes) / len(heading_changes)
            variance = sum((h - mean_hc) ** 2 for h in heading_changes) / len(heading_changes)
            m.heading_variance_deg = math.degrees(math.sqrt(variance))

    # Progress rate: fraction of samples getting closer to goal
    progress_count = 0
    for i in range(1, len(trajectory)):
        prev_dist = math.sqrt(
            (trajectory[i - 1][0] - goal_x) ** 2 + (trajectory[i - 1][1] - goal_y) ** 2
        )
        curr_dist = math.sqrt(
            (trajectory[i][0] - goal_x) ** 2 + (trajectory[i][1] - goal_y) ** 2
        )
        if curr_dist < prev_dist:
            progress_count += 1
    m.progress_rate = progress_count / (len(trajectory) - 1)

    # cmd_vel rate
    m.cmd_vel_hz = cmd_vel_during / elapsed if elapsed > 0 else 0

    return m


def _print_metrics(m: WaypointMetrics) -> None:
    status = "✓" if m.reached else "✗"
    print(f"\n[metrics] {status} {m.name}:")
    print(f"  reached:              {m.reached} ({m.elapsed_sec:.1f}s)")
    print(f"  distance:             {m.start_dist:.2f}m → {m.final_dist:.2f}m")
    print(f"  path_efficiency:      {m.path_efficiency:.3f} (1.0 = straight line)")
    print(f"  direction_reversals:  {m.direction_reversals}")
    print(f"  heading_variance:     {m.heading_variance_deg:.1f}°")
    print(f"  progress_rate:        {m.progress_rate:.1%} (samples getting closer)")
    print(f"  cmd_vel_hz:           {m.cmd_vel_hz:.0f} Hz")


def _print_summary(all_metrics: list[WaypointMetrics]) -> None:
    reached = [m for m in all_metrics if m.reached]
    print(f"\n{'=' * 60}")
    print(f"[summary] {len(reached)}/{len(all_metrics)} waypoints reached")
    if reached:
        avg_eff = sum(m.path_efficiency for m in reached) / len(reached)
        total_reversals = sum(m.direction_reversals for m in reached)
        avg_progress = sum(m.progress_rate for m in reached) / len(reached)
        avg_heading_var = sum(m.heading_variance_deg for m in reached) / len(reached)
        avg_cmd_hz = sum(m.cmd_vel_hz for m in reached) / len(reached)
        print(f"  avg path_efficiency:     {avg_eff:.3f}")
        print(f"  total direction_reversals: {total_reversals}")
        print(f"  avg heading_variance:    {avg_heading_var:.1f}°")
        print(f"  avg progress_rate:       {avg_progress:.1%}")
        print(f"  avg cmd_vel_hz:          {avg_cmd_hz:.0f} Hz")
    print(f"{'=' * 60}\n")


# -- Helpers -------------------------------------------------------------------

def _distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def _force_kill_port(port: int) -> None:
    """Kill processes on the given port. SIGTERM first, SIGKILL after 3s."""
    try:
        result = subprocess.run(
            ["lsof", "-ti", f":{port}"],
            capture_output=True, text=True, timeout=5,
        )
        pids = [int(p) for p in result.stdout.strip().split() if p]
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
            except (ProcessLookupError, ValueError):
                pass
        if pids:
            time.sleep(3)
            for pid in pids:
                try:
                    os.kill(pid, signal.SIGKILL)
                except (ProcessLookupError, ValueError):
                    pass
    except Exception:
        pass


def _wait_for_port_free(port: int, timeout: float = 30) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection(("localhost", port), timeout=1):
                time.sleep(1)
        except OSError:
            return True
    return False


def _kill_headless_chrome() -> None:
    """Kill leftover headless Chrome. SIGTERM first, SIGKILL after 3s."""
    try:
        result = subprocess.run(
            ["pgrep", "-f", "chrome.*headless.*playwright"],
            capture_output=True, text=True, timeout=5,
        )
        pids = [int(p) for p in result.stdout.strip().split() if p]
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
            except (ProcessLookupError, ValueError):
                pass
        if pids:
            time.sleep(3)
            for pid in pids:
                try:
                    os.kill(pid, signal.SIGKILL)
                except (ProcessLookupError, ValueError):
                    pass
    except Exception:
        pass


pytestmark = [pytest.mark.slow]


class TestDimSimCrossWall:
    """E2E integration test: cross-wall routing with smoothness metrics."""

    def test_cross_wall_sequence(self) -> None:
        from dimos.core.coordination.module_coordinator import ModuleCoordinator
        from dimos.msgs.geometry_msgs.PointStamped import PointStamped
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
        from dimos.msgs.geometry_msgs.Twist import Twist
        from dimos.msgs.geometry_msgs.Vector3 import Vector3
        from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_dimsim import (
            unitree_go2_dimsim,
        )

        # -- Cleanup -----------------------------------------------------------
        _force_kill_port(BRIDGE_PORT)
        _kill_headless_chrome()
        assert _wait_for_port_free(BRIDGE_PORT, timeout=10)

        # -- Build blueprint ---------------------------------------------------
        coordinator = ModuleCoordinator.build(unitree_go2_dimsim)

        # -- Odom + diagnostic tracking ----------------------------------------
        lock = threading.Lock()
        odom_count = 0
        robot_x = 0.0
        robot_y = 0.0
        cmd_vel_count = 0
        costmap_count = 0

        lcm_url = os.environ.get(
            "LCM_DEFAULT_URL", "udpm://239.255.76.67:7667?ttl=0"
        )
        lc = lcmlib.LCM(lcm_url)
        lc_pub = lcmlib.LCM(lcm_url)

        def _odom_handler(channel: str, data: bytes) -> None:
            nonlocal odom_count, robot_x, robot_y
            msg = PoseStamped.lcm_decode(data)
            with lock:
                odom_count += 1
                robot_x = msg.x
                robot_y = msg.y

        def _cmd_vel_handler(channel: str, data: bytes) -> None:
            nonlocal cmd_vel_count
            with lock:
                cmd_vel_count += 1

        def _costmap_handler(channel: str, data: bytes) -> None:
            nonlocal costmap_count
            with lock:
                costmap_count += 1

        lc.subscribe(ODOM_TOPIC, _odom_handler)
        lc.subscribe("/cmd_vel#geometry_msgs.Twist", _cmd_vel_handler)
        lc.subscribe("/global_costmap#nav_msgs.OccupancyGrid", _costmap_handler)

        lcm_running = True

        def _lcm_loop() -> None:
            while lcm_running:
                try:
                    lc.handle_timeout(100)
                except Exception:
                    pass

        lcm_thread = threading.Thread(target=_lcm_loop, daemon=True)
        lcm_thread.start()

        all_metrics: list[WaypointMetrics] = []

        try:
            print("[test] Blueprint started, waiting for odom…")

            deadline = time.monotonic() + ODOM_WAIT_SEC
            while time.monotonic() < deadline:
                with lock:
                    if odom_count > 0:
                        break
                time.sleep(0.5)

            with lock:
                assert odom_count > 0, (
                    f"No odometry received after {ODOM_WAIT_SEC}s"
                )

            print(f"[test] Odom online. Robot at ({robot_x:.2f}, {robot_y:.2f})")

            # -- Warmup: drive around to build map -----------------------------
            print("[test] Building map by driving robot…")

            def _drive(linear_x: float, angular_z: float, duration: float) -> None:
                twist = Twist(
                    linear=Vector3(linear_x, 0.0, 0.0),
                    angular=Vector3(0.0, 0.0, angular_z),
                )
                t_end = time.monotonic() + duration
                while time.monotonic() < t_end:
                    lc_pub.publish(CMD_VEL_TOPIC, twist.lcm_encode())
                    time.sleep(0.05)
                stop = Twist(
                    linear=Vector3(0.0, 0.0, 0.0),
                    angular=Vector3(0.0, 0.0, 0.0),
                )
                lc_pub.publish(CMD_VEL_TOPIC, stop.lcm_encode())

            _drive(0.5, 0.0, 3.0)
            _drive(0.0, 0.8, 2.0)
            _drive(0.5, 0.0, 3.0)
            _drive(0.0, 0.8, 2.0)
            _drive(0.5, 0.0, 3.0)
            _drive(0.0, 0.8, 2.0)
            _drive(0.5, 0.0, 3.0)
            time.sleep(3.0)

            with lock:
                print(
                    f"[test] Warmup complete. odom={odom_count}, "
                    f"costmap={costmap_count}, "
                    f"pos=({robot_x:.2f}, {robot_y:.2f})"
                )

            # -- Navigate waypoints with metrics -------------------------------
            for name, gx, gy, gz, timeout_sec, threshold in WAYPOINTS:
                with lock:
                    sx, sy = robot_x, robot_y
                    cv_start = cmd_vel_count

                start_dist = _distance(sx, sy, gx, gy)
                trajectory: list[tuple[float, float, float]] = [(sx, sy, time.monotonic())]

                print(
                    f"\n[test] === {name}: goal ({gx}, {gy}) | "
                    f"robot ({sx:.2f}, {sy:.2f}) | "
                    f"dist={start_dist:.2f}m | budget={timeout_sec}s ==="
                )

                # Publish goal
                goal = PointStamped(
                    x=gx, y=gy, z=gz, ts=time.time(), frame_id="map"
                )
                lc_pub.publish(GOAL_TOPIC, goal.lcm_encode())

                t0 = time.monotonic()
                reached = False
                last_print = t0
                last_goal_pub = t0

                while True:
                    with lock:
                        cx, cy = robot_x, robot_y

                    trajectory.append((cx, cy, time.monotonic()))
                    dist = _distance(cx, cy, gx, gy)
                    now = time.monotonic()
                    elapsed = now - t0

                    # Re-publish goal every 10s
                    if now - last_goal_pub >= 10.0:
                        goal = PointStamped(
                            x=gx, y=gy, z=gz, ts=time.time(), frame_id="map"
                        )
                        lc_pub.publish(GOAL_TOPIC, goal.lcm_encode())
                        last_goal_pub = now

                    if now - last_print >= 5.0:
                        with lock:
                            cv, cm = cmd_vel_count, costmap_count
                        print(
                            f"[test]   {name}: {elapsed:.0f}s/{timeout_sec}s | "
                            f"pos ({cx:.2f}, {cy:.2f}) | dist={dist:.2f}m | "
                            f"cmd_vel={cv} costmap={cm}"
                        )
                        last_print = now

                    if dist <= threshold:
                        reached = True
                        print(
                            f"[test] ✓ {name}: reached in {elapsed:.1f}s "
                            f"(dist={dist:.2f}m ≤ {threshold}m)"
                        )
                        break

                    if elapsed >= timeout_sec:
                        print(
                            f"[test] ✗ {name}: NOT reached after {elapsed:.1f}s "
                            f"(dist={dist:.2f}m > {threshold}m)"
                        )
                        break

                    time.sleep(0.2)  # sample at 5 Hz for trajectory

                # Compute metrics
                with lock:
                    cv_end = cmd_vel_count
                final_dist = _distance(cx, cy, gx, gy)
                m = _compute_metrics(
                    name=name,
                    reached=reached,
                    elapsed=elapsed,
                    start_dist=start_dist,
                    final_dist=final_dist,
                    trajectory=trajectory,
                    goal_x=gx,
                    goal_y=gy,
                    cmd_vel_during=cv_end - cv_start,
                )
                all_metrics.append(m)
                _print_metrics(m)

                if not reached:
                    # Don't assert — collect metrics for all waypoints
                    print(f"[test] WARNING: {name} not reached, continuing…")

            # -- Print summary -------------------------------------------------
            _print_summary(all_metrics)

            # Assert at least the easy waypoints pass
            reached_names = {m.name for m in all_metrics if m.reached}
            assert "p0_open" in reached_names, "Phase 1 (open corridor) failed"

        finally:
            print("\n[test] Stopping blueprint…")
            lcm_running = False
            lcm_thread.join(timeout=3)
            coordinator.stop()
            _kill_headless_chrome()
            _force_kill_port(BRIDGE_PORT)
            print("[test] Done.")
