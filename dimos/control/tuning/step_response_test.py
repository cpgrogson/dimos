#!/usr/bin/env python3
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

"""Step-response test for Go2 plant characterisation.

Sends step velocity commands on each channel (vx, vy, wz) independently,
records commanded vs actual velocity from odometry, and saves CSV data
for offline FOPDT model fitting.

Works on real hardware and MuJoCo sim (respects --simulation flag).

Usage:
    python -m dimos.control.tuning.step_response_test [--output step_data.csv]
"""

from __future__ import annotations

import argparse
import csv
import time
from dataclasses import dataclass, field

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Default test amplitudes
LINEAR_AMPLITUDES = [0.2, 0.4, 0.6]  # m/s
ANGULAR_AMPLITUDES = [0.3, 0.6, 0.9]  # rad/s
TRIALS_PER_AMPLITUDE = 3

BASELINE_DURATION = 2.0  # seconds — record at rest
STEP_DURATION = 5.0  # seconds — hold step command
DECAY_DURATION = 3.0  # seconds — record after zero command
SAMPLE_RATE = 0.01  # 100 Hz polling


@dataclass
class OdomSample:
    timestamp: float
    x: float
    y: float
    yaw: float


@dataclass
class StepResponseRecorder:
    """Records odom samples during a step response test."""

    samples: list[OdomSample] = field(default_factory=list)
    _latest: OdomSample | None = None

    def on_odom(self, msg: PoseStamped) -> None:
        sample = OdomSample(
            timestamp=time.perf_counter(),
            x=msg.position.x,
            y=msg.position.y,
            yaw=msg.orientation.euler[2],
        )
        self._latest = sample
        self.samples.append(sample)

    @property
    def latest(self) -> OdomSample | None:
        return self._latest

    def clear(self) -> None:
        self.samples.clear()
        self._latest = None


def _send_twist(pub: LCMTransport, vx: float, vy: float, wz: float) -> None:
    pub.publish(Twist(
        linear=Vector3(x=vx, y=vy, z=0.0),
        angular=Vector3(x=0.0, y=0.0, z=wz),
    ))


def run_single_step_test(
    cmd_pub: LCMTransport,
    recorder: StepResponseRecorder,
    channel: str,
    amplitude: float,
) -> list[dict]:
    """Run one step-response trial.

    Returns list of dicts: {timestamp, channel, cmd_value, actual_value}.
    """
    rows: list[dict] = []
    recorder.clear()

    def vx_vy_wz(amp: float) -> tuple[float, float, float]:
        if channel == "vx":
            return (amp, 0.0, 0.0)
        elif channel == "vy":
            return (0.0, amp, 0.0)
        else:
            return (0.0, 0.0, amp)

    def actual_value(s: OdomSample, prev: OdomSample | None, dt: float) -> float:
        """Estimate actual velocity from consecutive odom samples."""
        if prev is None or dt < 1e-6:
            return 0.0
        if channel == "vx":
            return (s.x - prev.x) / dt
        elif channel == "vy":
            return (s.y - prev.y) / dt
        else:
            # Yaw rate — handle wrapping
            import math
            dyaw = s.yaw - prev.yaw
            dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi
            return dyaw / dt

    # Phase 1: baseline (zero command)
    logger.info(f"  Baseline ({BASELINE_DURATION}s)...")
    t0 = time.perf_counter()
    prev_sample = None
    while time.perf_counter() - t0 < BASELINE_DURATION:
        _send_twist(cmd_pub, 0.0, 0.0, 0.0)
        s = recorder.latest
        if s is not None:
            dt = (s.timestamp - prev_sample.timestamp) if prev_sample else 0.0
            rows.append({
                "timestamp": s.timestamp,
                "channel": channel,
                "cmd_value": 0.0,
                "actual_value": actual_value(s, prev_sample, dt),
            })
            prev_sample = s
        time.sleep(SAMPLE_RATE)

    # Phase 2: step command
    logger.info(f"  Step amplitude={amplitude} ({STEP_DURATION}s)...")
    vx, vy, wz = vx_vy_wz(amplitude)
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < STEP_DURATION:
        _send_twist(cmd_pub, vx, vy, wz)
        s = recorder.latest
        if s is not None:
            dt = (s.timestamp - prev_sample.timestamp) if prev_sample else 0.0
            rows.append({
                "timestamp": s.timestamp,
                "channel": channel,
                "cmd_value": amplitude,
                "actual_value": actual_value(s, prev_sample, dt),
            })
            prev_sample = s
        time.sleep(SAMPLE_RATE)

    # Phase 3: decay (zero command)
    logger.info(f"  Decay ({DECAY_DURATION}s)...")
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < DECAY_DURATION:
        _send_twist(cmd_pub, 0.0, 0.0, 0.0)
        s = recorder.latest
        if s is not None:
            dt = (s.timestamp - prev_sample.timestamp) if prev_sample else 0.0
            rows.append({
                "timestamp": s.timestamp,
                "channel": channel,
                "cmd_value": 0.0,
                "actual_value": actual_value(s, prev_sample, dt),
            })
            prev_sample = s
        time.sleep(SAMPLE_RATE)

    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description="Go2 step-response plant characterisation")
    parser.add_argument("--output", default="step_response_data.csv", help="Output CSV path")
    parser.add_argument("--trials", type=int, default=TRIALS_PER_AMPLITUDE, help="Trials per amplitude")
    args = parser.parse_args()

    # Set up LCM transports matching the unitree_go2_coordinator blueprint.
    # Publish commands to /cmd_vel (coordinator's twist_command input).
    # Subscribe to odom from /go2/odom (GO2Connection's odom output).
    cmd_pub = LCMTransport("/cmd_vel", Twist)
    odom_sub = LCMTransport("/go2/odom", PoseStamped)

    recorder = StepResponseRecorder()
    odom_unsub = odom_sub.subscribe(recorder.on_odom)

    all_rows: list[dict] = []

    try:
        logger.info("Waiting 3s for odom stream to stabilise...")
        time.sleep(3.0)

        if recorder.latest is None:
            logger.error("No odom received — is the Go2 coordinator running?")
            return

        # Linear channels
        for channel in ["vx", "vy"]:
            for amp in LINEAR_AMPLITUDES:
                for trial in range(args.trials):
                    logger.info(f"Channel={channel}, amplitude={amp}, trial={trial + 1}/{args.trials}")
                    rows = run_single_step_test(cmd_pub, recorder, channel, amp)
                    all_rows.extend(rows)
                    time.sleep(1.0)

        # Angular channel
        for amp in ANGULAR_AMPLITUDES:
            for trial in range(args.trials):
                logger.info(f"Channel=wz, amplitude={amp}, trial={trial + 1}/{args.trials}")
                rows = run_single_step_test(cmd_pub, recorder, "wz", amp)
                all_rows.extend(rows)
                time.sleep(1.0)

        # Stop robot
        _send_twist(cmd_pub, 0.0, 0.0, 0.0)

    finally:
        odom_unsub()
        cmd_pub.stop()
        odom_sub.stop()

    # Write CSV
    with open(args.output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["timestamp", "channel", "cmd_value", "actual_value"])
        writer.writeheader()
        writer.writerows(all_rows)

    logger.info(f"Saved {len(all_rows)} samples to {args.output}")


if __name__ == "__main__":
    main()
