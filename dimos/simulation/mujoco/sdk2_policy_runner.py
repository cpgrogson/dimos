#!/usr/bin/env python3
"""SDK2 Policy Runner - runs ONNX policies over SDK2 interface.

This allows mjlab-trained policies to run on both simulation and real robots
using the same SDK2 interface (zero code change deployment).

Usage:
    # Simulation (domain_id=1, interface=lo0 on macOS, lo on Linux)
    python -m dimos.simulation.mujoco.sdk2_policy_runner policy.onnx

    # Real robot (domain_id=0, interface=eth0)
    python -m dimos.simulation.mujoco.sdk2_policy_runner policy.onnx --real eth0
"""

from __future__ import annotations

import argparse
import time
from typing import TYPE_CHECKING

from dimos.policies.sdk2.adapters.mjlab import MjlabVelocityAdapter
from dimos.policies.sdk2.runtime import PolicyRuntime, PolicyRuntimeConfig

if TYPE_CHECKING:  # pragma: no cover
    pass


class SDK2PolicyRunner:
    """Back-compat wrapper around the new SDK2 PolicyRuntime (mjlab adapter)."""

    def __init__(
        self,
        policy_path: str,
        robot_type: str = "g1",
        domain_id: int = 1,
        interface: str = "lo0",
        control_dt: float = 0.02,  # 50 Hz
    ) -> None:
        self.control_dt = float(control_dt)
        self._adapter = MjlabVelocityAdapter(policy_path=policy_path)
        self._rt = PolicyRuntime(
            adapter=self._adapter,
            config=PolicyRuntimeConfig(
                robot_type=robot_type,
                domain_id=domain_id,
                interface=interface,
                control_dt=float(control_dt),
                mode_pr=0,
            ),
        )

    def set_command(self, vx: float, vy: float, wz: float) -> None:
        self._rt.set_cmd_vel(vx, vy, wz)

    def set_enabled(self, enabled: bool) -> None:
        self._rt.set_enabled(enabled)
        print(f"[SDK2PolicyRunner] enabled={bool(enabled)}")

    def set_estop(self, estop: bool) -> None:
        self._rt.set_estop(estop)
        print(f"[SDK2PolicyRunner] estop={bool(estop)}")

    def set_policy_params_json(self, params_json: str) -> None:
        self._rt.set_policy_params_json(params_json)

    def step(self) -> None:
        self._rt.step()

    def run(self) -> None:
        """Run policy loop."""
        print("Waiting for robot state...")
        while not self._state_received:
            time.sleep(0.01)

        print("Running policy (Ctrl+C to stop)...")
        try:
            while True:
                step_start = time.perf_counter()
                self.step()
                elapsed = time.perf_counter() - step_start
                sleep_time = self.control_dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            print("\nStopping policy...")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run ONNX policy over SDK2")
    parser.add_argument("policy", help="Path to ONNX policy file")
    parser.add_argument("--robot", default="g1", choices=["g1", "go2"], help="Robot type")
    parser.add_argument("--real", metavar="INTERFACE", help="Run on real robot with given interface")
    parser.add_argument("--vx", type=float, default=0.0, help="Forward velocity command")
    parser.add_argument("--vy", type=float, default=0.0, help="Lateral velocity command")
    parser.add_argument("--wz", type=float, default=0.0, help="Angular velocity command")
    parser.add_argument("--hz", type=float, default=50.0, help="Control frequency")

    args = parser.parse_args()

    if args.real:
        domain_id = 0
        interface = args.real
    else:
        domain_id = 1
        interface = "lo0"

    runner = SDK2PolicyRunner(
        policy_path=args.policy,
        robot_type=args.robot,
        domain_id=domain_id,
        interface=interface,
        control_dt=1.0 / args.hz,
    )
    runner.set_command(args.vx, args.vy, args.wz)
    runner.run()


if __name__ == "__main__":
    main()
