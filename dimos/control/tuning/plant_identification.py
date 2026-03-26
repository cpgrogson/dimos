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

"""First-Order Plus Dead Time (FOPDT) plant identification from step-response data.

Reads a CSV produced by ``step_response_test.py`` and fits the FOPDT model::

    G(s) = K * exp(-theta * s) / (tau * s + 1)

per channel (vx, vy, wz).

Usage:
    python -m dimos.control.tuning.plant_identification --data step_response_data.csv
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass
class FOPDTParams:
    """First-Order Plus Dead Time model parameters."""

    K: float  # steady-state gain
    tau: float  # time constant (s)
    theta: float  # dead time (s)
    channel: str = ""

    def __repr__(self) -> str:
        return f"FOPDT({self.channel}: K={self.K:.4f}, tau={self.tau:.4f}s, theta={self.theta:.4f}s)"


def fopdt_step_response(t: NDArray, K: float, tau: float, theta: float, amplitude: float) -> NDArray:
    """Analytical FOPDT step response.

    y(t) = K * amplitude * (1 - exp(-(t - theta) / tau))  for t >= theta
    y(t) = 0                                                for t < theta
    """
    y = np.zeros_like(t)
    mask = t >= theta
    y[mask] = K * amplitude * (1.0 - np.exp(-(t[mask] - theta) / tau))
    return y


def identify_channel(
    timestamps: NDArray,
    cmd_values: NDArray,
    actual_values: NDArray,
) -> FOPDTParams:
    """Identify FOPDT parameters for a single step-response trial.

    Uses the 63.2% method as initial estimate, then refines with
    least-squares curve fitting.
    """
    from scipy.optimize import curve_fit

    # Find step onset (first non-zero command)
    step_idx = np.argmax(cmd_values > 0)
    if step_idx == 0:
        step_idx = 1
    amplitude = float(cmd_values[step_idx])
    t0 = timestamps[step_idx]

    # Re-zero time relative to step onset
    t_rel = timestamps[step_idx:] - t0
    y = actual_values[step_idx:]

    if len(t_rel) < 10 or amplitude < 1e-6:
        return FOPDTParams(K=1.0, tau=0.1, theta=0.0)

    # Steady-state gain estimate
    y_ss = float(np.mean(y[-max(1, len(y) // 5) :]))
    K_est = y_ss / amplitude if amplitude > 1e-6 else 1.0
    K_est = max(K_est, 0.01)

    # Dead time estimate: time until response exceeds 5% of steady state
    threshold = 0.05 * abs(y_ss)
    dead_time_mask = np.abs(y) > threshold
    theta_est = float(t_rel[np.argmax(dead_time_mask)]) if np.any(dead_time_mask) else 0.0
    theta_est = max(theta_est, 0.0)

    # Time constant estimate: time from theta to 63.2% of steady state
    target_63 = 0.632 * y_ss
    post_theta = t_rel >= theta_est
    above_63 = np.abs(y) >= abs(target_63)
    both = post_theta & above_63
    if np.any(both):
        tau_est = float(t_rel[np.argmax(both)] - theta_est)
    else:
        tau_est = 0.1
    tau_est = max(tau_est, 0.01)

    # Curve fit refinement
    def model(t: NDArray, K: float, tau: float, theta: float) -> NDArray:
        return fopdt_step_response(t, K, tau, theta, amplitude)

    try:
        popt, _ = curve_fit(
            model,
            t_rel,
            y,
            p0=[K_est, tau_est, theta_est],
            bounds=([0.001, 0.001, 0.0], [10.0, 5.0, 2.0]),
            maxfev=5000,
        )
        return FOPDTParams(K=popt[0], tau=popt[1], theta=popt[2])
    except Exception:
        return FOPDTParams(K=K_est, tau=tau_est, theta=theta_est)


def load_and_identify(csv_path: str) -> dict[str, FOPDTParams]:
    """Load CSV and identify FOPDT params per channel.

    Returns dict keyed by channel name.
    """
    import csv

    rows_by_channel: dict[str, list[dict]] = {}
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            ch = row["channel"]
            rows_by_channel.setdefault(ch, []).append(row)

    results: dict[str, FOPDTParams] = {}
    for channel, rows in rows_by_channel.items():
        ts = np.array([float(r["timestamp"]) for r in rows])
        cmd = np.array([float(r["cmd_value"]) for r in rows])
        actual = np.array([float(r["actual_value"]) for r in rows])

        params = identify_channel(ts, cmd, actual)
        params.channel = channel
        results[channel] = params

    return results


def main() -> None:
    parser = argparse.ArgumentParser(description="FOPDT plant identification from step-response CSV")
    parser.add_argument("--data", required=True, help="Path to step_response_data.csv")
    parser.add_argument("--plot", action="store_true", help="Generate diagnostic plots")
    args = parser.parse_args()

    results = load_and_identify(args.data)

    print("\n" + "=" * 60)
    print("FOPDT Plant Identification Results")
    print("=" * 60)
    for channel, params in sorted(results.items()):
        print(f"  {params}")
    print("=" * 60)

    if args.plot:
        try:
            import matplotlib.pyplot as plt
            _plot_results(args.data, results, plt)
        except ImportError:
            print("matplotlib not available — skipping plots")


def _plot_results(csv_path: str, results: dict[str, FOPDTParams], plt: object) -> None:
    """Generate diagnostic overlay plots."""
    import csv

    rows_by_channel: dict[str, list[dict]] = {}
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows_by_channel.setdefault(row["channel"], []).append(row)

    fig, axes = plt.subplots(len(results), 1, figsize=(10, 4 * len(results)))  # type: ignore[attr-defined]
    if len(results) == 1:
        axes = [axes]

    for ax, (channel, params) in zip(axes, sorted(results.items())):
        rows = rows_by_channel[channel]
        ts = np.array([float(r["timestamp"]) for r in rows])
        cmd = np.array([float(r["cmd_value"]) for r in rows])
        actual = np.array([float(r["actual_value"]) for r in rows])

        # Find step onset for model overlay
        step_idx = np.argmax(cmd > 0)
        amplitude = float(cmd[step_idx]) if step_idx > 0 else 0.3
        t0 = ts[step_idx]
        t_rel = ts - t0

        model_y = fopdt_step_response(t_rel, params.K, params.tau, params.theta, amplitude)

        ax.plot(t_rel, actual, "b-", label="Actual", alpha=0.7)
        ax.plot(t_rel, model_y, "r--", label=f"FOPDT (K={params.K:.3f}, τ={params.tau:.3f}, θ={params.theta:.3f})")
        ax.plot(t_rel, cmd * params.K, "g:", label="Command × K", alpha=0.5)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(f"{channel} velocity")
        ax.set_title(f"Channel: {channel}")
        ax.legend()
        ax.grid(True, alpha=0.3)

    plt.tight_layout()  # type: ignore[attr-defined]
    plt.savefig("plant_identification.png", dpi=150)  # type: ignore[attr-defined]
    print("Saved plot to plant_identification.png")


if __name__ == "__main__":
    main()
