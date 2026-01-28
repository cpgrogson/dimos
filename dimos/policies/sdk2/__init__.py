"""SDK2 policy runtime and adapters.

This package provides a pluggable policy framework for deploying controllers over
Unitree SDK2 DDS (rt/lowstate, rt/lowcmd), shared across:
- sim2sim: MuJoCo publishes rt/lowstate (SDK2BridgeController), policy publishes rt/lowcmd
- sim2real: policy publishes rt/lowcmd to the real robot
- mirror: MuJoCo subscribes rt/lowstate for visualization while policy publishes rt/lowcmd
"""

from .types import CommandContext, JointTargets, RobotState
from .runtime import PolicyRuntime

__all__ = [
    "CommandContext",
    "JointTargets",
    "PolicyRuntime",
    "RobotState",
]


