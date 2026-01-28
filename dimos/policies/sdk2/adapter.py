from __future__ import annotations

from typing import Protocol

from .types import CommandContext, JointTargets, RobotState


class PolicyAdapter(Protocol):
    """Adapter interface for SDK2-deployed policies."""

    @property
    def joint_names(self) -> list[str]:
        """Policy joint ordering (names). Must match RobotState.q/dq ordering."""

    def reset(self) -> None:
        """Reset internal state (history buffers, filters, etc.)."""

    def step(self, state: RobotState, ctx: CommandContext) -> JointTargets:
        """Compute the next joint targets from the current state + command context."""


