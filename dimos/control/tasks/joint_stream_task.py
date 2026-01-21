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

"""Streaming joint command task for real-time control.

Accepts streaming joint positions (e.g., from LCM topic) and outputs them
directly to hardware each tick. Useful for teleoperation, visual servoing,
or any real-time control where you don't want trajectory planning overhead.

CRITICAL: Uses t_now from OrchestratorState, never calls time.time()
"""

from __future__ import annotations

from dataclasses import dataclass
import threading

from dimos.control.task import (
    ControlMode,
    ControlTask,
    JointCommandOutput,
    OrchestratorState,
    ResourceClaim,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class JointStreamTaskConfig:
    """Configuration for streaming joint task.

    Attributes:
        joint_names: List of joint names this task controls
        priority: Priority for arbitration (higher wins)
        timeout: If no command received for this many seconds, go inactive (0 = never timeout)
    """

    joint_names: list[str]
    priority: int = 10
    timeout: float = 0.5  # 500ms default timeout


class JointStreamTask(ControlTask):
    """Streaming joint command task for real-time control.

    Accepts joint positions via set_positions() and outputs them each tick.
    No trajectory planning - just pass-through with optional timeout.

    Example:
        >>> task = JointStreamTask(
        ...     name="stream_arm",
        ...     config=JointStreamTaskConfig(
        ...         joint_names=["joint1", "joint2", "joint3"],
        ...         priority=10,
        ...         timeout=0.5,
        ...     ),
        ... )
        >>> orchestrator.add_task(task)
        >>>
        >>> # From LCM callback or other source:
        >>> task.set_positions([0.1, 0.2, 0.3])
    """

    def __init__(self, name: str, config: JointStreamTaskConfig) -> None:
        """Initialize streaming task.

        Args:
            name: Unique task name
            config: Task configuration
        """
        if not config.joint_names:
            raise ValueError(f"JointStreamTask '{name}' requires at least one joint")

        self._name = name
        self._config = config
        self._joint_names = frozenset(config.joint_names)
        self._joint_names_list = list(config.joint_names)
        self._num_joints = len(config.joint_names)

        # Current command (thread-safe)
        self._lock = threading.Lock()
        self._positions: list[float] | None = None
        self._last_update_time: float = 0.0
        self._active = False

        logger.info(f"JointStreamTask {name} initialized for joints: {config.joint_names}")

    @property
    def name(self) -> str:
        """Unique task identifier."""
        return self._name

    def claim(self) -> ResourceClaim:
        """Declare resource requirements."""
        return ResourceClaim(
            joints=self._joint_names,
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        """Check if task should run this tick."""
        with self._lock:
            return self._active and self._positions is not None

    def compute(self, state: OrchestratorState) -> JointCommandOutput | None:
        """Output current joint positions.
        Args:
            state: Current orchestrator state
        Returns:
            JointCommandOutput with positions, or None if inactive/timed out
        """
        with self._lock:
            if self._positions is None:
                return None

            # Check timeout
            if self._config.timeout > 0:
                time_since_update = state.t_now - self._last_update_time
                if time_since_update > self._config.timeout:
                    logger.warning(
                        f"JointStreamTask {self._name} timed out "
                        f"(no update for {time_since_update:.3f}s)"
                    )
                    self._active = False
                    return None

            return JointCommandOutput(
                joint_names=self._joint_names_list,
                positions=list(self._positions),
                mode=ControlMode.SERVO_POSITION,
            )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """Handle preemption by higher-priority task.

        Args:
            by_task: Name of preempting task
            joints: Joints that were preempted
        """
        if joints & self._joint_names:
            logger.warning(
                f"JointStreamTask {self._name} preempted by {by_task} on joints {joints}"
            )

    # =========================================================================
    # Task-specific methods
    # =========================================================================

    def set_positions(self, positions: list[float], t_now: float | None = None) -> bool:
        """Set target joint positions.

        Call this from your LCM callback or other data source.

        Args:
            positions: Joint positions in radians (must match joint_names length)
            t_now: Current time (if None, uses last known time - less accurate for timeout)

        Returns:
            True if accepted, False if wrong number of joints
        """
        if len(positions) != self._num_joints:
            logger.warning(
                f"JointStreamTask {self._name}: expected {self._num_joints} "
                f"positions, got {len(positions)}"
            )
            return False

        with self._lock:
            self._positions = list(positions)
            if t_now is not None:
                self._last_update_time = t_now
            self._active = True

        return True

    def start(self) -> None:
        """Activate the task (start accepting commands)."""
        with self._lock:
            self._active = True
        logger.info(f"JointStreamTask {self._name} started")

    def stop(self) -> None:
        """Deactivate the task (stop outputting commands)."""
        with self._lock:
            self._active = False
        logger.info(f"JointStreamTask {self._name} stopped")

    def clear(self) -> None:
        """Clear current positions and deactivate."""
        with self._lock:
            self._positions = None
            self._active = False
        logger.info(f"JointStreamTask {self._name} cleared")

    def is_streaming(self) -> bool:
        """Check if actively receiving and outputting commands."""
        with self._lock:
            return self._active and self._positions is not None


__all__ = [
    "JointStreamTask",
    "JointStreamTaskConfig",
]
