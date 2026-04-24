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

"""GR00T whole-body-control task for the Unitree G1 humanoid.

Runs the two-model GR00T WBC locomotion policy (balance + walk) inside
the coordinator tick loop.  Claims the 15 legs+waist joints at high
priority; arm joints are left to lower-priority tasks in the blueprint.

Reference implementation: g1_control/backends/groot_wbc_backend.py.
Observation, action, and model-selection semantics are preserved
verbatim — changing them drifts us away from the ONNX policies trained
by GR00T-WholeBodyControl.

CRITICAL: Uses t_now from CoordinatorState, never calls time.time().
"""

from __future__ import annotations

from dataclasses import dataclass, field
import threading
from typing import TYPE_CHECKING

import numpy as np
import onnxruntime as ort  # type: ignore[import-untyped]

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from pathlib import Path

    from dimos.hardware.whole_body.spec import WholeBodyAdapter
    from dimos.msgs.geometry_msgs import Twist

logger = setup_logger()


# Default joint angles copied verbatim from
# g1_control/backends/groot_wbc_backend.py DEFAULT_29.  Policy was trained
# against these as the zero-offset pose.
_DEFAULT_POSITIONS_29 = [
    -0.1,
    0.0,
    0.0,
    0.3,
    -0.2,
    0.0,  # left leg
    -0.1,
    0.0,
    0.0,
    0.3,
    -0.2,
    0.0,  # right leg
    0.0,
    0.0,
    0.0,  # waist
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,  # left arm (not driven by policy)
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,  # right arm (not driven by policy)
]

_SINGLE_OBS_DIM = 86
_OBS_HISTORY_LEN = 6
_NUM_ACTIONS = 15
_NUM_MOTORS = 29


@dataclass
class GrootWBCTaskConfig:
    """Configuration for the GR00T WBC task.

    Attributes:
        balance_onnx: Path to the balance ONNX model.  Used when
            ``||cmd|| <= cmd_norm_threshold``.
        walk_onnx: Path to the walk ONNX model.  Used otherwise.
        joint_names: The 15 coordinator joint names this task claims
            (legs 0-11 + waist 12-14, in DDS order).
        all_joint_names: All 29 coordinator joint names in DDS order
            (legs 0-11 + waist 12-14 + arms 15-28).  Required to build
            the observation, which feeds all 29 joint states.
        default_positions_29: Default joint angles for all 29 joints
            (DDS order).  First 15 are the policy's zero-offset pose.
        priority: Arbitration priority (higher wins).  50 is the
            recommended WBC priority per the task.py conventions.
        decimation: Run inference every N ticks.  At 500 Hz tick /
            50 Hz policy → decimation=10.
        action_scale: Multiplier on raw policy output before adding
            defaults.
        obs_ang_vel_scale: Scale for base angular velocity in obs.
        obs_dof_pos_scale: Scale for joint position offset in obs.
        obs_dof_vel_scale: Scale for joint velocity in obs.
        cmd_scale: Per-axis scale applied to (vx, vy, wz) in obs.
        cmd_norm_threshold: ||cmd|| below this selects the balance
            model, otherwise walk.
        height_cmd: Fixed height command slot in obs.
        timeout: Seconds without a velocity command before zeroing it.
    """

    balance_onnx: str | Path
    walk_onnx: str | Path
    joint_names: list[str]
    all_joint_names: list[str]
    default_positions_29: list[float] = field(default_factory=lambda: list(_DEFAULT_POSITIONS_29))
    priority: int = 50
    decimation: int = 10
    action_scale: float = 0.25
    obs_ang_vel_scale: float = 0.5
    obs_dof_pos_scale: float = 1.0
    obs_dof_vel_scale: float = 0.05
    cmd_scale: tuple[float, float, float] = (2.0, 2.0, 0.5)
    cmd_norm_threshold: float = 0.05
    height_cmd: float = 0.74
    timeout: float = 1.0


class GrootWBCTask(BaseControlTask):
    """Runs the GR00T balance / walk ONNX policies inside the coordinator tick loop.

    Observation vector (86 dims, built each inference tick, replicates
    ``groot_wbc_backend.GrootWBCBackend._compute_obs`` verbatim):

        [0:3]    cmd_vel * cmd_scale                # scaled velocity command
        [3]      height_cmd                         # fixed slot (0.74)
        [4:7]    (0, 0, 0)                          # rpy_cmd, zeros
        [7:10]   gyro * obs_ang_vel_scale           # body-frame ang vel
        [10:13]  projected_gravity(quat)            # gravity in body frame
        [13:42]  (q_29 - default_29) * dof_pos_scale
        [42:71]  dq_29 * dof_vel_scale
        [71:86]  last_action (15 dims)

    The observation is stacked into a 6-frame history buffer (516 dims)
    before being fed to ONNX.

    Action (15 dims, legs + waist only):

        target_q_15 = action * action_scale + default_15

    Arms are NOT driven by this task — the blueprint pairs this task
    with a lower-priority servo task scoped to the 14 arm joints.
    """

    def __init__(
        self,
        name: str,
        config: GrootWBCTaskConfig,
        adapter: WholeBodyAdapter,
    ) -> None:
        if len(config.joint_names) != _NUM_ACTIONS:
            raise ValueError(
                f"GrootWBCTask '{name}' requires exactly {_NUM_ACTIONS} joint names "
                f"(legs + waist), got {len(config.joint_names)}"
            )
        if len(config.all_joint_names) != _NUM_MOTORS:
            raise ValueError(
                f"GrootWBCTask '{name}' requires exactly {_NUM_MOTORS} all_joint_names "
                f"(full 29-DOF G1), got {len(config.all_joint_names)}"
            )
        if len(config.default_positions_29) != _NUM_MOTORS:
            raise ValueError(
                f"GrootWBCTask '{name}' requires exactly {_NUM_MOTORS} "
                f"default_positions_29, got {len(config.default_positions_29)}"
            )
        if config.decimation < 1:
            raise ValueError(f"GrootWBCTask '{name}' requires decimation >= 1")

        self._name = name
        self._config = config
        self._adapter = adapter
        self._joint_names_list = list(config.joint_names)
        self._joint_names_set = frozenset(config.joint_names)
        self._all_joint_names = list(config.all_joint_names)

        providers = ort.get_available_providers()
        self._balance_session = ort.InferenceSession(str(config.balance_onnx), providers=providers)
        self._walk_session = ort.InferenceSession(str(config.walk_onnx), providers=providers)
        self._balance_input = self._balance_session.get_inputs()[0].name
        self._walk_input = self._walk_session.get_inputs()[0].name
        logger.info(
            f"GrootWBCTask '{name}' loaded balance={config.balance_onnx}, "
            f"walk={config.walk_onnx} (providers: {providers})"
        )

        self._default_29 = np.asarray(config.default_positions_29, dtype=np.float32)
        self._default_15 = self._default_29[:_NUM_ACTIONS]
        self._cmd_scale = np.asarray(config.cmd_scale, dtype=np.float32)

        # State
        self._last_action = np.zeros(_NUM_ACTIONS, dtype=np.float32)
        self._obs_buf = np.zeros((1, _SINGLE_OBS_DIM * _OBS_HISTORY_LEN), dtype=np.float32)
        self._first_inference = True
        self._tick_count = 0
        self._last_targets: list[float] | None = None
        self._active = False

        self._cmd_lock = threading.Lock()
        self._cmd = np.zeros(3, dtype=np.float32)
        self._last_cmd_time: float = 0.0

    @property
    def name(self) -> str:
        return self._name

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=self._joint_names_set,
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        return self._active

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        if not self._active:
            return None

        self._tick_count += 1

        # Decimation: only run inference every N ticks.  Between inference
        # ticks, re-emit the last target so the coordinator keeps driving
        # the joints.  Matches the Go2 RLPolicyTask pattern.
        if self._tick_count % self._config.decimation != 0:
            if self._last_targets is not None:
                return JointCommandOutput(
                    joint_names=self._joint_names_list,
                    positions=self._last_targets,
                    mode=ControlMode.SERVO_POSITION,
                )
            return None

        # Read all 29 joints from CoordinatorState in DDS order.
        q_29 = np.zeros(_NUM_MOTORS, dtype=np.float32)
        dq_29 = np.zeros(_NUM_MOTORS, dtype=np.float32)
        for i, jname in enumerate(self._all_joint_names):
            pos = state.joints.get_position(jname)
            vel = state.joints.get_velocity(jname)
            q_29[i] = pos if pos is not None else 0.0
            dq_29[i] = vel if vel is not None else 0.0

        # IMU comes from the adapter, not CoordinatorState.
        imu = self._adapter.read_imu()
        gyro = np.asarray(imu.gyroscope, dtype=np.float32)
        gravity = self._projected_gravity(imu.quaternion)

        # Velocity command (with timeout → zero).
        with self._cmd_lock:
            if (
                self._config.timeout > 0.0
                and self._last_cmd_time > 0.0
                and (state.t_now - self._last_cmd_time) > self._config.timeout
            ):
                cmd = np.zeros(3, dtype=np.float32)
            else:
                cmd = self._cmd.copy()

        obs = self._build_obs(cmd=cmd, gyro=gyro, gravity=gravity, q=q_29, dq=dq_29)

        # History buffer: first inference fills all slots with the current
        # obs (warm-start); subsequent ticks roll the window.
        if self._first_inference:
            tiled = np.tile(obs, _OBS_HISTORY_LEN)
            self._obs_buf[0, :] = tiled
            self._first_inference = False
        else:
            self._obs_buf[0, : _SINGLE_OBS_DIM * (_OBS_HISTORY_LEN - 1)] = self._obs_buf[
                0, _SINGLE_OBS_DIM:
            ]
            self._obs_buf[0, _SINGLE_OBS_DIM * (_OBS_HISTORY_LEN - 1) :] = obs

        # Model selection: balance when near-stationary, walk otherwise.
        cmd_norm = float(np.linalg.norm(cmd))
        if cmd_norm <= self._config.cmd_norm_threshold:
            raw = self._balance_session.run(None, {self._balance_input: self._obs_buf})[0]
        else:
            raw = self._walk_session.run(None, {self._walk_input: self._obs_buf})[0]

        action = raw[0, :_NUM_ACTIONS].astype(np.float32)
        self._last_action[:] = action

        target_q_15 = action * self._config.action_scale + self._default_15
        self._last_targets = target_q_15.tolist()

        return JointCommandOutput(
            joint_names=self._joint_names_list,
            positions=self._last_targets,
            mode=ControlMode.SERVO_POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints & self._joint_names_set:
            logger.warning(f"GrootWBCTask '{self._name}' preempted by {by_task} on {joints}")

    # ------------------------------------------------------------------
    # Velocity command input
    # ------------------------------------------------------------------

    def set_velocity_command(self, vx: float, vy: float, yaw_rate: float, t_now: float) -> None:
        """Set the (vx, vy, yaw_rate) commanded to the policy.

        Called by the coordinator's twist_command dispatcher and by
        external Python callers.  Thread-safe.
        """
        with self._cmd_lock:
            self._cmd[:] = [vx, vy, yaw_rate]
            self._last_cmd_time = t_now

    def on_twist(self, msg: Twist, t_now: float) -> bool:
        """Accept a Twist message, e.g. from an LCM cmd_vel transport."""
        self.set_velocity_command(
            float(msg.linear.x),
            float(msg.linear.y),
            float(msg.angular.z),
            t_now,
        )
        return True

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Activate the policy.  Resets history + last-action state."""
        self._active = True
        self._tick_count = 0
        self._first_inference = True
        self._last_action[:] = 0.0
        # Seed with the default 15-DOF bent-knee pose so the coordinator
        # holds the robot in a stable stance during the ~decimation ticks
        # before the first 50 Hz inference fires.  Without this, legs
        # would get POS_STOP → 0.0 targets (= fully-extended stance) for
        # a few ms and the 150-kp PD would yank the robot around before
        # the policy has a chance to respond.
        self._last_targets = self._default_15.tolist()
        with self._cmd_lock:
            self._cmd[:] = 0.0
            self._last_cmd_time = 0.0
        logger.info(f"GrootWBCTask '{self._name}' started")

    def stop(self) -> None:
        """Deactivate the policy; re-activation resets history."""
        self._active = False
        self._last_targets = None
        logger.info(f"GrootWBCTask '{self._name}' stopped")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _build_obs(
        self,
        cmd: np.ndarray,
        gyro: np.ndarray,
        gravity: np.ndarray,
        q: np.ndarray,
        dq: np.ndarray,
    ) -> np.ndarray:
        """Build the 86-dim GR00T observation.  Layout matches
        ``groot_wbc_backend.py`` exactly."""
        obs = np.zeros(_SINGLE_OBS_DIM, dtype=np.float32)
        obs[0:3] = cmd * self._cmd_scale
        obs[3] = self._config.height_cmd
        obs[4:7] = 0.0
        obs[7:10] = gyro * self._config.obs_ang_vel_scale
        obs[10:13] = gravity
        obs[13:42] = (q - self._default_29) * self._config.obs_dof_pos_scale
        obs[42:71] = dq * self._config.obs_dof_vel_scale
        obs[71:86] = self._last_action
        return obs

    @staticmethod
    def _projected_gravity(quaternion: tuple[float, ...]) -> np.ndarray:
        """Project world gravity into body frame.

        Uses Unitree DDS quaternion order (w, x, y, z).  Formula matches
        ``groot_wbc_backend._get_gravity_orientation`` and is
        algebraically equivalent to the Go2 RLPolicyTask helper.
        """
        w, x, y, z = quaternion
        gx = 2.0 * (-x * z + w * y)
        gy = 2.0 * (-y * z - w * x)
        gz = -(w * w - x * x - y * y + z * z)
        return np.array([gx, gy, gz], dtype=np.float32)


__all__ = [
    "GrootWBCTask",
    "GrootWBCTaskConfig",
]
