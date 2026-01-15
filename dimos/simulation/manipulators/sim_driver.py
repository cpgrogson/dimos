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

"""Robot-agnostic MuJoCo simulation driver module."""

from dataclasses import dataclass
import threading
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.core.module import ModuleConfig
from dimos.msgs.sensor_msgs import JointCommand, JointState, RobotState
from dimos.simulation.manipulators.sim_backend import SimBackend


@dataclass
class SimDriverConfig(ModuleConfig):
    robot: str | None = None
    config_path: str | None = None
    headless: bool = False


class SimDriver(Module[SimDriverConfig]):
    """Module wrapper for MuJoCo simulation backends."""

    default_config = SimDriverConfig
    config: SimDriverConfig

    joint_state: Out[JointState]
    robot_state: Out[RobotState]
    joint_position_command: In[JointCommand]
    joint_velocity_command: In[JointCommand]

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._backend = self._create_backend()
        self._control_rate = 100.0
        self._monitor_rate = 100.0
        self._joint_prefix = "joint"
        self._stop_event = threading.Event()
        self._control_thread: threading.Thread | None = None
        self._monitor_thread: threading.Thread | None = None
        self._command_lock = threading.Lock()
        self._pending_positions: list[float] | None = None
        self._pending_velocities: list[float] | None = None

    def _create_backend(self) -> SimBackend:
        return SimBackend(
            robot=self.config.robot or "",
            config_path=self.config.config_path,
            headless=self.config.headless,
        )

    @rpc
    def start(self) -> None:
        super().start()
        if not self._backend.connect():
            raise RuntimeError("Failed to connect to MuJoCo simulation backend")
        self._backend.write_enable(True)

        try:
            if (
                self.joint_position_command.connection is not None
                or self.joint_position_command._transport is not None
            ):
                self._disposables.add(
                    Disposable(
                        self.joint_position_command.subscribe(self._on_joint_position_command)
                    )
                )
        except Exception as exc:
            import logging

            logging.getLogger(self.__class__.__name__).warning(
                f"Failed to subscribe joint_position_command: {exc}"
            )

        try:
            if (
                self.joint_velocity_command.connection is not None
                or self.joint_velocity_command._transport is not None
            ):
                self._disposables.add(
                    Disposable(
                        self.joint_velocity_command.subscribe(self._on_joint_velocity_command)
                    )
                )
        except Exception as exc:
            import logging

            logging.getLogger(self.__class__.__name__).warning(
                f"Failed to subscribe joint_velocity_command: {exc}"
            )

        self._stop_event.clear()
        self._control_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name=f"{self.__class__.__name__}-control",
        )
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            daemon=True,
            name=f"{self.__class__.__name__}-monitor",
        )
        self._control_thread.start()
        self._monitor_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=2.0)
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=2.0)
        self._backend.disconnect()
        super().stop()

    @rpc
    def enable_servos(self) -> bool:
        return self._backend.write_enable(True)

    @rpc
    def disable_servos(self) -> bool:
        return self._backend.write_enable(False)

    @rpc
    def clear_errors(self) -> bool:
        return self._backend.write_clear_errors()

    @rpc
    def emergency_stop(self) -> bool:
        return self._backend.write_stop()

    def _on_joint_position_command(self, msg: JointCommand) -> None:
        with self._command_lock:
            self._pending_positions = list(msg.positions)
            self._pending_velocities = None

    def _on_joint_velocity_command(self, msg: JointCommand) -> None:
        with self._command_lock:
            self._pending_velocities = list(msg.positions)
            self._pending_positions = None

    def _control_loop(self) -> None:
        period = 1.0 / max(self._control_rate, 1.0)
        while not self._stop_event.is_set():
            with self._command_lock:
                positions = (
                    None if self._pending_positions is None else list(self._pending_positions)
                )
                velocities = (
                    None if self._pending_velocities is None else list(self._pending_velocities)
                )

            if positions is not None:
                self._backend.write_joint_positions(positions)
            elif velocities is not None:
                self._backend.write_joint_velocities(velocities)

            time.sleep(period)

    def _monitor_loop(self) -> None:
        period = 1.0 / max(self._monitor_rate, 1.0)
        names = [f"{self._joint_prefix}{i + 1}" for i in range(self._backend.get_dof())]
        while not self._stop_event.is_set():
            positions = self._backend.read_joint_positions()
            velocities = self._backend.read_joint_velocities()
            efforts = self._backend.read_joint_efforts()
            state = self._backend.read_state()
            error_code, _ = self._backend.read_error()

            self.joint_state.publish(
                JointState(
                    frame_id=self.frame_id,
                    name=names,
                    position=positions,
                    velocity=velocities,
                    effort=efforts,
                )
            )
            self.robot_state.publish(
                RobotState(
                    state=state.get("state", 0),
                    mode=state.get("mode", 0),
                    error_code=error_code,
                    warn_code=0,
                    cmdnum=0,
                    mt_brake=0,
                    mt_able=1 if self._backend.read_enabled() else 0,
                    tcp_pose=[],
                    tcp_offset=[],
                    joints=[float(p) for p in positions],
                )
            )
            time.sleep(period)


def get_blueprint() -> dict[str, Any]:
    return {
        "name": "SimDriver",
        "class": SimDriver,
        "config": {
            "robot": None,
            "config_path": None,
            "headless": False,
        },
        "inputs": {
            "joint_position_command": "JointCommand",
            "joint_velocity_command": "JointCommand",
        },
        "outputs": {
            "joint_state": "JointState",
            "robot_state": "RobotState",
        },
    }


simulation = SimDriver.blueprint


__all__ = [
    "SimDriver",
    "SimDriverConfig",
    "get_blueprint",
    "simulation",
]
