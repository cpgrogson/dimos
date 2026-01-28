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


# pyright: reportMissingImports=false
# pyright: reportMissingModuleSource=false

import base64
from collections.abc import Callable
import functools
import json
from pathlib import Path
import pickle
import subprocess
import sys
import threading
import time
from typing import Any, TypeVar

import numpy as np
from numpy.typing import NDArray
from reactivex import Observable
from reactivex.abc import ObserverBase, SchedulerBase
from reactivex.disposable import Disposable

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs import Quaternion, Twist, Vector3
from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.simulation.mujoco.constants import LAUNCHER_PATH, LIDAR_FPS, VIDEO_FPS
from dimos.simulation.mujoco.model import load_bundle_json
from dimos.simulation.mujoco.shared_memory import ShmWriter
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

ODOM_FREQUENCY = 50

logger = setup_logger()

T = TypeVar("T")


class _PolicyRuntimeShim:
    """Unify PolicyRuntime to the legacy SDK2PolicyRunner surface used by MujocoConnection."""

    def __init__(self, rt: Any) -> None:
        self._rt = rt

    def step(self) -> None:
        self._rt.step()

    def set_command(self, vx: float, vy: float, wz: float) -> None:
        self._rt.set_cmd_vel(vx, vy, wz)

    def set_enabled(self, enabled: bool) -> None:
        self._rt.set_enabled(enabled)

    def set_estop(self, estop: bool) -> None:
        self._rt.set_estop(estop)

    def set_policy_params_json(self, params_json: str) -> None:
        self._rt.set_policy_params_json(params_json)


class MujocoConnection:
    """MuJoCo simulator connection that runs in a separate subprocess."""

    def __init__(self, global_config: GlobalConfig) -> None:
        try:
            import mujoco
        except ImportError:
            raise ImportError("'mujoco' is not installed. Use `pip install -e .[sim]`")

        # Pre-download the mujoco_sim data.
        get_data("mujoco_sim")

        # Trigger the download of the mujoco_menagerie package. This is so it
        # doesn't trigger in the mujoco process where it can time out.
        # When using a profile bundle, we should not rely on menagerie assets.
        if not global_config.mujoco_profile:
            from mujoco_playground._src import mjx_env

            mjx_env.ensure_menagerie_exists()

        self.global_config = global_config
        self.process: subprocess.Popen[bytes] | None = None
        self.shm_data: ShmWriter | None = None
        self._last_video_seq = 0
        self._last_odom_seq = 0
        self._last_lidar_seq = 0
        self._stop_timer: threading.Timer | None = None

        self._stream_threads: list[threading.Thread] = []
        self._stop_events: list[threading.Event] = []
        self._is_cleaned_up = False

        # SDK2 policy runner
        self._policy_runner: Any = None
        self._policy_thread: threading.Thread | None = None
        self._policy_stop_event: threading.Event | None = None

        # Latched safety state (so UI clicks before the runner is ready aren't dropped).
        self._desired_policy_enabled: bool = False
        self._desired_policy_estop: bool = False
        self._desired_policy_params_json: str = ""

    def start(self) -> None:
        self.shm_data = ShmWriter()

        config_pickle = base64.b64encode(pickle.dumps(self.global_config)).decode("ascii")
        shm_names_json = json.dumps(self.shm_data.shm.to_names())

        # Launch the subprocess
        try:
            # mjpython must be used macOS (because of launch_passive inside mujoco_process.py)
            executable = sys.executable if sys.platform != "darwin" else "mjpython"
            self.process = subprocess.Popen(
                [executable, str(LAUNCHER_PATH), config_pickle, shm_names_json],
            )

        except Exception as e:
            self.shm_data.cleanup()
            raise RuntimeError(f"Failed to start MuJoCo subprocess: {e}") from e

        # Wait for process to be ready
        ready_timeout = 300.0
        start_time = time.time()
        assert self.process is not None
        while time.time() - start_time < ready_timeout:
            if self.process.poll() is not None:
                exit_code = self.process.returncode
                self.stop()
                raise RuntimeError(f"MuJoCo process failed to start (exit code {exit_code})")
            if self.shm_data.is_ready():
                logger.info("MuJoCo process started successfully")
                # Start SDK2 policy runner if configured
                self._start_sdk2_policy_runner()
                return
            time.sleep(0.1)

        # Timeout
        self.stop()
        raise RuntimeError("MuJoCo process failed to start (timeout)")

    def _start_sdk2_policy_runner(self) -> None:
        """Start the SDK2 policy runner if configured."""
        if self.global_config.mujoco_control_mode not in ("sdk2", "mirror"):
            return

        profile = self.global_config.mujoco_profile
        if not profile:
            logger.warning("SDK2 mode enabled but no profile specified, skipping policy runner")
            return

        bundle_cfg = load_bundle_json(profile)
        if not bundle_cfg:
            logger.warning(f"No bundle.json found for profile {profile}")
            return

        policy_name = bundle_cfg.get("policy")
        if not policy_name:
            logger.info("SDK2 mode: no policy in bundle.json, waiting for external policy")
            return

        robot_type = str(bundle_cfg.get("robot_type", "g1"))
        policy_kind = str(bundle_cfg.get("policy_kind", "mjlab_velocity"))
        policy_config_name = bundle_cfg.get("policy_config")

        # Find policy file
        data_dir = Path(__file__).parent.parent.parent.parent / "data" / "mujoco_sim"   
        policy_path = data_dir / policy_name
        if not policy_path.exists():
            # Try in profile directory
            policy_path = data_dir / profile / policy_name
        if not policy_path.exists():
            logger.warning(f"SDK2 policy not found: {policy_name}")
            return

        logger.info(
            "Starting SDK2 policy runner",
            policy=str(policy_path),
            robot_type=robot_type,
            policy_kind=policy_kind,
        )

        # Import here to avoid circular imports / heavy optional deps.
        from dimos.simulation.mujoco.sdk2_policy_runner import SDK2PolicyRunner

        self._policy_stop_event = threading.Event()

        def run_policy() -> None:
            try:
                # Wait a bit for the simulator to fully initialize
                time.sleep(1.0)

                if policy_kind == "mjlab_velocity":
                    self._policy_runner = SDK2PolicyRunner(
                        policy_path=str(policy_path),
                        robot_type=robot_type,
                        domain_id=self.global_config.sdk2_domain_id,
                        interface=self.global_config.sdk2_interface,
                        control_dt=0.02,  # 50 Hz
                    )
                elif policy_kind == "falcon_loco_manip":
                    if not policy_config_name:
                        raise RuntimeError("Falcon policy_kind requires bundle.json policy_config (YAML)")
                    yaml_path = data_dir / profile / str(policy_config_name)
                    if not yaml_path.exists():
                        yaml_path = data_dir / str(policy_config_name)
                    if not yaml_path.exists():
                        raise RuntimeError(f"Falcon policy_config not found: {policy_config_name}")

                    from dimos.policies.sdk2.adapters.falcon import FalconLocoManipAdapter
                    from dimos.policies.sdk2.runtime import PolicyRuntime, PolicyRuntimeConfig

                    adapter = FalconLocoManipAdapter(
                        policy_path=str(policy_path),
                        falcon_yaml_path=str(yaml_path),
                        policy_action_scale=float(bundle_cfg.get("policy_action_scale", 0.25)),  # type: ignore[arg-type]
                    )
                    rt = PolicyRuntime(
                        adapter=adapter,
                        config=PolicyRuntimeConfig(
                            robot_type=robot_type,
                            domain_id=self.global_config.sdk2_domain_id,
                            interface=self.global_config.sdk2_interface,
                            control_dt=0.02,
                            mode_pr=int(bundle_cfg.get("mode_pr", 0)),  # type: ignore[arg-type]
                        ),
                    )
                    self._policy_runner = _PolicyRuntimeShim(rt)
                else:
                    raise RuntimeError(f"Unknown policy_kind '{policy_kind}'")

                # Run policy loop with stop check
                logger.info("SDK2 policy runner started")

                # Apply any latched safety state immediately.
                try:
                    if hasattr(self._policy_runner, "set_estop"):
                        self._policy_runner.set_estop(self._desired_policy_estop)  # type: ignore[attr-defined]
                    if hasattr(self._policy_runner, "set_enabled"):
                        self._policy_runner.set_enabled(self._desired_policy_enabled)  # type: ignore[attr-defined]
                    if hasattr(self._policy_runner, "set_policy_params_json") and self._desired_policy_params_json:
                        self._policy_runner.set_policy_params_json(self._desired_policy_params_json)  # type: ignore[attr-defined]
                    logger.info(
                        "Applied latched policy safety state",
                        enabled=self._desired_policy_enabled,
                        estop=self._desired_policy_estop,
                    )
                except Exception as e:
                    logger.warning(f"Failed applying latched safety state: {e}")

                while not self._policy_stop_event.is_set():
                    step_start = time.perf_counter()
                    if hasattr(self._policy_runner, "step"):
                        self._policy_runner.step()  # type: ignore[attr-defined]
                    elapsed = time.perf_counter() - step_start
                    sleep_time = 0.02 - elapsed
                    if sleep_time > 0:
                        time.sleep(sleep_time)

            except Exception as e:
                logger.error(f"SDK2 policy runner error: {e}")

        self._policy_thread = threading.Thread(target=run_policy, daemon=True, name="SDK2PolicyRunner")
        self._policy_thread.start()

    def stop(self) -> None:
        if self._is_cleaned_up:
            return

        self._is_cleaned_up = True

        # clean up open file descriptors
        if self.process:
            if self.process.stderr:
                self.process.stderr.close()
            if self.process.stdout:
                self.process.stdout.close()

        # Cancel any pending timers
        if self._stop_timer:
            self._stop_timer.cancel()
            self._stop_timer = None

        # Stop SDK2 policy runner
        if self._policy_stop_event:
            self._policy_stop_event.set()
        if self._policy_thread and self._policy_thread.is_alive():
            self._policy_thread.join(timeout=2.0)
            if self._policy_thread.is_alive():
                logger.warning("SDK2 policy runner did not stop gracefully")
        self._policy_runner = None
        self._policy_thread = None
        self._policy_stop_event = None

        # Stop all stream threads
        for stop_event in self._stop_events:
            stop_event.set()

        # Wait for threads to finish
        for thread in self._stream_threads:
            if thread.is_alive():
                thread.join(timeout=2.0)
                if thread.is_alive():
                    logger.warning(f"Stream thread {thread.name} did not stop gracefully")

        # Signal subprocess to stop
        if self.shm_data:
            self.shm_data.signal_stop()

        # Wait for process to finish
        if self.process:
            try:
                self.process.terminate()
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    logger.warning("MuJoCo process did not stop gracefully, killing")
                    self.process.kill()
                    self.process.wait(timeout=2)
            except Exception as e:
                logger.error(f"Error stopping MuJoCo process: {e}")

            self.process = None

        # Clean up shared memory
        if self.shm_data:
            self.shm_data.cleanup()
            self.shm_data = None

        # Clear references
        self._stream_threads.clear()
        self._stop_events.clear()

        self.lidar_stream.cache_clear()
        self.odom_stream.cache_clear()
        self.video_stream.cache_clear()

    def standup(self) -> bool:
        return True

    def liedown(self) -> bool:
        return True

    def get_video_frame(self) -> NDArray[Any] | None:
        if self.shm_data is None:
            return None

        frame, seq = self.shm_data.read_video()
        if seq > self._last_video_seq:
            self._last_video_seq = seq
            return frame

        return None

    def get_odom_message(self) -> Odometry | None:
        if self.shm_data is None:
            return None

        odom_data, seq = self.shm_data.read_odom()
        if seq > self._last_odom_seq and odom_data is not None:
            self._last_odom_seq = seq
            pos, quat_wxyz, timestamp = odom_data

            # Convert quaternion from (w,x,y,z) to (x,y,z,w) for ROS/Dimos
            orientation = Quaternion(quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0])

            return Odometry(
                position=Vector3(pos[0], pos[1], pos[2]),
                orientation=orientation,
                ts=timestamp,
                frame_id="world",
            )

        return None

    def get_lidar_message(self) -> LidarMessage | None:
        if self.shm_data is None:
            return None

        lidar_msg, seq = self.shm_data.read_lidar()
        if seq > self._last_lidar_seq and lidar_msg is not None:
            self._last_lidar_seq = seq
            return lidar_msg

        return None

    def _create_stream(
        self,
        getter: Callable[[], T | None],
        frequency: float,
        stream_name: str,
    ) -> Observable[T]:
        def on_subscribe(observer: ObserverBase[T], _scheduler: SchedulerBase | None) -> Disposable:
            if self._is_cleaned_up:
                observer.on_completed()
                return Disposable(lambda: None)

            stop_event = threading.Event()
            self._stop_events.append(stop_event)

            def run() -> None:
                try:
                    while not stop_event.is_set() and not self._is_cleaned_up:
                        data = getter()
                        if data is not None:
                            observer.on_next(data)
                        time.sleep(1 / frequency)
                except Exception as e:
                    logger.error(f"{stream_name} stream error: {e}")
                finally:
                    observer.on_completed()

            thread = threading.Thread(target=run, daemon=True)
            self._stream_threads.append(thread)
            thread.start()

            def dispose() -> None:
                stop_event.set()

            return Disposable(dispose)

        return Observable(on_subscribe)

    @functools.cache
    def lidar_stream(self) -> Observable[LidarMessage]:
        return self._create_stream(self.get_lidar_message, LIDAR_FPS, "Lidar")

    @functools.cache
    def odom_stream(self) -> Observable[Odometry]:
        return self._create_stream(self.get_odom_message, ODOM_FREQUENCY, "Odom")

    @functools.cache
    def video_stream(self) -> Observable[Image]:
        def get_video_as_image() -> Image | None:
            frame = self.get_video_frame()
            # MuJoCo renderer returns RGB uint8 frames; Image.from_numpy defaults to BGR.
            return Image.from_numpy(frame, format=ImageFormat.RGB) if frame is not None else None

        return self._create_stream(get_video_as_image, VIDEO_FPS, "Video")

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        if self._is_cleaned_up:
            return True

        # SDK2 mode: send velocity command to policy runner
        if self._policy_runner is not None:
            self._policy_runner.set_command(
                twist.linear.x,
                twist.linear.y,
                twist.angular.z,
            )
        elif self.shm_data is not None:
            # ONNX mode: send to shared memory
            linear = np.array([twist.linear.x, twist.linear.y, twist.linear.z], dtype=np.float32)
            angular = np.array([twist.angular.x, twist.angular.y, twist.angular.z], dtype=np.float32)
            self.shm_data.write_command(linear, angular)

        if duration > 0:
            if self._stop_timer:
                self._stop_timer.cancel()

            def stop_movement() -> None:
                if self._policy_runner is not None:
                    self._policy_runner.set_command(0.0, 0.0, 0.0)
                elif self.shm_data:
                    self.shm_data.write_command(
                        np.zeros(3, dtype=np.float32), np.zeros(3, dtype=np.float32)
                    )
                self._stop_timer = None

            self._stop_timer = threading.Timer(duration, stop_movement)
            self._stop_timer.daemon = True
            self._stop_timer.start()
        return True

    def set_policy_enabled(self, enabled: bool) -> None:
        """Enable/disable the SDK2 policy runner (no-op when not running in SDK2 mode)."""
        self._desired_policy_enabled = bool(enabled)
        if self._policy_runner is not None and hasattr(self._policy_runner, "set_enabled"):
            self._policy_runner.set_enabled(bool(enabled))

    def set_policy_estop(self, estop: bool) -> None:
        """Latch/unlatch E-stop on the SDK2 policy runner (no-op when not running in SDK2 mode)."""
        self._desired_policy_estop = bool(estop)
        if self._desired_policy_estop:
            # Match runner behavior: estop forces disabled.
            self._desired_policy_enabled = False
        if self._policy_runner is not None and hasattr(self._policy_runner, "set_estop"):
            self._policy_runner.set_estop(bool(estop))

    def set_policy_params_json(self, params_json: str) -> None:
        """Update policy params JSON (forwarded to policy runtime/adapters)."""
        self._desired_policy_params_json = str(params_json or "")
        if self._policy_runner is not None and hasattr(self._policy_runner, "set_policy_params_json"):
            self._policy_runner.set_policy_params_json(self._desired_policy_params_json)

    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        print(f"publishing request, topic={topic}, data={data}")
        return {}
