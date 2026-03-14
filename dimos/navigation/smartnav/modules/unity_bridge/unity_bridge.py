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

"""UnityBridgeModule: TCP bridge to the CMU VLA Challenge Unity simulator.

Implements the ROS-TCP-Endpoint binary protocol to communicate with Unity
directly — no ROS dependency needed, no Unity-side changes.

Unity sends simulated sensor data (lidar PointCloud2, compressed camera images).
We send back vehicle PoseStamped updates so Unity renders the robot position.

Protocol (per message on the TCP stream):
  [4 bytes LE uint32] destination string length
  [N bytes]           destination string (topic name or __syscommand)
  [4 bytes LE uint32] message payload length
  [M bytes]           payload (ROS1-serialized message, or JSON for syscommands)
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import platform
from queue import Empty, Queue
import socket
import struct
import subprocess
import threading
import time
from typing import Any
import zipfile

import numpy as np
from pydantic import Field

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.smartnav.ros1_deserializer import (
    deserialize_compressed_image,
    deserialize_pointcloud2,
    serialize_pose_stamped,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
PI = math.pi

# Google Drive folder containing environment zips
_GDRIVE_FOLDER_ID = "1UD5v6cSfcwIMWmsq9WSk7blJut4kgb-1"
_DEFAULT_SCENE = "office_1"
_SUPPORTED_SYSTEMS = {"Linux"}
_SUPPORTED_ARCHS = {"x86_64", "AMD64"}


# ---------------------------------------------------------------------------
# TCP protocol helpers
# ---------------------------------------------------------------------------


def _recvall(sock: socket.socket, size: int) -> bytes:
    buf = bytearray(size)
    view = memoryview(buf)
    pos = 0
    while pos < size:
        n = sock.recv_into(view[pos:], size - pos)
        if not n:
            raise OSError("Connection closed")
        pos += n
    return bytes(buf)


def _read_tcp_message(sock: socket.socket) -> tuple[str, bytes]:
    dest_len = struct.unpack("<I", _recvall(sock, 4))[0]
    dest = _recvall(sock, dest_len).decode("utf-8").rstrip("\x00")
    msg_len = struct.unpack("<I", _recvall(sock, 4))[0]
    msg_data = _recvall(sock, msg_len) if msg_len > 0 else b""
    return dest, msg_data


def _write_tcp_message(sock: socket.socket, destination: str, data: bytes) -> None:
    dest_bytes = destination.encode("utf-8")
    sock.sendall(
        struct.pack("<I", len(dest_bytes)) + dest_bytes + struct.pack("<I", len(data)) + data
    )


def _write_tcp_command(sock: socket.socket, command: str, params: dict) -> None:
    dest_bytes = command.encode("utf-8")
    json_bytes = json.dumps(params).encode("utf-8")
    sock.sendall(
        struct.pack("<I", len(dest_bytes))
        + dest_bytes
        + struct.pack("<I", len(json_bytes))
        + json_bytes
    )


# ---------------------------------------------------------------------------
# Auto-download
# ---------------------------------------------------------------------------


def _download_unity_scene(scene: str, dest_dir: Path) -> Path:
    """Download a Unity environment zip from Google Drive and extract it.

    Returns the path to the Model.x86_64 binary.
    """
    try:
        import gdown  # type: ignore[import-untyped]
    except ImportError:
        raise RuntimeError(
            "Unity sim binary not found and 'gdown' is not installed for auto-download. "
            "Install it with: pip install gdown\n"
            "Or manually download from: "
            f"https://drive.google.com/drive/folders/{_GDRIVE_FOLDER_ID}"
        )

    dest_dir.mkdir(parents=True, exist_ok=True)
    zip_path = dest_dir / f"{scene}.zip"

    if not zip_path.exists():
        print("\n" + "=" * 70, flush=True)
        print(f"  DOWNLOADING UNITY SIMULATOR — scene: '{scene}'", flush=True)
        print("  Source: Google Drive (CMU VLA Challenge environments)", flush=True)
        print("  Size: ~130-580 MB per scene (depends on scene complexity)", flush=True)
        print(f"  Destination: {dest_dir}", flush=True)
        print("  This is a one-time download. Subsequent runs use the cache.", flush=True)
        print("=" * 70 + "\n", flush=True)
        gdown.download_folder(
            id=_GDRIVE_FOLDER_ID,
            output=str(dest_dir),
            quiet=False,
        )
        # gdown downloads all scenes into a subfolder; find our zip
        for candidate in dest_dir.rglob(f"{scene}.zip"):
            zip_path = candidate
            break

    if not zip_path.exists():
        raise FileNotFoundError(
            f"Failed to download scene '{scene}'. "
            f"Check https://drive.google.com/drive/folders/{_GDRIVE_FOLDER_ID}"
        )

    # Extract
    extract_dir = dest_dir / scene
    if not extract_dir.exists():
        logger.info(f"Extracting {zip_path}...")
        with zipfile.ZipFile(zip_path, "r") as zf:
            zf.extractall(dest_dir)

    binary = extract_dir / "environment" / "Model.x86_64"
    if not binary.exists():
        raise FileNotFoundError(
            f"Extracted scene but Model.x86_64 not found at {binary}. "
            f"Expected structure: {scene}/environment/Model.x86_64"
        )

    binary.chmod(binary.stat().st_mode | 0o111)
    return binary


# ---------------------------------------------------------------------------
# Platform validation
# ---------------------------------------------------------------------------


def _validate_platform() -> None:
    """Raise if the current platform can't run the Unity x86_64 binary."""
    system = platform.system()
    arch = platform.machine()

    if system not in _SUPPORTED_SYSTEMS:
        raise RuntimeError(
            f"Unity simulator requires Linux x86_64 but running on {system} {arch}. "
            f"macOS and Windows are not supported (the binary is a Linux ELF executable). "
            f"Use a Linux VM, Docker, or WSL2."
        )

    if arch not in _SUPPORTED_ARCHS:
        raise RuntimeError(
            f"Unity simulator requires x86_64 but running on {arch}. "
            f"ARM64 Linux is not supported. Use an x86_64 machine or emulation layer."
        )


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------


class UnityBridgeConfig(ModuleConfig):
    """Configuration for the Unity bridge / vehicle simulator.

    Set ``unity_binary=""`` to skip launching Unity and connect to an
    already-running instance. Set ``auto_download=True`` (default) to
    automatically download the scene if the binary is missing.
    """

    # Path to the Unity x86_64 binary. Relative paths resolved from cwd.
    # Leave empty to auto-detect from cache or auto-download.
    unity_binary: str = ""

    # Scene name for auto-download (e.g. "office_1", "hotel_room_1").
    # Only used when unity_binary is not found and auto_download is True.
    unity_scene: str = _DEFAULT_SCENE

    # Directory to download/cache Unity scenes.
    unity_cache_dir: str = "~/.cache/smartnav/unity_envs"

    # Auto-download the scene from Google Drive if binary is missing.
    auto_download: bool = True

    # Max seconds to wait for Unity to connect after launch.
    unity_connect_timeout: float = 30.0

    # TCP server settings (we listen; Unity connects to us).
    unity_host: str = "0.0.0.0"
    unity_port: int = 10000

    # Run Unity with no visible window (set -batchmode -nographics).
    # Note: headless mode may not produce camera images.
    headless: bool = False

    # Extra CLI args to pass to the Unity binary.
    unity_extra_args: list[str] = Field(default_factory=list)

    # Vehicle parameters
    sensor_offset_x: float = 0.0
    sensor_offset_y: float = 0.0
    vehicle_height: float = 0.75

    # Initial vehicle pose
    init_x: float = 0.0
    init_y: float = 0.0
    init_z: float = 0.0
    init_yaw: float = 0.0

    # Kinematic sim rate (Hz) for odometry integration
    sim_rate: float = 200.0


# ---------------------------------------------------------------------------
# Module
# ---------------------------------------------------------------------------


class UnityBridgeModule(Module[UnityBridgeConfig]):
    """TCP bridge to the Unity simulator with kinematic odometry integration.

    Ports:
        cmd_vel (In[Twist]): Velocity commands.
        terrain_map (In[PointCloud2]): Terrain for Z adjustment.
        odometry (Out[Odometry]): Vehicle state at sim_rate.
        registered_scan (Out[PointCloud2]): Lidar from Unity.
        color_image (Out[Image]): RGB camera from Unity (1920x640 panoramic).
        semantic_image (Out[Image]): Semantic segmentation from Unity.
        camera_info (Out[CameraInfo]): Camera intrinsics.
    """

    default_config = UnityBridgeConfig

    cmd_vel: In[Twist]
    terrain_map: In[PointCloud2]
    odometry: Out[Odometry]
    registered_scan: Out[PointCloud2]
    color_image: Out[Image]
    semantic_image: Out[Image]
    camera_info: Out[CameraInfo]

    # Rerun static config for 3D camera projection — use this when building
    # your rerun_config so the panoramic image renders correctly in 3D.
    #
    # Usage:
    #   rerun_config = {
    #       "static": {"world/color_image": UnityBridgeModule.rerun_static_pinhole},
    #       "visual_override": {"world/camera_info": UnityBridgeModule.rerun_suppress_camera_info},
    #   }
    @staticmethod
    def rerun_static_pinhole(rr: Any) -> list[Any]:
        """Static Pinhole + Transform3D for the Unity panoramic camera."""
        width, height = 1920, 640
        hfov_rad = math.radians(120.0)
        fx = (width / 2.0) / math.tan(hfov_rad / 2.0)
        fy = fx
        cx, cy = width / 2.0, height / 2.0
        return [
            rr.Pinhole(
                resolution=[width, height],
                focal_length=[fx, fy],
                principal_point=[cx, cy],
                camera_xyz=rr.ViewCoordinates.RDF,
            ),
            rr.Transform3D(
                parent_frame="tf#/sensor",
                translation=[0.0, 0.0, 0.1],
                rotation=rr.Quaternion(xyzw=[0.5, -0.5, 0.5, -0.5]),
            ),
        ]

    @staticmethod
    def rerun_suppress_camera_info(_: Any) -> None:
        """Suppress CameraInfo logging — the static pinhole handles 3D projection."""
        return None

    # ---- lifecycle --------------------------------------------------------

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._x = self.config.init_x
        self._y = self.config.init_y
        self._z = self.config.init_z + self.config.vehicle_height
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = self.config.init_yaw
        self._terrain_z = self.config.init_z
        self._fwd_speed = 0.0
        self._left_speed = 0.0
        self._yaw_rate = 0.0
        self._cmd_lock = threading.Lock()
        self._running = False
        self._sim_thread: threading.Thread | None = None
        self._unity_thread: threading.Thread | None = None
        self._unity_connected = False
        self._unity_ready = threading.Event()
        self._unity_process: subprocess.Popen | None = None  # type: ignore[type-arg]
        self._send_queue: Queue[tuple[str, bytes]] = Queue()

    def __getstate__(self) -> dict:
        state = super().__getstate__()
        for key in (
            "_cmd_lock",
            "_sim_thread",
            "_unity_thread",
            "_unity_process",
            "_send_queue",
            "_unity_ready",
        ):
            state.pop(key, None)
        return state

    def __setstate__(self, state: dict) -> None:
        super().__setstate__(state)
        self._cmd_lock = threading.Lock()
        self._sim_thread = None
        self._unity_thread = None
        self._unity_process = None
        self._send_queue = Queue()
        self._unity_ready = threading.Event()
        self._running = False

    def start(self) -> None:
        self.cmd_vel._transport.subscribe(self._on_cmd_vel)
        self.terrain_map._transport.subscribe(self._on_terrain)
        self._running = True
        self._sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
        self._sim_thread.start()
        self._unity_thread = threading.Thread(target=self._unity_loop, daemon=True)
        self._unity_thread.start()
        self._launch_unity()

    def stop(self) -> None:
        self._running = False
        if self._sim_thread:
            self._sim_thread.join(timeout=2.0)
        if self._unity_thread:
            self._unity_thread.join(timeout=2.0)
        if self._unity_process is not None and self._unity_process.poll() is None:
            import signal as _sig

            logger.info(f"Stopping Unity (pid={self._unity_process.pid})")
            self._unity_process.send_signal(_sig.SIGTERM)
            try:
                self._unity_process.wait(timeout=5)
            except Exception:
                self._unity_process.kill()
            self._unity_process = None
        super().stop()

    # ---- Unity process management -----------------------------------------

    def _resolve_binary(self) -> Path | None:
        """Find the Unity binary, downloading if needed. Returns None to skip launch."""
        cfg = self.config

        # Explicit path provided
        if cfg.unity_binary:
            p = Path(cfg.unity_binary)
            if not p.is_absolute():
                p = Path.cwd() / p
                if not p.exists():
                    p = (Path(__file__).resolve().parent / cfg.unity_binary).resolve()
            if p.exists():
                return p
            if not cfg.auto_download:
                logger.error(
                    f"Unity binary not found at {p} and auto_download is disabled. "
                    f"Set unity_binary to a valid path or enable auto_download."
                )
                return None

        # Auto-download
        if cfg.auto_download:
            _validate_platform()
            cache = Path(cfg.unity_cache_dir).expanduser()
            candidate = cache / cfg.unity_scene / "environment" / "Model.x86_64"
            if candidate.exists():
                return candidate
            logger.info(f"Unity binary not found, downloading scene '{cfg.unity_scene}'...")
            return _download_unity_scene(cfg.unity_scene, cache)

        return None

    def _launch_unity(self) -> None:
        """Launch the Unity simulator binary as a subprocess."""
        binary_path = self._resolve_binary()
        if binary_path is None:
            logger.info("No Unity binary — TCP server will wait for external connection")
            return

        _validate_platform()

        if not os.access(binary_path, os.X_OK):
            binary_path.chmod(binary_path.stat().st_mode | 0o111)

        cmd = [str(binary_path)]
        if self.config.headless:
            cmd.extend(["-batchmode", "-nographics"])
        cmd.extend(self.config.unity_extra_args)

        logger.info(f"Launching Unity: {' '.join(cmd)}")
        env = {**os.environ}
        if "DISPLAY" not in env and not self.config.headless:
            env["DISPLAY"] = ":0"

        self._unity_process = subprocess.Popen(
            cmd,
            cwd=str(binary_path.parent),
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        logger.info(f"Unity pid={self._unity_process.pid}, waiting for TCP connection...")

        if self._unity_ready.wait(timeout=self.config.unity_connect_timeout):
            logger.info("Unity connected")
        else:
            # Check if process died
            rc = self._unity_process.poll()
            if rc is not None:
                logger.error(
                    f"Unity process exited with code {rc} before connecting. "
                    f"Check that DISPLAY is set and the binary is not corrupted."
                )
            else:
                logger.warning(
                    f"Unity did not connect within {self.config.unity_connect_timeout}s. "
                    f"The binary may still be loading — it will connect when ready."
                )

    # ---- input callbacks --------------------------------------------------

    def _on_cmd_vel(self, twist: Twist) -> None:
        with self._cmd_lock:
            self._fwd_speed = twist.linear.x
            self._left_speed = twist.linear.y
            self._yaw_rate = twist.angular.z

    def _on_terrain(self, cloud: PointCloud2) -> None:
        points, _ = cloud.as_numpy()
        if len(points) == 0:
            return
        dx = points[:, 0] - self._x
        dy = points[:, 1] - self._y
        near = points[np.sqrt(dx * dx + dy * dy) < 0.5]
        if len(near) >= 10:
            self._terrain_z = 0.8 * self._terrain_z + 0.2 * near[:, 2].mean()

    # ---- Unity TCP bridge -------------------------------------------------

    def _unity_loop(self) -> None:
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((self.config.unity_host, self.config.unity_port))
        server_sock.listen(1)
        server_sock.settimeout(2.0)
        logger.info(f"TCP server on :{self.config.unity_port}")

        while self._running:
            try:
                conn, addr = server_sock.accept()
                logger.info(f"Unity connected from {addr}")
                try:
                    self._bridge_connection(conn)
                except Exception as e:
                    logger.info(f"Unity connection ended: {e}")
                finally:
                    self._unity_connected = False
                    conn.close()
            except TimeoutError:
                continue
            except Exception as e:
                if self._running:
                    logger.warning(f"TCP server error: {e}")
                    time.sleep(1.0)

        server_sock.close()

    def _bridge_connection(self, sock: socket.socket) -> None:
        sock.settimeout(None)
        self._unity_connected = True
        self._unity_ready.set()

        _write_tcp_command(
            sock,
            "__handshake",
            {
                "version": "v0.7.0",
                "metadata": json.dumps({"protocol": "ROS2"}),
            },
        )

        halt = threading.Event()
        sender = threading.Thread(target=self._unity_sender, args=(sock, halt), daemon=True)
        sender.start()

        try:
            while self._running and not halt.is_set():
                dest, data = _read_tcp_message(sock)
                if dest == "":
                    continue
                elif dest.startswith("__"):
                    self._handle_syscommand(dest, data)
                else:
                    self._handle_unity_message(dest, data)
        finally:
            halt.set()
            self._unity_connected = False

    def _unity_sender(self, sock: socket.socket, halt: threading.Event) -> None:
        while not halt.is_set():
            try:
                dest, data = self._send_queue.get(timeout=1.0)
                if dest == "__raw__":
                    sock.sendall(data)
                else:
                    _write_tcp_message(sock, dest, data)
            except Empty:
                continue
            except Exception:
                halt.set()

    def _handle_syscommand(self, dest: str, data: bytes) -> None:
        payload = data.rstrip(b"\x00")
        try:
            params = json.loads(payload.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            params = {}

        cmd = dest[2:]
        logger.info(f"Unity syscmd: {cmd} {params}")

        if cmd == "topic_list":
            resp = json.dumps(
                {
                    "topics": ["/unity_sim/set_model_state", "/tf"],
                    "types": ["geometry_msgs/PoseStamped", "tf2_msgs/TFMessage"],
                }
            ).encode("utf-8")
            hdr = b"__topic_list"
            frame = struct.pack("<I", len(hdr)) + hdr + struct.pack("<I", len(resp)) + resp
            self._send_queue.put(("__raw__", frame))

    def _handle_unity_message(self, topic: str, data: bytes) -> None:
        if topic == "/registered_scan":
            result = deserialize_pointcloud2(data)
            if result is not None:
                points, frame_id, ts = result
                if len(points) > 0:
                    self.registered_scan._transport.publish(
                        PointCloud2.from_numpy(points, frame_id=frame_id, timestamp=ts)
                    )

        elif "image" in topic and "compressed" in topic:
            result = deserialize_compressed_image(data)
            if result is not None:
                img_bytes, _fmt, _frame_id, ts = result
                try:
                    import cv2

                    decoded = cv2.imdecode(np.frombuffer(img_bytes, np.uint8), cv2.IMREAD_COLOR)
                    if decoded is not None:
                        img = Image.from_numpy(decoded, frame_id="camera", ts=ts)
                        if "semantic" in topic:
                            pass  # skip semantic image to reduce bandwidth
                        else:
                            self.color_image._transport.publish(img)
                            h, w = decoded.shape[:2]
                            self._publish_camera_info(w, h, ts)
                except Exception as e:
                    logger.warning(f"Image decode failed ({topic}): {e}")

    def _publish_camera_info(self, width: int, height: int, ts: float) -> None:
        fx = fy = height / 2.0
        cx, cy = width / 2.0, height / 2.0
        self.camera_info._transport.publish(
            CameraInfo(
                height=height,
                width=width,
                distortion_model="plumb_bob",
                D=[0.0, 0.0, 0.0, 0.0, 0.0],
                K=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
                R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                P=[fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
                frame_id="camera",
                ts=ts,
            )
        )

    def _send_to_unity(self, topic: str, data: bytes) -> None:
        if self._unity_connected:
            self._send_queue.put((topic, data))

    # ---- kinematic sim loop -----------------------------------------------

    def _sim_loop(self) -> None:
        dt = 1.0 / self.config.sim_rate

        while self._running:
            t0 = time.monotonic()

            with self._cmd_lock:
                fwd, left, yaw_rate = self._fwd_speed, self._left_speed, self._yaw_rate

            prev_z = self._z

            self._yaw += dt * yaw_rate
            if self._yaw > PI:
                self._yaw -= 2 * PI
            elif self._yaw < -PI:
                self._yaw += 2 * PI

            cy, sy = math.cos(self._yaw), math.sin(self._yaw)
            self._x += dt * cy * fwd - dt * sy * left
            self._y += dt * sy * fwd + dt * cy * left
            self._z = self._terrain_z + self.config.vehicle_height

            now = time.time()
            quat = Quaternion.from_euler(Vector3(self._roll, self._pitch, self._yaw))

            self.odometry._transport.publish(
                Odometry(
                    ts=now,
                    frame_id="map",
                    child_frame_id="sensor",
                    pose=Pose(
                        position=[self._x, self._y, self._z],
                        orientation=[quat.x, quat.y, quat.z, quat.w],
                    ),
                    twist=Twist(
                        linear=[fwd, left, (self._z - prev_z) * self.config.sim_rate],
                        angular=[0.0, 0.0, yaw_rate],
                    ),
                )
            )

            self.tf.publish(
                Transform(
                    translation=Vector3(self._x, self._y, self._z),
                    rotation=quat,
                    frame_id="map",
                    child_frame_id="sensor",
                    ts=now,
                ),
                Transform(
                    translation=Vector3(0.0, 0.0, 0.0),
                    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                    frame_id="map",
                    child_frame_id="world",
                    ts=now,
                ),
            )

            if self._unity_connected:
                self._send_to_unity(
                    "/unity_sim/set_model_state",
                    serialize_pose_stamped(
                        self._x,
                        self._y,
                        self._z,
                        quat.x,
                        quat.y,
                        quat.z,
                        quat.w,
                    ),
                )

            sleep_for = dt - (time.monotonic() - t0)
            if sleep_for > 0:
                time.sleep(sleep_for)
