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

"""DimSimModule: Browser-based 3D simulator bridge with Unity-compatible interface.

Wraps the DimSim CLI (from jeff-hykin/DimSim fork) as a subprocess, subscribes
to its LCM sensor output, and exposes the same stream interface as
UnityBridgeModule — making it a drop-in replacement for nav/planning tests
that need to run on macOS and ARM Linux.

Data flow:
    DimSim subprocess → LCM (odom, lidar, images) → DimSimModule Out ports
    DimSimModule In[cmd_vel] → LCM → DimSim subprocess

Uses --topic-remap to namespace LCM channels, enabling multiple instances.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time
from typing import Any

import lcm as lcmlib
import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# -- GitHub release info for auto-download ------------------------------------
_GITHUB_REPO = "jeff-hykin/DimSim"
_RELEASES_API = f"https://api.github.com/repos/{_GITHUB_REPO}/releases/latest"

# -- Camera defaults (DimSim default resolution 640x288, 80° HFOV) -----------
_DEFAULT_CAM_W = 640
_DEFAULT_CAM_H = 288
_DEFAULT_CAM_HFOV_DEG = 80
_DEFAULT_CAM_FX = _DEFAULT_CAM_W / (2 * math.tan(math.radians(_DEFAULT_CAM_HFOV_DEG / 2)))
_DEFAULT_CAM_FY = _DEFAULT_CAM_FX
_DEFAULT_CAM_CX = _DEFAULT_CAM_W / 2
_DEFAULT_CAM_CY = _DEFAULT_CAM_H / 2


def _dimsim_bin() -> Path:
    """Path to the dimsim compiled binary."""
    return Path.home() / ".dimsim" / "bin" / "dimsim"


def _find_local_cli() -> Path | None:
    """Find local DimSim/dimos-cli/cli.ts for development."""
    # Check ~/repos/DimSim first (jeff's fork location)
    candidate = Path.home() / "repos" / "DimSim" / "dimos-cli" / "cli.ts"
    if candidate.exists():
        return candidate
    # Fall back to sibling of dimos repo
    repo_root = Path(__file__).resolve().parents[4]
    candidate = repo_root / "DimSim" / "dimos-cli" / "cli.ts"
    return candidate if candidate.exists() else None


class DimSimModuleConfig(ModuleConfig):
    """Configuration for the DimSim simulator module."""

    scene: str = "apt"
    port: int = 8090
    local: bool = False

    # Sensor publish rates (ms). None = use DimSim defaults.
    image_rate_ms: int | None = None
    lidar_rate_ms: int | None = None
    odom_rate_ms: int | None = None

    enable_depth: bool = True
    camera_fov: int | None = None
    headless: bool = False
    render: str = "cpu"

    # Sim parameters (matching UnityBridgeConfig interface)
    sim_rate: float = 50.0
    vehicle_height: float = 0.75
    init_x: float = 0.0
    init_y: float = 0.0
    init_z: float = 0.0
    init_yaw: float = 0.0

    # Topic namespace prefix for multi-instance isolation
    topic_prefix: str = ""

    # LCM URL override (defaults to standard multicast)
    lcm_url: str = ""


class DimSimModule(Module):
    """Browser-based 3D simulator with Unity-compatible stream interface.

    Ports (matching UnityBridgeModule):
        cmd_vel (In[Twist]): Velocity commands.
        terrain_map (In[PointCloud2]): Terrain for Z adjustment (accepted but unused).
        odometry (Out[Odometry]): Vehicle state (converted from DimSim PoseStamped).
        registered_scan (Out[PointCloud2]): Lidar pointcloud.
        color_image (Out[Image]): RGB camera.
        camera_info (Out[CameraInfo]): Camera intrinsics.
    """

    config: DimSimModuleConfig

    cmd_vel: In[Twist]
    terrain_map: In[PointCloud2]
    odometry: Out[Odometry]
    registered_scan: Out[PointCloud2]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    @staticmethod
    def rerun_blueprint() -> Any:
        """3D world view for DimSim visualization."""
        import rerun.blueprint as rrb

        return rrb.Blueprint(
            rrb.Vertical(
                rrb.Spatial3DView(
                    origin="world",
                    name="3D",
                    eye_controls=rrb.EyeControls3D(
                        position=(0.0, 0.0, 20.0),
                        look_target=(0.0, 0.0, 0.0),
                        eye_up=(0.0, 0.0, 1.0),
                    ),
                ),
            ),
            collapse_panels=True,
        )

    @staticmethod
    def rerun_static_pinhole(rr: Any) -> list[Any]:
        """Static Pinhole + Transform3D for the DimSim camera."""
        return [
            rr.Pinhole(
                resolution=[_DEFAULT_CAM_W, _DEFAULT_CAM_H],
                focal_length=[_DEFAULT_CAM_FX, _DEFAULT_CAM_FY],
                principal_point=[_DEFAULT_CAM_CX, _DEFAULT_CAM_CY],
                camera_xyz=rr.ViewCoordinates.RDF,
            ),
            rr.Transform3D(
                parent_frame="tf#/sensor",
                translation=[0.3, 0.0, 0.0],
                rotation=rr.Quaternion(xyzw=[0.5, -0.5, 0.5, -0.5]),
            ),
        ]

    @staticmethod
    def rerun_suppress_camera_info(_: Any) -> None:
        """Suppress CameraInfo logging — the static pinhole handles 3D projection."""
        return None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._running = threading.Event()
        self._process: subprocess.Popen | None = None  # type: ignore[type-arg]
        self._lcm: lcmlib.LCM | None = None
        self._lcm_thread: threading.Thread | None = None
        self._caminfo_thread: threading.Thread | None = None

        # Velocity state for odometry twist (updated by cmd_vel subscriber)
        self._fwd_speed = 0.0
        self._left_speed = 0.0
        self._yaw_rate = 0.0
        self._cmd_lock = threading.Lock()

        # Previous odom for velocity estimation
        self._prev_odom_time: float | None = None
        self._prev_x = 0.0
        self._prev_y = 0.0
        self._prev_yaw = 0.0

        # Build topic names (with optional prefix for multi-instance)
        p = self.config.topic_prefix
        self._topic_odom = f"{p}/odom#geometry_msgs.PoseStamped"
        self._topic_lidar = f"{p}/lidar#sensor_msgs.PointCloud2"
        self._topic_color = f"{p}/color_image#sensor_msgs.Image"
        self._topic_depth = f"{p}/depth_image#sensor_msgs.Image"
        self._topic_cmd_vel = f"{p}/cmd_vel#geometry_msgs.Twist"
        self._topic_camera_info = f"{p}/camera_info#sensor_msgs.CameraInfo"

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))
        self._running.set()

        # Launch subprocess
        threading.Thread(target=self._launch_dimsim, daemon=True).start()

        # Start LCM listener
        lcm_url = self.config.lcm_url or os.environ.get("LCM_DEFAULT_URL", "udpm://239.255.76.67:7667?ttl=0")
        self._lcm = lcmlib.LCM(lcm_url)
        self._lcm.subscribe(self._topic_odom, self._on_lcm_odom)
        self._lcm.subscribe(self._topic_lidar, self._on_lcm_lidar)
        self._lcm.subscribe(self._topic_color, self._on_lcm_color_image)

        self._lcm_thread = threading.Thread(target=self._lcm_loop, daemon=True)
        self._lcm_thread.start()

        # Publish camera info at 1 Hz
        self._caminfo_thread = threading.Thread(target=self._caminfo_loop, daemon=True)
        self._caminfo_thread.start()

    @rpc
    def stop(self) -> None:
        self._running.clear()
        if self._lcm_thread:
            self._lcm_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        if self._caminfo_thread:
            self._caminfo_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        proc = self._process
        self._process = None
        if proc is not None and proc.poll() is None:
            logger.info(f"Stopping DimSim (pid={proc.pid})")
            proc.send_signal(signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                logger.warning(f"DimSim pid={proc.pid} did not exit after SIGTERM, killing")
                proc.kill()
        super().stop()

    # -- Subprocess management -------------------------------------------------

    def _build_cli_args(self) -> list[str]:
        """Build CLI arguments for the dimsim subprocess."""
        cfg = self.config
        args = ["dev", "--scene", cfg.scene, "--port", str(cfg.port)]

        if cfg.image_rate_ms is not None:
            args.extend(["--image-rate", str(cfg.image_rate_ms)])
        if cfg.lidar_rate_ms is not None:
            args.extend(["--lidar-rate", str(cfg.lidar_rate_ms)])
        if cfg.odom_rate_ms is not None:
            args.extend(["--odom-rate", str(cfg.odom_rate_ms)])
        if not cfg.enable_depth:
            args.append("--no-depth")
        if cfg.camera_fov is not None:
            args.extend(["--camera-fov", str(cfg.camera_fov)])
        if cfg.headless or os.environ.get("DIMSIM_HEADLESS", "").strip() in ("1", "true"):
            render = os.environ.get("DIMSIM_RENDER", cfg.render).strip()
            args.extend(["--headless", "--render", render])

        # Topic remapping
        if cfg.topic_prefix:
            remap_pairs = [
                f"/odom={cfg.topic_prefix}/odom",
                f"/lidar={cfg.topic_prefix}/lidar",
                f"/color_image={cfg.topic_prefix}/color_image",
                f"/depth_image={cfg.topic_prefix}/depth_image",
                f"/cmd_vel={cfg.topic_prefix}/cmd_vel",
                f"/camera_info={cfg.topic_prefix}/camera_info",
            ]
            args.extend(["--topic-remap", ",".join(remap_pairs)])

        return args

    def _resolve_executable(self) -> tuple[str, list[str]]:
        """Resolve the dimsim executable and any prefix args (e.g. deno run)."""
        use_local = self.config.local or os.environ.get("DIMSIM_LOCAL", "").strip() in ("1", "true")

        if use_local:
            cli_ts = _find_local_cli()
            if not cli_ts:
                raise FileNotFoundError(
                    "Local DimSim not found. Expected ~/repos/DimSim/dimos-cli/cli.ts"
                )
            deno = shutil.which("deno") or str(Path.home() / ".deno" / "bin" / "deno")
            return deno, ["run", "--allow-all", "--unstable-net", str(cli_ts)]

        dimsim = _dimsim_bin()
        if dimsim.exists():
            return str(dimsim), []

        path_dimsim = shutil.which("dimsim")
        if path_dimsim:
            return path_dimsim, []

        # Try auto-download
        self._download_binary()
        if dimsim.exists():
            return str(dimsim), []

        raise FileNotFoundError(
            "dimsim not found. Install via: deno install -gAf jsr:@antim/dimsim"
        )

    def _download_binary(self) -> None:
        """Download compiled dimsim binary from GitHub Releases."""
        import json
        import platform as plat
        import stat
        import urllib.request

        dimsim = _dimsim_bin()
        dimsim.parent.mkdir(parents=True, exist_ok=True)

        try:
            req = urllib.request.Request(
                _RELEASES_API,
                headers={"Accept": "application/vnd.github.v3+json"},
            )
            with urllib.request.urlopen(req, timeout=10) as resp:
                data = json.loads(resp.read())
                release_tag = data["tag_name"]
        except Exception as e:
            logger.warning(f"Could not fetch DimSim releases: {e}")
            return

        system = plat.system().lower()
        machine = plat.machine().lower()
        if system == "darwin" and machine in ("arm64", "aarch64"):
            binary_name = "dimsim-darwin-arm64"
        elif system == "darwin":
            binary_name = "dimsim-darwin-x64"
        elif system == "linux" and machine in ("x86_64", "amd64"):
            binary_name = "dimsim-linux-x64"
        else:
            logger.warning(f"No prebuilt binary for {system}/{machine}")
            return

        url = f"https://github.com/{_GITHUB_REPO}/releases/download/{release_tag}/{binary_name}"
        try:
            logger.info(f"Downloading dimsim from {_GITHUB_REPO}...")
            urllib.request.urlretrieve(url, str(dimsim))
            dimsim.chmod(dimsim.stat().st_mode | stat.S_IEXEC | stat.S_IXGRP | stat.S_IXOTH)
            if system == "darwin":
                subprocess.run(["xattr", "-c", str(dimsim)], capture_output=True)
            logger.info("dimsim binary installed.")
        except Exception as e:
            logger.warning(f"Binary download failed: {e}")

    def _launch_dimsim(self) -> None:
        """Launch the DimSim subprocess."""
        try:
            exe, prefix_args = self._resolve_executable()
        except FileNotFoundError as e:
            logger.error(str(e))
            return

        # Ensure setup and scene are installed
        exe_path = exe
        try:
            subprocess.run([exe_path, *prefix_args[:0], "setup"], check=True, capture_output=True)
            subprocess.run(
                [exe_path, *prefix_args[:0], "scene", "install", self.config.scene],
                check=True, capture_output=True,
            )
        except Exception:
            pass  # setup/scene install may fail if using local dev mode

        cli_args = self._build_cli_args()
        cmd = [exe, *prefix_args, *cli_args]
        logger.info(f"Launching DimSim: {' '.join(cmd)}")

        env = {**os.environ}
        self._process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env,
        )

        # Drain stderr in background
        def _drain() -> None:
            proc = self._process
            if proc and proc.stderr:
                for line in proc.stderr:
                    if self._running.is_set():
                        logger.debug(f"[dimsim] {line.decode(errors='replace').rstrip()}")

        threading.Thread(target=_drain, daemon=True).start()

    # -- LCM listener ----------------------------------------------------------

    def _lcm_loop(self) -> None:
        """Poll LCM for incoming messages from the DimSim subprocess."""
        while self._running.is_set():
            try:
                if self._lcm:
                    self._lcm.handle_timeout(100)
            except Exception:
                pass

    def _on_lcm_odom(self, channel: str, data: bytes) -> None:
        """Convert DimSim PoseStamped to Odometry and publish."""
        try:
            pose_stamped = PoseStamped.lcm_decode(data)
        except Exception:
            return

        now = time.time()

        # Extract position and orientation
        x = pose_stamped.x
        y = pose_stamped.y
        z = pose_stamped.z
        qx = pose_stamped.qx
        qy = pose_stamped.qy
        qz = pose_stamped.qz
        qw = pose_stamped.qw

        # Estimate yaw from quaternion
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        # Estimate velocity from position deltas
        vx = 0.0
        vy = 0.0
        vyaw = 0.0
        if self._prev_odom_time is not None:
            dt = now - self._prev_odom_time
            if dt > 0.001:
                dx = x - self._prev_x
                dy = y - self._prev_y
                # Transform world-frame velocity to body-frame
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)
                vx = (dx * cos_yaw + dy * sin_yaw) / dt
                vy = (-dx * sin_yaw + dy * cos_yaw) / dt
                dyaw = yaw - self._prev_yaw
                # Normalize angle delta
                while dyaw > math.pi:
                    dyaw -= 2 * math.pi
                while dyaw < -math.pi:
                    dyaw += 2 * math.pi
                vyaw = dyaw / dt

        self._prev_odom_time = now
        self._prev_x = x
        self._prev_y = y
        self._prev_yaw = yaw

        # Build Odometry message
        odom = Odometry(
            ts=pose_stamped.ts,
            frame_id="world",
            child_frame_id="base_link",
            x=x, y=y, z=z,
            qx=qx, qy=qy, qz=qz, qw=qw,
            vx=vx, vy=vy, vz=0.0,
            wx=0.0, wy=0.0, wz=vyaw,
        )
        self.odometry.publish(odom)

        # Publish TF: world -> base_link
        self.tf.publish(
            Transform(
                ts=pose_stamped.ts,
                parent_frame_id="world",
                child_frame_id="base_link",
                translation=Vector3(x=x, y=y, z=z),
                rotation=Quaternion(x=qx, y=qy, z=qz, w=qw),
            )
        )

        # Publish sensor frame transforms
        self.tf.publish(
            Transform(
                ts=pose_stamped.ts,
                parent_frame_id="base_link",
                child_frame_id="sensor",
                translation=Vector3(x=0.3, y=0.0, z=0.0),
                rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )

    def _on_lcm_lidar(self, channel: str, data: bytes) -> None:
        """Forward lidar PointCloud2 from DimSim to registered_scan output."""
        try:
            cloud = PointCloud2.lcm_decode(data)
            self.registered_scan.publish(cloud)
        except Exception as e:
            logger.debug(f"Lidar decode error: {e}")

    def _on_lcm_color_image(self, channel: str, data: bytes) -> None:
        """Forward color image from DimSim."""
        try:
            img = Image.lcm_decode(data)
            self.color_image.publish(img)
        except Exception as e:
            logger.debug(f"Image decode error: {e}")

    # -- cmd_vel → LCM ----------------------------------------------------------

    def _on_cmd_vel(self, twist: Twist) -> None:
        """Forward velocity commands to DimSim via LCM."""
        with self._cmd_lock:
            self._fwd_speed = twist.linear.x
            self._left_speed = twist.linear.y
            self._yaw_rate = twist.angular.z

        if self._lcm:
            try:
                encoded = twist.lcm_encode()
                self._lcm.publish(self._topic_cmd_vel, encoded)
            except Exception:
                pass

    # -- Camera info publishing -------------------------------------------------

    def _caminfo_loop(self) -> None:
        """Publish CameraInfo at 1 Hz."""
        while self._running.is_set():
            self._publish_camera_info()
            time.sleep(1.0)

    def _publish_camera_info(self) -> None:
        """Publish static camera intrinsics."""
        fov_deg = self.config.camera_fov or _DEFAULT_CAM_HFOV_DEG
        w, h = _DEFAULT_CAM_W, _DEFAULT_CAM_H
        fx = w / (2 * math.tan(math.radians(fov_deg / 2)))
        fy = fx
        cx, cy = w / 2, h / 2

        info = CameraInfo(
            ts=time.time(),
            frame_id="camera_optical",
            height=h,
            width=w,
            distortion_model="plumb_bob",
            D=[0.0, 0.0, 0.0, 0.0, 0.0],
            K=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
            R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            P=[fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        )
        self.camera_info.publish(info)


# Blueprint factory
dimsim_module = DimSimModule.blueprint

__all__ = ["DimSimModule", "DimSimModuleConfig", "dimsim_module"]
