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

"""Python NativeModule wrapper for the FAST-LIO2 + Livox Mid-360 binary.

Binds Livox SDK2 directly into FAST-LIO-NON-ROS for real-time LiDAR SLAM.
Outputs registered (world-frame) point clouds and odometry with covariance.

Usage::

    from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2Module
    from dimos.core.blueprints import autoconnect

    autoconnect(
        FastLio2Module.blueprint(host_ip="192.168.1.5"),
        SomeConsumer.blueprint(),
    ).build().loop()
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
from pathlib import Path
import tempfile

from dimos.core import Out  # noqa: TC001
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.msgs.nav_msgs.Odometry import Odometry  # noqa: TC001
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2  # noqa: TC001

_DEFAULT_EXECUTABLE = str(Path(__file__).parent / "cpp" / "build" / "fastlio2_native")
_DEFAULT_CONFIG = str(Path(__file__).parent / "cpp" / "config" / "mid360.json")


@dataclass(kw_only=True)
class FastLio2Config(NativeModuleConfig):
    """Config for the FAST-LIO2 + Livox Mid-360 native module."""

    executable: str = _DEFAULT_EXECUTABLE

    # Livox SDK hardware config
    host_ip: str = "192.168.1.5"
    lidar_ip: str = "192.168.1.155"
    frequency: float = 10.0

    # Frame IDs for output messages
    frame_id: str = "map"
    child_frame_id: str = "body"

    # Output publish rates (Hz)
    pointcloud_freq: float = 10.0
    odom_freq: float = 30.0

    # FAST-LIO config (written to JSON, passed as --config_path)
    scan_line: int = 1
    blind: float = 1.0
    fov_degree: int = 360
    det_range: float = 100.0
    acc_cov: float = 0.1
    gyr_cov: float = 0.1
    b_acc_cov: float = 0.0001
    b_gyr_cov: float = 0.0001
    extrinsic_est_en: bool = True
    extrinsic_t: list[float] = field(default_factory=lambda: [0.04165, 0.02326, -0.0284])
    extrinsic_r: list[float] = field(
        default_factory=lambda: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    )

    # SDK port configuration (for multi-sensor setups)
    cmd_data_port: int = 56100
    push_msg_port: int = 56200
    point_data_port: int = 56300
    imu_data_port: int = 56400
    log_data_port: int = 56500
    host_cmd_data_port: int = 56101
    host_push_msg_port: int = 56201
    host_point_data_port: int = 56301
    host_imu_data_port: int = 56401
    host_log_data_port: int = 56501


class FastLio2(NativeModule):
    """FAST-LIO2 SLAM module with integrated Livox Mid-360 driver.

    Ports:
        lidar (Out[PointCloud2]): World-frame registered point cloud.
        odometry (Out[Odometry]): Pose with covariance at LiDAR scan rate.
    """

    default_config: type[FastLio2Config] = FastLio2Config  # type: ignore[assignment]
    lidar: Out[PointCloud2]
    odometry: Out[Odometry]

    def _build_extra_args(self) -> list[str]:
        """Pass hardware and SLAM config to the C++ binary as CLI args."""
        cfg: FastLio2Config = self.config  # type: ignore[assignment]
        config_path = self._write_fastlio_config(cfg)
        return [
            "--config_path",
            config_path,
            "--host_ip",
            cfg.host_ip,
            "--lidar_ip",
            cfg.lidar_ip,
            "--frequency",
            str(cfg.frequency),
            "--frame_id",
            cfg.frame_id,
            "--child_frame_id",
            cfg.child_frame_id,
            "--pointcloud_freq",
            str(cfg.pointcloud_freq),
            "--odom_freq",
            str(cfg.odom_freq),
            "--cmd_data_port",
            str(cfg.cmd_data_port),
            "--push_msg_port",
            str(cfg.push_msg_port),
            "--point_data_port",
            str(cfg.point_data_port),
            "--imu_data_port",
            str(cfg.imu_data_port),
            "--log_data_port",
            str(cfg.log_data_port),
            "--host_cmd_data_port",
            str(cfg.host_cmd_data_port),
            "--host_push_msg_port",
            str(cfg.host_push_msg_port),
            "--host_point_data_port",
            str(cfg.host_point_data_port),
            "--host_imu_data_port",
            str(cfg.host_imu_data_port),
            "--host_log_data_port",
            str(cfg.host_log_data_port),
        ]

    @staticmethod
    def _write_fastlio_config(cfg: FastLio2Config) -> str:
        """Write FAST-LIO JSON config to a temp file, return path."""
        config = {
            "common": {
                "time_sync_en": False,
                "time_offset_lidar_to_imu": 0.0,
                "msr_freq": 50.0,
                "main_freq": 5000.0,
            },
            "preprocess": {
                "lidar_type": 1,
                "scan_line": cfg.scan_line,
                "blind": cfg.blind,
            },
            "mapping": {
                "acc_cov": cfg.acc_cov,
                "gyr_cov": cfg.gyr_cov,
                "b_acc_cov": cfg.b_acc_cov,
                "b_gyr_cov": cfg.b_gyr_cov,
                "fov_degree": cfg.fov_degree,
                "det_range": cfg.det_range,
                "extrinsic_est_en": cfg.extrinsic_est_en,
                "extrinsic_T": list(cfg.extrinsic_t),
                "extrinsic_R": list(cfg.extrinsic_r),
            },
        }
        f = tempfile.NamedTemporaryFile(mode="w", suffix=".json", prefix="fastlio2_", delete=False)
        json.dump(config, f, indent=2)
        f.close()
        return f.name


fastlio2_module = FastLio2.blueprint

__all__ = [
    "FastLio2",
    "FastLio2Config",
    "fastlio2_module",
]
