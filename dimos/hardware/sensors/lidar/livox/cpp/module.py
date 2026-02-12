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

"""Python NativeModule wrapper for the C++ Livox Mid-360 driver.

Declares the same ports as LivoxLidarModule (pointcloud, imu) but delegates
all real work to the ``mid360_native`` C++ binary, which talks directly to
the Livox SDK2 C API and publishes on LCM.

Usage::

    from dimos.hardware.sensors.lidar.livox.cpp.module import Mid360CppModule
    from dimos.core.blueprints import autoconnect

    autoconnect(
        Mid360CppModule.blueprint(host_ip="192.168.1.5"),
        SomeConsumer.blueprint(),
    ).build().loop()
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.spec import perception

if TYPE_CHECKING:
    from dimos.core import Out
    from dimos.msgs.sensor_msgs.Imu import Imu
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

_DEFAULT_EXECUTABLE = str(Path(__file__).parent / "build" / "mid360_native")


@dataclass(kw_only=True)
class Mid360CppConfig(NativeModuleConfig):
    """Config for the C++ Mid-360 native module."""

    executable: str = _DEFAULT_EXECUTABLE
    host_ip: str = "192.168.1.5"
    lidar_ip: str = "192.168.1.155"
    frequency: float = 10.0
    enable_imu: bool = True
    frame_id: str = "lidar_link"
    imu_frame_id: str = "imu_link"

    # SDK port configuration (match defaults in LivoxMid360Config)
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


class Mid360CppModule(NativeModule, perception.Lidar, perception.IMU):
    """Livox Mid-360 LiDAR module backed by a native C++ binary.

    Ports:
        pointcloud (Out[PointCloud2]): Point cloud frames at configured frequency.
        imu (Out[Imu]): IMU data at ~200 Hz (if enabled).
    """

    default_config: type[Mid360CppConfig] = Mid360CppConfig  # type: ignore[assignment]
    pointcloud: Out[PointCloud2]
    imu: Out[Imu]

    def _collect_topics(self) -> dict[str, str]:
        cfg: Mid360CppConfig = self.config  # type: ignore[assignment]
        topics = super()._collect_topics()
        if not cfg.enable_imu:
            topics.pop("imu", None)
        return topics

    def _build_extra_args(self) -> list[str]:
        """Pass hardware config to the C++ binary as CLI args."""
        cfg: Mid360CppConfig = self.config  # type: ignore[assignment]
        return [
            "--host_ip",
            cfg.host_ip,
            "--lidar_ip",
            cfg.lidar_ip,
            "--frequency",
            str(cfg.frequency),
            "--frame_id",
            cfg.frame_id,
            "--imu_frame_id",
            cfg.imu_frame_id,
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


mid360_cpp_module = Mid360CppModule.blueprint

__all__ = [
    "Mid360CppConfig",
    "Mid360CppModule",
    "mid360_cpp_module",
]
