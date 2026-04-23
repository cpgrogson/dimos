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

"""AriseSlam NativeModule: tightly-coupled LiDAR-inertial SLAM.

Reimplements https://github.com/YWL0720/ARISE-SLAM as a decoupled DimOS
NativeModule.  Consumes raw lidar scans + IMU and produces corrected
odometry and registered point clouds.

Key features:
  - Tightly-coupled LiDAR-inertial odometry with IMU preintegration
  - Multi-sensor support (Livox, Velodyne, Ouster)
  - Adaptive voxel sizing for mapping
  - Shift-based scan undistortion
  - Configurable blind-zone filtering for solid-state lidars
"""

from __future__ import annotations

from pathlib import Path

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class AriseSlamConfig(NativeModuleConfig):
    """Config for arise_slam native module.

    Defaults mirror the upstream livox_mid360.yaml config verbatim.
    Scene-specific tuning should be passed via the ``arise_slam``
    kwarg dict on ``smart_nav(...)``.
    """

    cwd: str | None = str(Path(__file__).resolve().parent / "cpp")
    executable: str = "result/bin/arise_slam"
    build_command: str | None = "nix build .#default --no-write-lock-file"

    # ── Feature extraction ─────────────────────────────────────────
    scan_line: int = 4
    """Number of scan lines (4=Livox, 16/32/64=Velodyne)."""
    sensor: str = "livox"
    """Sensor type: "livox", "velodyne", or "ouster"."""
    min_range: float = 0.1
    """Minimum valid point range (metres)."""
    max_range: float = 130.0
    """Maximum valid point range (metres)."""
    provide_point_time: int = 1
    """Whether point cloud includes per-point timestamps (1=yes, 0=no)."""
    mapping_skip_frame: int = 1
    """Process every Nth frame for mapping."""
    use_imu_roll_pitch: bool = False
    """Use IMU roll/pitch for initial orientation estimate."""

    # ── Laser mapping ──────────────────────────────────────────────
    mapping_line_resolution: float = 0.1
    """Voxel resolution for line features (metres)."""
    mapping_plane_resolution: float = 0.2
    """Voxel resolution for plane features (metres)."""
    max_iterations: int = 5
    """Maximum optimization iterations per scan."""
    max_surface_features: int = 2000
    """Maximum number of surface features to use."""
    velocity_failure_threshold: float = 30.0
    """Velocity threshold to detect tracking failure (m/s)."""
    auto_voxel_size: bool = True
    """Enable adaptive voxel sizing."""
    pos_degeneracy_threshold: float = 1.0
    """Position degeneracy detection threshold."""
    ori_degeneracy_threshold: float = 1.0
    """Orientation degeneracy detection threshold."""
    shift_undistortion: bool = True
    """Enable shift-based scan undistortion."""

    # ── IMU preintegration ─────────────────────────────────────────
    acc_n: float = 0.3994
    """Accelerometer noise density."""
    gyr_n: float = 0.001564
    """Gyroscope noise density."""
    acc_w: float = 0.006436
    """Accelerometer random walk."""
    gyr_w: float = 0.0000356
    """Gyroscope random walk."""
    g_norm: float = 9.80511
    """Local gravity magnitude (m/s^2)."""
    lidar_correction_noise: float = 0.01
    """Noise added to lidar pose corrections."""
    smooth_factor: float = 0.9
    """IMU preintegration smoothing factor."""

    # ── IMU bias offsets ───────────────────────────────────────────
    imu_acc_x_offset: float = 0.04
    """Accelerometer X-axis bias offset."""
    imu_acc_y_offset: float = 0.04
    """Accelerometer Y-axis bias offset."""
    imu_acc_z_offset: float = 0.04
    """Accelerometer Z-axis bias offset."""
    imu_acc_x_limit: float = 0.3
    """Accelerometer X-axis bias limit."""
    imu_acc_y_limit: float = 0.3
    """Accelerometer Y-axis bias limit."""
    imu_acc_z_limit: float = 1.0
    """Accelerometer Z-axis bias limit."""

    # ── Blind zone (for Livox) ─────────────────────────────────────
    blind_front: float = 0.2
    """Front blind zone distance (metres)."""
    blind_back: float = 0.2
    """Back blind zone distance (metres)."""
    blind_left: float = 0.3
    """Left blind zone distance (metres)."""
    blind_right: float = 0.3
    """Right blind zone distance (metres)."""
    blind_disk_radius: float = 0.5
    """Central disk blind zone radius (metres)."""


class AriseSlam(NativeModule):
    """Tightly-coupled LiDAR-inertial SLAM.

    Consumes raw lidar scans and IMU measurements from any upstream sensor
    module and produces corrected odometry and world-frame registered
    point clouds.

    Ports:
        lidar (In[PointCloud2]): Raw lidar scan.
        imu (In[Imu]): IMU measurements.
        odometry (Out[Odometry]): Corrected pose.
        registered_scan (Out[PointCloud2]): World-frame registered cloud.
    """

    config: AriseSlamConfig

    lidar: In[PointCloud2]
    imu: In[Imu]
    odometry: Out[Odometry]
    registered_scan: Out[PointCloud2]
