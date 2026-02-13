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

"""Spec compliance tests for LidarModule."""

from __future__ import annotations

import typing

from dimos.hardware.sensors.lidar.module import LidarModule
from dimos.spec import perception
from dimos.spec.utils import assert_implements_protocol


def test_lidar_module_implements_lidar_spec() -> None:
    assert_implements_protocol(LidarModule, perception.Lidar)


def test_lidar_spec_does_not_require_imu() -> None:
    """Not all LiDARs have IMU — Lidar spec should only require pointcloud."""
    hints = typing.get_type_hints(perception.Lidar, include_extras=True)
    assert "pointcloud" in hints
    assert "imu" not in hints
