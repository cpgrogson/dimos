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

"""Spec compliance tests for the Livox LiDAR module."""

from __future__ import annotations

from dimos.hardware.sensors.lidar.livox.module import Mid360Module
from dimos.spec import perception
from dimos.spec.utils import assert_implements_protocol


def test_livox_module_implements_lidar_spec() -> None:
    assert_implements_protocol(Mid360Module, perception.Lidar)


def test_livox_module_implements_imu_spec() -> None:
    assert_implements_protocol(Mid360Module, perception.IMU)
