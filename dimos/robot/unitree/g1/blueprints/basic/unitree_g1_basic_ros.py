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

"""G1 hardware stack with ROS nav: real robot connection and ROS navigation stack."""

from dimos.core.blueprints import autoconnect
from dimos.navigation.rosnav_docker import ros_nav
from dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_primitive_no_cam import (
    unitree_g1_primitive_no_cam,
)

# G1 EDU hardware defaults.  Override by calling ros_nav(...) directly.
# lidar_interface: ethernet port connected to the robot/lidar (Jetson Orin: "eth0")
# lidar subnet: 192.168.123.x — internal G1 EDU network
# unitree_ip: G1 LocalAP WiFi address (used when robot runs its own access point)
unitree_g1_basic_ros = autoconnect(
    unitree_g1_primitive_no_cam,
    ros_nav(
        mode="hardware",
        robot_config_path="unitree/unitree_g1",
        lidar_interface="eth0",
        lidar_computer_ip="192.168.123.5",
        lidar_gateway="192.168.123.1",
        lidar_ip="192.168.123.120",
        unitree_ip="192.168.12.1",
        unitree_conn="LocalAP",
    ),
)

__all__ = ["unitree_g1_basic_ros"]
