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

"""G1 nav onboard — FAR planner + PGO loop closure + local obstacle avoidance.

Full navigation stack on real hardware with:
- FAR visibility-graph global route planner
- PGO pose graph optimization with loop closure detection (GTSAM iSAM2)
- Local planner for reactive obstacle avoidance
- Path follower for velocity control
- FastLio2 SLAM from Livox Mid-360 lidar
- G1HighLevelDdsSdk for robot velocity commands

Pose routing via TF tree (per CMU ICRA 2022 Fig. 11):
- PGO publishes map→odom correction to TF.  FastLio2 publishes odom→body.
- Global planners (MovementManager) query self.tf.get("map", "body").
- NativeModules (FarPlanner, TerrainAnalysis, LocalPlanner, PathFollower) receive
  odometry via a TF-to-Odometry bridge in their Python wrappers.

Data flow:
    Click → MovementManager (TF: map→body) → goal → FarPlanner (TF bridge)
    → way_point → LocalPlanner (raw odom) → path → PathFollower (raw odom)
    → nav_cmd_vel → MovementManager → cmd_vel → G1HighLevelDdsSdk

    registered_scan + odometry → PGO → map→odom TF + global_map
"""

from __future__ import annotations

import os

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.navigation.smart_nav.main import smart_nav, smart_nav_rerun_config
from dimos.robot.unitree.g1.config import G1
from dimos.robot.unitree.g1.effectors.high_level.dds_sdk import G1HighLevelDdsSdk
from dimos.robot.unitree.g1.g1_rerun import (
    g1_odometry_tf_override,
    g1_static_robot,
)
from dimos.visualization.vis_module import vis_module

unitree_g1_nav_onboard = (
    autoconnect(
        FastLio2.blueprint(
            host_ip=os.getenv("LIDAR_HOST_IP", "192.168.123.164"),
            lidar_ip=os.getenv("LIDAR_IP", "192.168.123.120"),
            mount=G1.internal_odom_offsets["mid360_link"],
            map_freq=1.0,
            config="lio_autonomy.yaml",
        ),
        smart_nav(
            use_simple_planner=True,
            vehicle_height=G1.height_clearance,
            vehicle_width=G1.width_clearance,
            terrain_analysis={
                "obstacle_height_threshold": 0.01,
                "ground_height_threshold": 0.01,
            },
            local_planner={
                "max_speed": 0.5,
                "autonomy_speed": 0.5,
            },
            path_follower={
                "max_speed": 0.5,
                "autonomy_speed": 0.5,
            },
            simple_planner={
                "cell_size": 0.3,
                "obstacle_height_threshold": 0.20,
                "inflation_radius": 0.4,
                "lookahead_distance": 2.0,
                "replan_rate": 5.0,
                "replan_cooldown": 2.0,
            },
            far_planner={
                "sensor_range": 15.0,
                "is_static_env": False,
                "converge_dist": 1.5,
            },
        ),
        G1HighLevelDdsSdk.blueprint(),
        vis_module(
            viewer_backend=global_config.viewer,
            rerun_config=smart_nav_rerun_config(
                {
                    "visual_override": {"world/odometry": g1_odometry_tf_override},
                    "static": {"world/tf/robot": g1_static_robot},
                    "memory_limit": "1GB",
                }
            ),
        ),
    )
    .remappings(
        [
            # FastLio2 outputs "lidar"; SmartNav modules expect "registered_scan"
            (FastLio2, "lidar", "registered_scan"),
            (FastLio2, "global_map", "global_map_fastlio"),
        ]
    )
    .global_config(n_workers=12, robot_model="unitree_g1")
)


__all__ = ["unitree_g1_nav_onboard"]
