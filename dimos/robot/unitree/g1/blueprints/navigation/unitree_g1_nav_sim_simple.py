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

"""G1 nav sim variant — SimplePlanner (grid A*) in place of FarPlanner.

Identical to ``unitree_g1_nav_sim`` except the visibility-graph FarPlanner
is swapped for SimplePlanner, a small pure-Python 2D-costmap + A* planner.
Useful for comparing the two planners apples-to-apples in the same
environment (and as a fallback when the C++ FarPlanner binary isn't
available).

See ``dimos/navigation/smart_nav/modules/simple_planner/`` for the
planner implementation.
"""

from __future__ import annotations

from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.navigation.smart_nav.main import smart_nav, smart_nav_rerun_config
from dimos.navigation.smart_nav.modules.sensor_scan_generation.sensor_scan_generation import (
    SensorScanGeneration,
)
from dimos.robot.unitree.g1.blueprints.navigation.g1_rerun import g1_static_robot
from dimos.simulation.unity.module import UnityBridgeModule
from dimos.visualization.vis_module import vis_module

unitree_g1_nav_sim_simple = (
    autoconnect(
        UnityBridgeModule.blueprint(
            unity_binary="",
            unity_scene="home_building_1",
            vehicle_height=1.24,
        ),
        SensorScanGeneration.blueprint(),
        smart_nav(
            use_simple_planner=True,
            use_smooth_local_planner=True,
            terrain_analysis={
                "obstacle_height_threshold": 0.1,
                "ground_height_threshold": 0.05,
                "max_relative_z": 0.3,
                "min_relative_z": -1.5,
            },
            smooth_local_planner={
                "max_curvature": 1.2,
                "arc_length": 3.0,
                "obstacle_range": 4.0,
                "robot_radius": 0.50,
                "obstacle_height_threshold": 0.1,
                "max_relative_z": 0.3,
                "min_relative_z": -1.5,
                "curvature_ema_alpha": 0.3,
                "clearance_weight": 2.0,
                "alignment_weight": 1.0,
                "hysteresis_weight": 0.4,
                "publish_rate": 20.0,
                "publish_length": 3.0,
                "goal_stop_radius": 0.5,
            },
            path_follower={
                "max_speed": 1.0,
                "autonomy_speed": 1.0,
                "max_acceleration": 2.0,
                "slow_down_distance_threshold": 0.5,
                "omni_dir_goal_threshold": 0.5,
                "two_way_drive": False,
            },
            simple_planner={
                "cell_size": 0.3,
                "obstacle_height_threshold": 0.15,
                "inflation_radius": 0.1,
                "lookahead_distance": 2.0,
                "replan_rate": 5.0,
            },
        ),
        vis_module(
            viewer_backend=global_config.viewer,
            rerun_config=smart_nav_rerun_config(
                {
                    "blueprint": UnityBridgeModule.rerun_blueprint,
                    "visual_override": {
                        "world/camera_info": UnityBridgeModule.rerun_suppress_camera_info,
                    },
                    "static": {
                        "world/color_image": UnityBridgeModule.rerun_static_pinhole,
                        "world/tf/robot": g1_static_robot,
                    },
                }
            ),
        ),
    )
    .remappings(
        [
            # Unity needs the extended (persistent) terrain map for Z-height, not the local one
            (UnityBridgeModule, "terrain_map", "terrain_map_ext"),
        ]
    )
    .global_config(n_workers=8, robot_model="unitree_g1", simulation=True)
)


def main() -> None:
    unitree_g1_nav_sim_simple.build().loop()


__all__ = ["unitree_g1_nav_sim_simple"]

if __name__ == "__main__":
    main()
