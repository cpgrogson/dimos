#!/usr/bin/env python3
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

"""Go2 IncrementalNav: IncrementalMap + VoxelMapper + CostMapper + Planner.

Like ``unitree_go2_smartnav`` but replaces PGO + ScanCorrector with the
lighter-weight IncrementalMap module (no GTSAM dependency).

Builds on unitree_go2 which already includes:
  GO2Connection, VoxelGridMapper, CostMapper, ReplanningAStarPlanner,
  WavefrontFrontierExplorer, PatrollingModule

We add IncrementalMap and remap GO2Connection streams to feed it.

Data flow:
    GO2Connection.lidar  (remapped → registered_scan) → IncrementalMap
    GO2Connection.odom   (remapped → raw_odom)         → IncrementalMap.raw_odom
    IncrementalMap.odom  (PoseStamped)                  → Planner, Explorer
    IncrementalMap.global_map                           → VoxelGridMapper (via lidar remap)

Usage:
    dimos run unitree-go2-incremental-nav --robot-ip 192.168.123.161
"""

from dimos.core.blueprints import autoconnect
from dimos.mapping.voxels import VoxelGridMapper
from dimos.navigation.incremental_map.module import IncrementalMap
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2
from dimos.robot.unitree.go2.connection import GO2Connection

unitree_go2_incremental_nav = (
    autoconnect(
        unitree_go2,
        IncrementalMap.blueprint(),
    )
    .global_config(n_workers=8)
    .remappings(
        [
            # Feed GO2's lidar into IncrementalMap as registered_scan
            (GO2Connection, "lidar", "registered_scan"),
            # Feed GO2's odom into IncrementalMap as raw_odom
            (GO2Connection, "odom", "raw_odom"),
            # Feed IncrementalMap's global_map into VoxelGridMapper
            (VoxelGridMapper, "lidar", "global_map"),
        ]
    )
)

__all__ = ["unitree_go2_incremental_nav"]
