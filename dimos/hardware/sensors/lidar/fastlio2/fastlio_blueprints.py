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

from dimos.core.blueprints import autoconnect
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.mapping.voxels import VoxelGridMapper
from dimos.visualization.rerun.bridge import rerun_bridge

mid360_fastlio = autoconnect(
    FastLio2.blueprint(),
    rerun_bridge(),
).global_config(n_dask_workers=2, robot_model="mid360_fastlio2")


rerun_config = {}

mid360_fastlio_voxels = autoconnect(
    FastLio2Module.blueprint(),
    VoxelGridMapper.blueprint(publish_interval=0.5, voxel_size=0.1, carve_columns=False),
    rerun_bridge(
        visual_override={
            "world/global_map": lambda grid: grid.to_rerun(voxel_size=0.1, mode="boxes"),
            "world/lidar": None,
        }
    ),
).global_config(n_dask_workers=3, robot_model="mid360_fastlio2_voxels")
