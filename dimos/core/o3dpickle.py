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

import copyreg

# open3d is imported lazily (inside functions) rather than at module level.
# dimos.core.core imports this module just to register pickle handlers, and core is
# imported by almost everything — including lightweight docker modules that don't use
# open3d. A module-level import would drag in open3d's sklearn/scipy chain everywhere,
# which crashes in environments where those packages aren't installed or version-matched.
# (i.e. minimal docker envs)

def reduce_external(obj):  # type: ignore[no-untyped-def]
    import numpy as np

    # Convert Vector3dVector to numpy array for pickling
    points_array = np.asarray(obj.points)
    return (reconstruct_pointcloud, (points_array,))


def reconstruct_pointcloud(points_array):  # type: ignore[no-untyped-def]
    import open3d as o3d  # type: ignore[import-untyped]

    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points_array)
    return pc


def register_picklers() -> None:
    try:
        import open3d as o3d  # type: ignore[import-untyped]
    except ImportError:
        return  # open3d not installed in this environment; skip registration

    _dummy_pc = o3d.geometry.PointCloud()
    copyreg.pickle(_dummy_pc.__class__, reduce_external)
