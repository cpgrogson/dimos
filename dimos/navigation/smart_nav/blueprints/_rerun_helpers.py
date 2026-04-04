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

"""Shared Rerun visual overrides for SmartNav blueprints."""

from __future__ import annotations

from typing import Any


def sensor_scan_override(cloud: Any) -> Any:
    """Render sensor_scan attached to the sensor TF frame so it moves with the robot."""
    import rerun as rr

    arch = cloud.to_rerun(colormap="turbo", size=0.02)
    return [
        ("world/sensor_scan", rr.Transform3D(parent_frame="tf#/sensor")),
        ("world/sensor_scan", arch),
    ]


def global_map_override(cloud: Any) -> Any:
    """Render accumulated global map — small grey/blue points for map context."""
    return cloud.to_rerun(colormap="cool", size=0.03)


def terrain_map_override(cloud: Any) -> Any:
    """Render terrain_map: big green dots = traversable, red = obstacle.

    The terrain_analysis C++ module sets point intensity to the height
    difference above the planar voxel ground. Low intensity → ground,
    high intensity → obstacle.
    """
    import numpy as np
    import rerun as rr

    points, _ = cloud.as_numpy()
    if len(points) == 0:
        return None

    # Color by z-height: low = green (ground), high = red (obstacle)
    z = points[:, 2]
    z_min, z_max = z.min(), z.max()
    z_norm = (z - z_min) / (z_max - z_min + 1e-8)

    colors = np.zeros((len(points), 3), dtype=np.uint8)
    colors[:, 0] = (z_norm * 255).astype(np.uint8)  # R
    colors[:, 1] = ((1 - z_norm) * 200 + 55).astype(np.uint8)  # G
    colors[:, 2] = 30

    return rr.Points3D(positions=points[:, :3], colors=colors, radii=0.08)


def terrain_map_ext_override(cloud: Any) -> Any:
    """Render extended terrain map — persistent accumulated cloud."""
    return cloud.to_rerun(colormap="viridis", size=0.06)


def path_override(path_msg: Any) -> Any:
    """Render path in vehicle frame by attaching to the sensor TF."""
    import rerun as rr

    if not path_msg.poses:
        return None

    points = [[p.x, p.y, p.z + 0.3] for p in path_msg.poses]
    return [
        ("world/nav_path", rr.Transform3D(parent_frame="tf#/sensor")),
        ("world/nav_path", rr.LineStrips3D([points], colors=[(0, 255, 128)], radii=0.05)),
    ]


def goal_path_override(path_msg: Any) -> Any:
    """Render FAR planner's planned path: orange line + yellow node markers."""
    import rerun as rr

    if not path_msg.poses or len(path_msg.poses) < 2:
        return None

    points = [[p.x, p.y, p.z] for p in path_msg.poses]
    return [
        # Edges: orange line connecting all waypoints
        ("world/goal_path/edges", rr.LineStrips3D([points], colors=[(255, 140, 0)], radii=0.04)),
        # Nodes: yellow spheres at each graph node in the path
        ("world/goal_path/nodes", rr.Points3D(points, colors=[(255, 255, 0)], radii=0.12)),
    ]


def waypoint_override(msg: Any) -> Any:
    """Render the current waypoint goal as a visible marker."""
    import math

    import rerun as rr

    if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
        return None

    return rr.Points3D(
        positions=[[msg.x, msg.y, msg.z + 0.5]],
        colors=[(255, 50, 50)],
        radii=0.3,
    )


def static_robot(rr: Any) -> list[Any]:
    """Static robot rectangle attached to the sensor TF frame.

    Renders a wireframe box roughly the size of the mecanum-wheel platform,
    so you can see the robot's position and heading in the 3D view.
    """
    return [
        rr.Boxes3D(
            half_sizes=[0.25, 0.20, 0.6],  # ~50x40x120 cm box (G1 humanoid)
            centers=[[0, 0, -0.6]],
            colors=[(0, 255, 127)],
            fill_mode="MajorWireframe",
        ),
        rr.Transform3D(parent_frame="tf#/sensor"),
    ]


def static_floor(rr: Any) -> list[Any]:
    """Static ground plane at z=0 as a solid textured quad."""

    s = 50.0  # half-size
    return [
        rr.Mesh3D(
            vertex_positions=[[-s, -s, 0], [s, -s, 0], [s, s, 0], [-s, s, 0]],
            triangle_indices=[[0, 1, 2], [0, 2, 3]],
            vertex_colors=[[40, 40, 40, 120]] * 4,
        )
    ]
