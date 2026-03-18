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

from typing import Any

from dimos.core.blueprints import autoconnect
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.perception.depth.depth_anything3.module import DA3Mode, DA3Model, DepthAnything3Module
from dimos.perception.slam.orbslam3.module import OrbSlam3
from dimos.protocol.pubsub.impl.lcmpubsub import LCM
from dimos.visualization.rerun.bridge import rerun_bridge

_CAM = "world/tf/camera_optical"


def _convert_camera_info(camera_info: Any) -> Any:
    """Log Pinhole under the TF frame so path hierarchy matches transform hierarchy."""
    import rerun as rr

    fx, fy = camera_info.K[0], camera_info.K[4]
    cx, cy = camera_info.K[2], camera_info.K[5]
    pinhole = rr.Pinhole(
        focal_length=[fx, fy],
        principal_point=[cx, cy],
        width=camera_info.width,
        height=camera_info.height,
        image_plane_distance=1.0,
    )
    return [
        (_CAM + "/image", pinhole),
        (_CAM + "/depth", pinhole),
    ]


def _relog_image(image: Any) -> Any:
    """Relog color_image under the camera TF frame instead of world/."""
    return [(_CAM + "/image", image.to_rerun())]


def _convert_odometry(odom: Any) -> Any:
    """Place odometry in the TF tree as map → camera_optical."""
    import rerun as rr

    return [
        (
            _CAM,
            rr.Transform3D(
                translation=[odom.x, odom.y, odom.z],
                rotation=rr.Quaternion(
                    xyzw=[
                        odom.orientation.x,
                        odom.orientation.y,
                        odom.orientation.z,
                        odom.orientation.w,
                    ]
                ),
                parent_frame="tf#/map",
            ),
        ),
    ]


def _orbslam3_rerun_blueprint() -> Any:
    """Split layout: camera feed + 3D world view side by side."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin=_CAM + "/image", name="Camera"),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.archetypes.Background(color=[0, 0, 0]),
            ),
            column_shares=[1, 2],
        ),
    )


orbslam3_webcam = autoconnect(
    CameraModule.blueprint(transform=None),
    OrbSlam3.blueprint(sensor_mode="MONOCULAR"),
    rerun_bridge(
        blueprint=_orbslam3_rerun_blueprint,
        pubsubs=[LCM()],
        visual_override={
            "world/camera_info": _convert_camera_info,
            "world/color_image": _relog_image,
            "world/odometry": _convert_odometry,
        },
    ),
)


def _relog_depth(depth_image: Any) -> Any:
    """Relog depth_image under the camera TF frame as a sibling of image."""
    return [(_CAM + "/depth", depth_image.to_rerun())]


def _orbslam3_depth_rerun_blueprint() -> Any:
    """Three-panel layout: camera + depth + 3D world view."""
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin=_CAM + "/image", name="Camera"),
                rrb.Spatial2DView(origin=_CAM + "/depth", name="Depth"),
            ),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.archetypes.Background(color=[0, 0, 0]),
            ),
            column_shares=[1, 2],
        ),
    )


orbslam3_depth_webcam = autoconnect(
    CameraModule.blueprint(transform=None),
    OrbSlam3.blueprint(sensor_mode="MONOCULAR"),
    DepthAnything3Module.blueprint(
        model=DA3Model.SMALL,
        mode=DA3Mode.TEMPORAL,
        window_frames=10,
    ),
    rerun_bridge(
        blueprint=_orbslam3_depth_rerun_blueprint,
        pubsubs=[LCM()],
        visual_override={
            "world/camera_info": _convert_camera_info,
            "world/color_image": _relog_image,
            "world/odometry": _convert_odometry,
            "world/depth_image": _relog_depth,
        },
    ),
).global_config(n_workers=4)

__all__ = ["orbslam3_depth_webcam", "orbslam3_webcam"]
