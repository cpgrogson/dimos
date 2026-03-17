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

"""Python NativeModule wrapper for ORB-SLAM3 visual SLAM.

Wraps ORB-SLAM3 as a native subprocess that receives camera images and
outputs camera pose estimates (odometry).

Usage::

    from dimos.perception.slam.orbslam3.module import OrbSlam3
    from dimos.core.blueprints import autoconnect

    autoconnect(
        OrbSlam3.blueprint(sensor_mode="MONOCULAR"),
        SomeConsumer.blueprint(),
    ).build().loop()
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Annotated

from pydantic.experimental.pipeline import validate_as

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image
from dimos.spec import perception

_CONFIG_DIR = Path(__file__).parent / "config"


class OrbSlam3Config(NativeModuleConfig):
    """Config for the ORB-SLAM3 visual SLAM native module."""

    cwd: str | None = "cpp"
    executable: str = "result/bin/orbslam3_native"
    build_command: str | None = "nix build .#orbslam3_native"

    # ORB-SLAM3 sensor mode
    sensor_mode: str = "MONOCULAR"  # MONOCULAR, STEREO, RGBD, IMU_MONOCULAR, IMU_STEREO, IMU_RGBD

    # Pangolin viewer (disable for headless)
    use_viewer: bool = False

    # Frame IDs for output messages
    frame_id: str = "map"
    child_frame_id: str = "camera_optical"

    # Camera settings YAML (relative to config/ dir, or absolute path)
    settings: Annotated[
        Path, validate_as(...).transform(lambda p: p if p.is_absolute() else _CONFIG_DIR / p)
    ] = Path("RealSense_D435i.yaml.opencv")

    # Resolved from settings, passed as --settings_path to the binary
    settings_path: str | None = None

    # Vocabulary path (None = use compiled-in default from nix build)
    vocab_path: str | None = None

    # settings is not a CLI arg (settings_path is)
    cli_exclude: frozenset[str] = frozenset({"settings"})

    def model_post_init(self, __context: object) -> None:
        super().model_post_init(__context)
        # Resolve relative settings path against config/ dir
        if not self.settings.is_absolute():
            self.settings = _CONFIG_DIR / self.settings
        if self.settings_path is None:
            self.settings_path = str(self.settings)


class OrbSlam3(NativeModule[OrbSlam3Config], perception.Odometry):
    """ORB-SLAM3 visual SLAM module.

    Ports:
        color_image (In[Image]): Camera frames to track.
        odometry (Out[Odometry]): Camera pose as nav_msgs.Odometry.
    """

    default_config = OrbSlam3Config
    color_image: In[Image]
    odometry: Out[Odometry]


orbslam3_module = OrbSlam3.blueprint

__all__ = [
    "OrbSlam3",
    "OrbSlam3Config",
    "orbslam3_module",
]

# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    OrbSlam3()
