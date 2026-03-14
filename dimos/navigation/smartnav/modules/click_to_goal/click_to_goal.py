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

"""ClickToGoal: forwards Rerun clicked_point to LocalPlanner's way_point.

When the user clicks a point in the Rerun 3D view, the viewer publishes
it on LCM as ``/clicked_point#geometry_msgs.PointStamped``. This module
subscribes to that LCM channel and re-publishes to the ``way_point`` port
so autoconnect wires it to LocalPlanner.

Also publishes a ``goal_path`` (straight line from robot to goal) so the
user can see the full intended route in Rerun.
"""

from __future__ import annotations

import math
import threading

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.protocol.pubsub.impl.lcmpubsub import LCM, Topic


class ClickToGoalConfig(ModuleConfig):
    """Config for the click-to-goal relay."""

    lcm_topic: str = "/clicked_point#geometry_msgs.PointStamped"


class ClickToGoal(Module[ClickToGoalConfig]):
    """Relay Rerun clicked_point → way_point for click-to-navigate.

    Also publishes goal_path (robot→goal straight line) for visualization.

    Ports:
        odometry (In[Odometry]): Vehicle pose for goal line rendering.
        way_point (Out[PointStamped]): Navigation goal for LocalPlanner.
        goal_path (Out[Path]): Straight line from robot to goal for Rerun.
    """

    default_config = ClickToGoalConfig

    odometry: In[Odometry]
    way_point: Out[PointStamped]
    goal: Out[PointStamped]
    goal_path: Out[Path]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._lcm: LCM | None = None
        self._unsub = None
        self._lock = threading.Lock()
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_z = 0.0

    def __getstate__(self) -> dict:
        state = super().__getstate__()
        state.pop("_lcm", None)
        state.pop("_unsub", None)
        state.pop("_lock", None)
        return state

    def __setstate__(self, state: dict) -> None:
        super().__setstate__(state)
        self._lcm = None
        self._unsub = None
        self._lock = threading.Lock()

    def start(self) -> None:
        self.odometry._transport.subscribe(self._on_odom)
        self._lcm = LCM()
        self._lcm.start()
        topic = Topic.from_channel_str(self.config.lcm_topic)
        self._unsub = self._lcm.subscribe(topic, self._on_click)

    def stop(self) -> None:
        if self._unsub:
            self._unsub()
            self._unsub = None
        if self._lcm:
            self._lcm.stop()
            self._lcm = None
        super().stop()

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._robot_x = msg.pose.position.x
            self._robot_y = msg.pose.position.y
            self._robot_z = msg.pose.position.z

    def _on_click(self, msg: PointStamped, _topic: object = None) -> None:
        # Reject invalid clicks (sky/background gives inf or huge coords)
        if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
            print(f"[click_to_goal] Ignored invalid click: ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})")
            return
        if abs(msg.x) > 500 or abs(msg.y) > 500 or abs(msg.z) > 50:
            print(
                f"[click_to_goal] Ignored out-of-range click: ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})"
            )
            return

        with self._lock:
            rx, ry, rz = self._robot_x, self._robot_y, self._robot_z

        print(f"[click_to_goal] Goal: ({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})")
        self.way_point._transport.publish(msg)
        self.goal._transport.publish(msg)

        # Publish a straight-line path from robot to goal for visualization
        import time

        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        now = time.time()
        poses = [
            PoseStamped(
                ts=now, frame_id="map", position=[rx, ry, rz + 0.3], orientation=[0, 0, 0, 1]
            ),
            PoseStamped(
                ts=now,
                frame_id="map",
                position=[msg.x, msg.y, msg.z + 0.3],
                orientation=[0, 0, 0, 1],
            ),
        ]
        goal_line = Path(ts=now, frame_id="map", poses=poses)
        self.goal_path._transport.publish(goal_line)
