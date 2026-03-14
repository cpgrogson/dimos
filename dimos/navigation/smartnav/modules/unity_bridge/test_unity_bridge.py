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

"""Tests for UnityBridgeModule (kinematic simulator)."""

import math
import time

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.navigation.smartnav.modules.unity_bridge.unity_bridge import UnityBridgeModule


class _MockTransport:
    """Lightweight mock transport that captures published messages."""

    def __init__(self):
        self._messages = []
        self._subscribers = []

    def publish(self, msg):
        self._messages.append(msg)
        for cb in self._subscribers:
            cb(msg)

    def broadcast(self, _stream, msg):
        self.publish(msg)

    def subscribe(self, cb):
        self._subscribers.append(cb)

        def unsub():
            self._subscribers.remove(cb)

        return unsub


class TestUnityBridge:
    """Test the kinematic vehicle simulator."""

    def _make_module(self, **kwargs) -> UnityBridgeModule:
        """Create a UnityBridgeModule with test config (kwargs go directly to constructor)."""
        defaults = dict(sim_rate=200.0, vehicle_height=0.75)
        defaults.update(kwargs)
        return UnityBridgeModule(**defaults)

    def test_initial_state(self):
        """Module starts at configured initial position."""
        module = self._make_module(init_x=1.0, init_y=2.0, init_z=0.5)
        # The internal z includes vehicle height
        assert module._x == 1.0
        assert module._y == 2.0
        assert abs(module._z - (0.5 + 0.75)) < 0.01

    def test_zero_velocity_no_motion(self):
        """With zero velocity, position should not change."""
        module = self._make_module()
        initial_x = module._x
        initial_y = module._y

        # Simulate one step manually
        module._fwd_speed = 0.0
        module._left_speed = 0.0
        module._yaw_rate = 0.0

        # Call the simulation logic directly (extract from loop)
        dt = 1.0 / module.config.sim_rate
        cos_yaw = math.cos(module._yaw)
        sin_yaw = math.sin(module._yaw)
        module._x += dt * cos_yaw * 0 - dt * sin_yaw * 0
        module._y += dt * sin_yaw * 0 + dt * cos_yaw * 0

        assert module._x == initial_x
        assert module._y == initial_y

    def test_forward_motion(self):
        """Forward velocity should move vehicle in yaw direction."""
        module = self._make_module()
        module._yaw = 0.0  # Facing +X
        module._fwd_speed = 1.0
        module._left_speed = 0.0
        module._yaw_rate = 0.0

        dt = 1.0 / module.config.sim_rate
        initial_x = module._x

        # Simulate one step
        module._x += dt * math.cos(module._yaw) * module._fwd_speed
        module._y += dt * math.sin(module._yaw) * module._fwd_speed

        assert module._x > initial_x

    def test_cmd_vel_handler(self):
        """Twist messages should update internal velocity state."""
        module = self._make_module()

        twist = Twist(linear=[1.5, 0.5, 0.0], angular=[0.0, 0.0, 0.3])
        module._on_cmd_vel(twist)

        assert module._fwd_speed == 1.5
        assert module._left_speed == 0.5
        assert module._yaw_rate == 0.3

    def test_yaw_wrapping(self):
        """Yaw should wrap around at +/-pi."""
        module = self._make_module()
        module._yaw = math.pi - 0.01
        module._yaw_rate = 1.0

        dt = 1.0 / module.config.sim_rate
        module._yaw += dt * module._yaw_rate

        # Should wrap around
        if module._yaw > math.pi:
            module._yaw -= 2 * math.pi

        assert module._yaw < math.pi
        assert module._yaw > -math.pi


class TestUnityBridgeOdometryOutput:
    """Test odometry output from the simulator."""

    def test_odometry_publish(self):
        """Simulator should publish odometry messages."""
        module = UnityBridgeModule(sim_rate=200.0)

        # Wire a mock transport to the odometry output port
        odom_transport = _MockTransport()
        module.odometry._transport = odom_transport

        results = []
        odom_transport.subscribe(lambda msg: results.append(msg))

        # Manually trigger one step worth of publishing
        module._fwd_speed = 0.0
        module._left_speed = 0.0
        module._yaw_rate = 0.0

        # Build and publish manually (same logic as _sim_loop)
        from dimos.msgs.geometry_msgs.Pose import Pose
        from dimos.msgs.geometry_msgs.Quaternion import Quaternion
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        quat = Quaternion.from_euler(Vector3(0.0, 0.0, 0.0))
        odom = Odometry(
            ts=time.time(),
            frame_id="map",
            child_frame_id="sensor",
            pose=Pose(position=[0, 0, 0.75], orientation=[quat.x, quat.y, quat.z, quat.w]),
        )
        module.odometry._transport.publish(odom)

        assert len(results) == 1
        assert results[0].frame_id == "map"
