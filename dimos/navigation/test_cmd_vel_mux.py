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

"""Tests for CmdVelMux teleop/nav priority switching."""

from __future__ import annotations

import threading
import time
from typing import Any, cast
from unittest.mock import MagicMock, patch

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.cmd_vel_mux import CmdVelMux, CmdVelMuxConfig


def _make_mux(cooldown: float = 0.1, linear_scale: float = 1.0) -> Any:
    """Build a CmdVelMux with mocked output streams. __del__ cleans up the timer."""
    with patch.object(CmdVelMux, "__init__", lambda self: None):
        mux = cast("Any", CmdVelMux.__new__(CmdVelMux))
    mux.config = CmdVelMuxConfig(
        tele_cooldown_sec=cooldown,
        tele_linear_scale=linear_scale,
    )
    mux._teleop_active = False
    mux._lock = threading.Lock()
    mux._timer = None
    mux._timer_gen = 0
    mux.cmd_vel = MagicMock()
    mux.stop_movement = MagicMock()
    return mux


def _twist(lx: float = 0.0, az: float = 0.0) -> Twist:
    return Twist(linear=Vector3(lx, 0, 0), angular=Vector3(0, 0, az))


class TestNavPassthrough:
    def test_nav_passes_through_when_no_teleop(self) -> None:
        mux = _make_mux()
        mux._on_nav(_twist(lx=0.5))
        mux.cmd_vel.publish.assert_called_once()
        mux.stop_movement.publish.assert_not_called()

    def test_nav_suppressed_while_teleop_active(self) -> None:
        mux = _make_mux(cooldown=10.0)
        mux._on_teleop(_twist(lx=0.3))  # activates teleop
        mux.cmd_vel.publish.reset_mock()

        mux._on_nav(_twist(lx=0.9))
        mux.cmd_vel.publish.assert_not_called()

    def test_nav_resumes_after_cooldown(self) -> None:
        mux = _make_mux(cooldown=0.05)
        mux._on_teleop(_twist(lx=0.3))
        time.sleep(0.15)  # let the Timer fire
        mux.cmd_vel.publish.reset_mock()

        mux._on_nav(_twist(lx=0.9))
        mux.cmd_vel.publish.assert_called_once()


class TestTeleop:
    def test_first_teleop_publishes_stop_movement(self) -> None:
        mux = _make_mux()
        mux._on_teleop(_twist(lx=0.3))
        mux.stop_movement.publish.assert_called_once()

    def test_subsequent_teleop_does_not_republish_stop_movement(self) -> None:
        mux = _make_mux(cooldown=10.0)
        mux._on_teleop(_twist(lx=0.3))
        mux._on_teleop(_twist(lx=0.4))
        mux._on_teleop(_twist(lx=0.5))
        assert mux.stop_movement.publish.call_count == 1

    def test_teleop_publishes_to_cmd_vel(self) -> None:
        mux = _make_mux()
        mux._on_teleop(_twist(lx=0.5, az=0.1))
        mux.cmd_vel.publish.assert_called_once()

    def test_tele_linear_scale_applied(self) -> None:
        mux = _make_mux(linear_scale=0.5)
        mux._on_teleop(_twist(lx=1.0))
        published = mux.cmd_vel.publish.call_args[0][0]
        assert published.linear.x == 0.5

    def test_tele_linear_scale_of_one_skips_copy(self) -> None:
        mux = _make_mux(linear_scale=1.0)
        msg = _twist(lx=0.7)
        mux._on_teleop(msg)
        published = mux.cmd_vel.publish.call_args[0][0]
        assert published is msg  # no unnecessary allocation when scale == 1


class TestEndTeleop:
    def test_end_teleop_clears_flag(self) -> None:
        mux = _make_mux(cooldown=10.0)
        mux._on_teleop(_twist(lx=0.3))  # installs timer, bumps _timer_gen to 1
        timer = mux._timer  # keep a ref so we can tear it down after
        mux._end_teleop(mux._timer_gen)
        assert not mux._teleop_active
        assert mux._timer is None
        # The installed timer is still counting down; cancel so it doesn't
        # outlive the test and trip the thread-leak detector.
        timer.cancel()
        timer.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

    def test_end_teleop_noop_when_superseded(self) -> None:
        mux = _make_mux(cooldown=10.0)
        # Two back-to-back teleop calls: the first cooldown's generation is
        # stale by the time the second call bumps _timer_gen. Firing the
        # stale callback must be a no-op against the current state.
        mux._on_teleop(_twist(lx=0.3))
        stale_gen = mux._timer_gen
        mux._on_teleop(_twist(lx=0.4))
        current_timer = mux._timer

        mux._end_teleop(stale_gen)
        assert mux._teleop_active  # still active
        assert mux._timer is current_timer  # current timer untouched


class TestConfigDefaults:
    def test_cooldown_default(self) -> None:
        config = CmdVelMuxConfig()
        assert config.tele_cooldown_sec == 1.0

    def test_linear_scale_default(self) -> None:
        config = CmdVelMuxConfig()
        assert config.tele_linear_scale == 1.0
