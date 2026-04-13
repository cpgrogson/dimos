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

"""CmdVelMux: merges nav and teleop velocity commands.

Teleop (tele_cmd_vel) takes priority over autonomous navigation
(nav_cmd_vel). When teleop is active, nav commands are suppressed
and a stop_movement signal is published. After a cooldown period
with no teleop input, nav commands resume.
"""

from __future__ import annotations

import threading
from typing import Any
import weakref

from dimos_lcm.std_msgs import Bool

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class CmdVelMuxConfig(ModuleConfig):
    tele_cooldown_sec: float = 1.0
    tele_linear_scale: float = 1.0


class CmdVelMux(Module):
    """Multiplexes nav_cmd_vel and tele_cmd_vel into a single cmd_vel output.

    When teleop input arrives, stop_movement is published so downstream
    modules (planner, explorer) can cancel their active goals.
    
    config.tele_cooldown_sec
        nav_cmd_vel will be ignored for tele_cooldown_sec seconds after
        the last teleop command
        
        dev notes: each new tele_cmd_vel message restarts the cooldown
        so under continuous teleop (e.g. 50 Hz joystick) the cooldown
        is never actually reached; it only fires once the operator stops.

    Ports:
        nav_cmd_vel (In[Twist]): Velocity from the autonomous planner.
        tele_cmd_vel (In[Twist]): Velocity from keyboard/joystick teleop.
        cmd_vel (Out[Twist]): Merged output — teleop wins when active.
        stop_movement (Out[Bool]): Published once per cooldown window, on
            the first teleop message; downstream nav modules should cancel
            their active goal when they see it.
    """

    config: CmdVelMuxConfig

    nav_cmd_vel: In[Twist]
    tele_cmd_vel: In[Twist]
    cmd_vel: Out[Twist]
    stop_movement: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._teleop_active = False
        self._lock = threading.Lock()
        self._timer: threading.Timer | None = None
        # Monotonic token identifying the current cooldown timer. Each new
        # _on_teleop bumps this; _end_teleop short-circuits if its captured
        # generation doesn't match — a cheap fix for stale Timer callbacks.
        self._timer_gen = 0

    def __getstate__(self) -> dict[str, Any]:
        state: dict[str, Any] = super().__getstate__()  # type: ignore[no-untyped-call]
        state.pop("_lock", None)
        state.pop("_timer", None)
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        super().__setstate__(state)
        self._lock = threading.Lock()
        self._timer = None
        self._timer_gen = 0

    def __del__(self) -> None:
        # Cancel any pending cooldown timer so the daemon thread doesn't
        # outlive the mux and trip pytest's thread-leak detector.
        timer = getattr(self, "_timer", None)
        if timer is not None:
            timer.cancel()
            timer.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(self.nav_cmd_vel.subscribe(self._on_nav))
        self.register_disposable(self.tele_cmd_vel.subscribe(self._on_teleop))

    @rpc
    def stop(self) -> None:
        with self._lock:
            self._timer_gen += 1  # invalidate any pending _end_teleop
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
        super().stop()

    def _on_nav(self, msg: Twist) -> None:
        with self._lock:
            if self._teleop_active:
                return
        self.cmd_vel.publish(msg)

    def _on_teleop(self, msg: Twist) -> None:
        was_active: bool
        with self._lock:
            was_active = self._teleop_active
            self._teleop_active = True
            if self._timer is not None:
                # Cancel + join so the superseded Timer thread exits promptly
                # rather than accumulating under rapid teleop (50 Hz) and
                # tripping pytest's thread-leak detector.
                self._timer.cancel()
                self._timer.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._timer_gen += 1
            my_gen = self._timer_gen
            # weakref prevents the Timer thread from keeping the mux alive
            # via a bound-method reference — otherwise mux.__del__ can't
            # run at test scope exit.
            self_ref = weakref.ref(self)

            def _end() -> None:
                obj = self_ref()
                if obj is not None:
                    obj._end_teleop(my_gen)

            self._timer = threading.Timer(self.config.tele_cooldown_sec, _end)
            self._timer.daemon = True
            self._timer.start()

        if not was_active:
            self.stop_movement.publish(Bool(data=True))
            logger.info("Teleop active — published stop_movement")

        s = self.config.tele_linear_scale
        if s != 1.0:
            msg = Twist(
                linear=[msg.linear.x * s, msg.linear.y * s, msg.linear.z],
                angular=[msg.angular.x, msg.angular.y, msg.angular.z],
            )
        self.cmd_vel.publish(msg)

    def _end_teleop(self, expected_gen: int) -> None:
        with self._lock:
            if expected_gen != self._timer_gen:
                # Superseded by a newer timer (or cleared by stop()).
                return
            self._teleop_active = False
            self._timer = None
