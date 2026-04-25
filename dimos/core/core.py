#!/usr/bin/env python3
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

from __future__ import annotations

import asyncio
import functools
import inspect
from typing import (
    TYPE_CHECKING,
    Any,
    TypeVar,
)

if TYPE_CHECKING:
    from collections.abc import Callable, Coroutine

T = TypeVar("T")

from typing import ParamSpec, TypeVar

P = ParamSpec("P")
R = TypeVar("R")


def rpc(fn: Callable[P, R]) -> Callable[P, R]:
    fn.__rpc__ = True  # type: ignore[attr-defined]
    return fn


def arpc(fn: Callable[..., Coroutine[Any, Any, Any]]) -> Callable[..., Any]:
    """Mark an async method as an RPC body that runs on the module's self._loop.

    Dual-mode dispatch:
      * Caller is on self._loop (another @arpc, a handle_*, or a process_observable
        callback): returns the coroutine so the caller can ``await`` it normally.
      * Caller is on any other thread (RPC dispatcher, sync test, sync @rpc on the
        same module): schedules the coroutine onto self._loop and blocks until done.

    Discovery is shared with @rpc — sets ``__rpc__ = True`` so the method appears
    in ``Module.rpcs`` and is served by the existing RPC machinery without changes.
    """
    if not inspect.iscoroutinefunction(fn):
        raise TypeError("@arpc requires an `async def` method")

    @functools.wraps(fn)
    def wrapper(self, *args: Any, **kwargs: Any) -> Any:
        loop = getattr(self, "_loop", None)
        try:
            running = asyncio.get_running_loop()
        except RuntimeError:
            running = None
        if running is loop:
            return fn(self, *args, **kwargs)
        future = asyncio.run_coroutine_threadsafe(fn(self, *args, **kwargs), loop)
        return future.result()

    wrapper.__rpc__ = True  # type: ignore[attr-defined]
    wrapper.__arpc__ = True  # type: ignore[attr-defined]
    wrapper.aio = fn  # type: ignore[attr-defined]
    return wrapper
