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

import asyncio
from concurrent.futures import Future
from typing import Protocol

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import arpc, rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.spec.utils import Spec


class Dubler(Module):
    a: In[int]
    double_a: Out[int]

    async def handle_a(self, x: int) -> None:
        self.double_a.publish(x * 2)

    @arpc
    async def find_duble(self, x: int) -> int:
        await asyncio.sleep(0.5)
        return x * 2


class DublerSpec(Spec, Protocol):
    async def find_duble(self, x: int) -> int: ...


class StartModule(Module):
    _dubler: DublerSpec
    _timer_future: Future | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._timer_future = self.spawn(self._timer_loop())

    async def _timer_loop(self) -> None:
        i = 1
        import time

        while True:
            await asyncio.sleep(1.0)
            print("Finding duble of", i, "time=", time.time())
            ret = await self._dubler.find_duble(i)
            print("Found duble of", ret, "time=", time.time())
            i += 1
            if i == 3:
                raise Exception("asdf")

    @rpc
    def stop(self) -> None:
        if self._timer_future is not None:
            self._timer_future.cancel()
            self._timer_future = None
        super().stop()


blueprint = autoconnect(StartModule.blueprint(), Dubler.blueprint())

if __name__ == "__main__":
    ModuleCoordinator.build(blueprint).loop()
