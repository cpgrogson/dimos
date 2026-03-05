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

from __future__ import annotations

from contextlib import suppress
from typing import TYPE_CHECKING, Any

from dimos.core.docker_runner import DockerModule
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.module import Module

logger = setup_logger()


class DockerWorkerManager:
    """Manages DockerModule instances, mirroring WorkerManager's interface for docker-based modules."""

    def __init__(self) -> None:
        self._docker_modules: list[DockerModule] = []
        self._closed = False

    def deploy(self, module_class: type[Module], *args: Any, **kwargs: Any) -> DockerModule:
        if self._closed:
            raise RuntimeError("DockerWorkerManager is closed")

        logger.info("Deploying module in Docker.", module=module_class.__name__)
        dm = DockerModule(module_class, *args, **kwargs)
        try:
            dm.start()  # Docker modules must be running before streams/RPC can be wired
        except Exception:
            with suppress(Exception):
                dm.stop()
            raise
        self._docker_modules.append(dm)
        return dm

    def close_all(self) -> None:
        if self._closed:
            return
        self._closed = True

        logger.info("Stopping all Docker modules...")
        for dm in reversed(self._docker_modules):
            try:
                dm.stop()
            except Exception:
                logger.error("Error stopping Docker module", exc_info=True)

        self._docker_modules.clear()
        logger.info("All Docker modules stopped.")
