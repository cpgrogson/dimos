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

from contextlib import suppress
from typing import TYPE_CHECKING, Any

from dimos.core.module import ModuleSpec
from dimos.utils.safe_thread_map import ExceptionGroup, safe_thread_map

if TYPE_CHECKING:
    from dimos.core.docker_module import DockerModuleOuter


class DockerWorkerManager:
    """Parallel deployment of Docker-backed modules."""

    @staticmethod
    def deploy_parallel(
        specs: list[ModuleSpec],
    ) -> list[DockerModuleOuter]:
        """Deploy multiple DockerModules in parallel.

        If any deployment fails, all successfully-started containers are
        stopped before an ExceptionGroup is raised.
        """
        from dimos.core.docker_module import DockerModuleOuter

        def _on_errors(
            _outcomes: list[Any], successes: list[DockerModuleOuter], errors: list[Exception]
        ) -> None:
            for mod in successes:
                with suppress(Exception):
                    mod.stop()
            raise ExceptionGroup("docker deploy_parallel failed", errors)

        def _deploy_one(spec: ModuleSpec) -> DockerModuleOuter:
            mod = DockerModuleOuter(spec[0], g=spec[1], **spec[2])  # type: ignore[arg-type]
            mod.build()
            return mod

        return safe_thread_map(specs, _deploy_one, _on_errors)
