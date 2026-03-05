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

"""
Smoke tests for Docker module deployment routing.

These tests verify that the ModuleCoordinator correctly detects and routes
docker modules to DockerModule WITHOUT actually running Docker.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING
from unittest.mock import MagicMock, patch

import pytest

from dimos.core.docker_runner import DockerModuleConfig, is_docker_module
from dimos.core.module import Module
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.stream import Out

if TYPE_CHECKING:
    from pathlib import Path

# -- Fixtures: fake module classes -------------------------------------------


@dataclass
class FakeDockerConfig(DockerModuleConfig):
    docker_image: str = "fake:latest"
    docker_file: Path | None = None
    docker_gpus: str | None = None
    docker_rm: bool = True
    docker_restart_policy: str = "no"


class FakeDockerModule(Module["FakeDockerConfig"]):
    default_config = FakeDockerConfig
    output: Out[str]


class FakeRegularModule(Module):
    output: Out[str]


# -- Tests -------------------------------------------------------------------


class TestIsDockerModule:
    def test_docker_module_detected(self):
        assert is_docker_module(FakeDockerModule) is True

    def test_regular_module_not_detected(self):
        assert is_docker_module(FakeRegularModule) is False

    def test_plain_class_not_detected(self):
        assert is_docker_module(str) is False

    def test_no_default_config(self):
        class Bare(Module):
            pass

        # Module has default_config = ModuleConfig, which is not DockerModuleConfig
        assert is_docker_module(Bare) is False


class TestModuleCoordinatorDockerRouting:
    @patch("dimos.core.module_coordinator.DockerModule")
    @patch("dimos.core.module_coordinator.WorkerManager")
    def test_deploy_routes_docker_module(self, mock_worker_manager_cls, mock_docker_module_cls):
        mock_worker_mgr = MagicMock()
        mock_worker_manager_cls.return_value = mock_worker_mgr

        mock_dm = MagicMock()
        mock_docker_module_cls.return_value = mock_dm

        coordinator = ModuleCoordinator()
        coordinator.start()

        result = coordinator.deploy(FakeDockerModule)

        # Should NOT go through worker manager
        mock_worker_mgr.deploy.assert_not_called()
        # Should create a DockerModule and start it
        mock_docker_module_cls.assert_called_once_with(FakeDockerModule)
        mock_dm.start.assert_called_once()
        assert result is mock_dm
        # Should be tracked
        assert coordinator.get_instance(FakeDockerModule) is mock_dm

        coordinator.stop()

    @patch("dimos.core.module_coordinator.DockerModule")
    @patch("dimos.core.module_coordinator.WorkerManager")
    def test_deploy_docker_cleans_up_on_start_failure(
        self, mock_worker_manager_cls, mock_docker_module_cls
    ):
        mock_worker_mgr = MagicMock()
        mock_worker_manager_cls.return_value = mock_worker_mgr

        mock_dm = MagicMock()
        mock_dm.start.side_effect = RuntimeError("start failed")
        mock_docker_module_cls.return_value = mock_dm

        coordinator = ModuleCoordinator()
        coordinator.start()

        with pytest.raises(RuntimeError, match="start failed"):
            coordinator.deploy(FakeDockerModule)

        # stop() called to clean up the failed container
        mock_dm.stop.assert_called_once()

        coordinator.stop()

    @patch("dimos.core.module_coordinator.WorkerManager")
    def test_deploy_routes_regular_module_to_worker_manager(self, mock_worker_manager_cls):
        mock_worker_mgr = MagicMock()
        mock_worker_manager_cls.return_value = mock_worker_mgr
        mock_proxy = MagicMock()
        mock_worker_mgr.deploy.return_value = mock_proxy

        coordinator = ModuleCoordinator()
        coordinator.start()

        result = coordinator.deploy(FakeRegularModule)

        mock_worker_mgr.deploy.assert_called_once_with(FakeRegularModule)
        assert result is mock_proxy

        coordinator.stop()

    @patch("dimos.core.module_coordinator.DockerModule")
    @patch("dimos.core.module_coordinator.WorkerManager")
    def test_deploy_parallel_separates_docker_and_regular(
        self, mock_worker_manager_cls, mock_docker_module_cls
    ):
        mock_worker_mgr = MagicMock()
        mock_worker_manager_cls.return_value = mock_worker_mgr

        regular_proxy = MagicMock()
        mock_worker_mgr.deploy_parallel.return_value = [regular_proxy]

        mock_dm = MagicMock()
        mock_docker_module_cls.return_value = mock_dm

        coordinator = ModuleCoordinator()
        coordinator.start()

        specs = [
            (FakeRegularModule, (), {}),
            (FakeDockerModule, (), {}),
        ]
        results = coordinator.deploy_parallel(specs)

        # Regular module goes through worker manager
        mock_worker_mgr.deploy_parallel.assert_called_once_with([(FakeRegularModule, (), {})])
        # Docker module gets its own DockerModule
        mock_docker_module_cls.assert_called_once_with(FakeDockerModule)
        mock_dm.start.assert_called_once()

        # Results are in original order
        assert results[0] is regular_proxy
        assert results[1] is mock_dm

        coordinator.stop()

    @patch("dimos.core.module_coordinator.DockerModule")
    @patch("dimos.core.module_coordinator.WorkerManager")
    def test_stop_cleans_up_docker_modules(self, mock_worker_manager_cls, mock_docker_module_cls):
        mock_worker_mgr = MagicMock()
        mock_worker_manager_cls.return_value = mock_worker_mgr

        mock_dm = MagicMock()
        mock_docker_module_cls.return_value = mock_dm

        coordinator = ModuleCoordinator()
        coordinator.start()
        coordinator.deploy(FakeDockerModule)
        coordinator.stop()

        # stop() called exactly once (no double cleanup)
        assert mock_dm.stop.call_count == 1
        # Worker manager also closed
        mock_worker_mgr.close_all.assert_called_once()
