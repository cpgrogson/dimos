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

"""Tests for DIM-681: daemon mode, run registry, and health checks."""

from __future__ import annotations

import os
import re
import signal
import time
from typing import TYPE_CHECKING
from unittest import mock

import pytest

# ---------------------------------------------------------------------------
# Registry tests
# ---------------------------------------------------------------------------
from dimos.core.run_registry import (
    RunEntry,
    check_port_conflicts,
    cleanup_stale,
    generate_run_id,
)

if TYPE_CHECKING:
    from pathlib import Path


@pytest.fixture()
def tmp_registry(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    """Redirect the registry to a temp dir for test isolation."""
    monkeypatch.setattr("dimos.core.run_registry.REGISTRY_DIR", tmp_path)
    return tmp_path


def _make_entry(
    run_id: str = "20260306-120000-test",
    pid: int | None = None,
    mcp_port: int = 9990,
    grpc_port: int = 9877,
) -> RunEntry:
    return RunEntry(
        run_id=run_id,
        pid=pid if pid is not None else os.getpid(),
        blueprint="test",
        started_at="2026-03-06T12:00:00Z",
        log_dir="/tmp/test-logs",
        cli_args=["test"],
        config_overrides={},
        mcp_port=mcp_port,
        grpc_port=grpc_port,
    )


class TestRunEntryCRUD:
    """test_run_entry_save_load_remove — full CRUD cycle."""

    def test_run_entry_save_load_remove(self, tmp_registry: Path):
        entry = _make_entry()
        entry.save()

        loaded = RunEntry.load(entry.registry_path)
        assert loaded.run_id == entry.run_id
        assert loaded.pid == entry.pid
        assert loaded.blueprint == entry.blueprint
        assert loaded.mcp_port == entry.mcp_port
        assert loaded.grpc_port == entry.grpc_port

        entry.remove()
        assert not entry.registry_path.exists()

    def test_save_creates_directory(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
        nested = tmp_path / "a" / "b" / "c"
        monkeypatch.setattr("dimos.core.run_registry.REGISTRY_DIR", nested)
        entry = _make_entry()
        entry.save()
        assert entry.registry_path.exists()

    def test_remove_idempotent(self, tmp_registry: Path):
        entry = _make_entry()
        entry.remove()  # file doesn't exist — should not raise
        entry.save()
        entry.remove()
        entry.remove()  # already gone — still fine


class TestGenerateRunId:
    """test_generate_run_id_format — timestamp + sanitized blueprint name."""

    def test_generate_run_id_format(self):
        rid = generate_run_id("unitree-go2")
        # Pattern: YYYYMMDD-HHMMSS-<name>
        assert re.match(r"^\d{8}-\d{6}-unitree-go2$", rid), f"unexpected format: {rid}"

    def test_sanitizes_slashes(self):
        rid = generate_run_id("path/to/bp")
        assert "/" not in rid

    def test_sanitizes_spaces(self):
        rid = generate_run_id("my blueprint")
        assert " " not in rid


class TestCleanupStale:
    """Stale PID cleanup tests."""

    def test_cleanup_stale_removes_dead_pids(self, tmp_registry: Path):
        # PID 2_000_000_000 is almost certainly not alive
        entry = _make_entry(pid=2_000_000_000)
        entry.save()
        assert entry.registry_path.exists()

        removed = cleanup_stale()
        assert removed == 1
        assert not entry.registry_path.exists()

    def test_cleanup_stale_keeps_alive_pids(self, tmp_registry: Path):
        # Our own PID is alive
        entry = _make_entry(pid=os.getpid())
        entry.save()

        removed = cleanup_stale()
        assert removed == 0
        assert entry.registry_path.exists()

    def test_cleanup_corrupt_file(self, tmp_registry: Path):
        bad = tmp_registry / "corrupt.json"
        bad.write_text("not json{{{")
        removed = cleanup_stale()
        assert removed == 1
        assert not bad.exists()


class TestPortConflicts:
    """Port conflict detection."""

    def test_port_conflict_detection(self, tmp_registry: Path):
        entry = _make_entry(pid=os.getpid(), mcp_port=9990, grpc_port=9877)
        entry.save()

        conflict = check_port_conflicts(mcp_port=9990, grpc_port=9877)
        assert conflict is not None
        assert conflict.run_id == entry.run_id

    def test_port_conflict_mcp_only(self, tmp_registry: Path):
        entry = _make_entry(pid=os.getpid(), mcp_port=9990, grpc_port=1111)
        entry.save()

        conflict = check_port_conflicts(mcp_port=9990, grpc_port=2222)
        assert conflict is not None

    def test_port_conflict_grpc_only(self, tmp_registry: Path):
        entry = _make_entry(pid=os.getpid(), mcp_port=1111, grpc_port=9877)
        entry.save()

        conflict = check_port_conflicts(mcp_port=2222, grpc_port=9877)
        assert conflict is not None

    def test_port_conflict_no_false_positive(self, tmp_registry: Path):
        entry = _make_entry(pid=os.getpid(), mcp_port=8000, grpc_port=8001)
        entry.save()

        conflict = check_port_conflicts(mcp_port=9990, grpc_port=9877)
        assert conflict is None


# ---------------------------------------------------------------------------
# Health check tests
# ---------------------------------------------------------------------------

from dimos.core.daemon import health_check


def _mock_worker(pid: int | None = 1234, worker_id: int = 0):
    """Create a mock Worker with a controllable pid property."""
    w = mock.MagicMock()
    w.pid = pid
    w.worker_id = worker_id
    return w


def _mock_coordinator(workers: list | None = None):
    """Create a mock ModuleCoordinator with mock WorkerManager."""
    coord = mock.MagicMock()
    if workers is not None:
        coord._client.workers = workers
    else:
        coord._client = None
    return coord


class TestHealthCheckAllHealthy:
    """test_health_check_all_healthy — mock workers with alive PIDs → passes."""

    def test_health_check_all_healthy(self):
        workers = [_mock_worker(pid=os.getpid(), worker_id=i) for i in range(3)]
        coord = _mock_coordinator(workers)

        result = health_check(coord, timeout=0.5, poll_interval=0.1)
        assert result is True


class TestHealthCheckImmediateDeath:
    """test_health_check_immediate_death — worker with pid=None → fails immediately."""

    def test_health_check_immediate_death(self):
        dead_worker = _mock_worker(pid=None, worker_id=0)
        coord = _mock_coordinator([dead_worker])

        start = time.monotonic()
        result = health_check(coord, timeout=5.0, poll_interval=0.1)
        elapsed = time.monotonic() - start

        assert result is False
        # Should fail almost immediately, not wait for the full timeout
        assert elapsed < 1.0


class TestHealthCheckDelayedDeath:
    """test_health_check_delayed_death — worker alive initially, then pid → None."""

    def test_health_check_delayed_death(self):
        worker = _mock_worker(pid=os.getpid(), worker_id=0)

        # After a few accesses, pid becomes None (simulating delayed crash)
        call_count = 0
        real_pid = os.getpid()

        def pid_getter():
            nonlocal call_count
            call_count += 1
            if call_count > 3:
                return None
            return real_pid

        type(worker).pid = mock.PropertyMock(side_effect=pid_getter)
        coord = _mock_coordinator([worker])

        result = health_check(coord, timeout=5.0, poll_interval=0.05)
        assert result is False


class TestHealthCheckNoWorkers:
    """test_health_check_no_workers — empty worker list → fails."""

    def test_health_check_no_workers(self):
        coord = _mock_coordinator(workers=[])
        result = health_check(coord, timeout=0.5, poll_interval=0.1)
        assert result is False

    def test_health_check_no_client(self):
        coord = _mock_coordinator(workers=None)  # _client = None
        result = health_check(coord, timeout=0.5, poll_interval=0.1)
        assert result is False


class TestHealthCheckPartialDeath:
    """test_health_check_partial_death — 3 workers, 1 dies → fails."""

    def test_health_check_partial_death(self):
        w1 = _mock_worker(pid=os.getpid(), worker_id=0)
        w2 = _mock_worker(pid=os.getpid(), worker_id=1)
        w3 = _mock_worker(pid=None, worker_id=2)  # dead

        coord = _mock_coordinator([w1, w2, w3])
        result = health_check(coord, timeout=0.5, poll_interval=0.1)
        assert result is False


# ---------------------------------------------------------------------------
# Daemon tests
# ---------------------------------------------------------------------------

from dimos.core.daemon import _shutdown_handler, daemonize, install_signal_handlers


class TestDaemonize:
    """test_daemonize_creates_log_dir."""

    def test_daemonize_creates_log_dir(self, tmp_path: Path):
        log_dir = tmp_path / "nested" / "logs"
        assert not log_dir.exists()

        # We can't actually double-fork in tests (child would continue running
        # pytest), so we mock os.fork to return >0 both times (parent path).
        with mock.patch("os.fork", return_value=1), pytest.raises(SystemExit):
            # Parent calls os._exit(0) which we let raise
            with mock.patch("os._exit", side_effect=SystemExit(0)):
                daemonize(log_dir)

        assert log_dir.exists()


class TestSignalHandler:
    """test_signal_handler_cleans_registry."""

    def test_signal_handler_cleans_registry(self, tmp_registry: Path):
        entry = _make_entry()
        entry.save()
        assert entry.registry_path.exists()

        coord = mock.MagicMock()
        install_signal_handlers(entry, coord)

        with pytest.raises(SystemExit):
            _shutdown_handler(signal.SIGTERM, None)

        # Registry file should be cleaned up
        assert not entry.registry_path.exists()
        # Coordinator should have been stopped
        coord.stop.assert_called_once()

    def test_signal_handler_tolerates_stop_error(self, tmp_registry: Path):
        entry = _make_entry()
        entry.save()

        coord = mock.MagicMock()
        coord.stop.side_effect = RuntimeError("boom")
        install_signal_handlers(entry, coord)

        with pytest.raises(SystemExit):
            _shutdown_handler(signal.SIGTERM, None)

        # Entry still removed even if stop() throws
        assert not entry.registry_path.exists()
