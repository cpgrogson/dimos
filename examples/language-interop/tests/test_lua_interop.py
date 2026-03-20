"""Integration test: Lua robot-control script talks to Python simplerobot via LCM."""

from __future__ import annotations

import shutil
import subprocess

import pytest

from .conftest import LUA_DIR

pytestmark = pytest.mark.interop


@pytest.fixture(scope="module")
def lua_available() -> None:
    if shutil.which("lua") is None and shutil.which("lua5.4") is None:
        pytest.skip("lua not found on PATH")


def test_lua_receives_pose_and_publishes_twist(
    simplerobot: subprocess.Popen[str],
    lua_available: None,
) -> None:
    """Run the Lua script for a few seconds and verify message exchange."""
    lua_bin = shutil.which("lua") or shutil.which("lua5.4") or "lua"
    try:
        result = subprocess.run(
            [lua_bin, str(LUA_DIR / "main.lua")],
            capture_output=True,
            text=True,
            timeout=5,
            cwd=str(LUA_DIR),
        )
    except subprocess.TimeoutExpired as e:
        stdout = e.stdout or ""
        stderr = e.stderr or ""
    else:
        stdout = result.stdout
        stderr = result.stderr

    assert "[pose]" in stdout, (
        f"Lua script never received a PoseStamped.\nstdout: {stdout!r}\nstderr: {stderr!r}"
    )
    assert "[twist]" in stdout, (
        f"Lua script never published a Twist.\nstdout: {stdout!r}\nstderr: {stderr!r}"
    )
