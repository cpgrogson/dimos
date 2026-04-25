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

"""Viser-based 3D viewer module for dimos.

Streams a Gaussian splat scene + the robot (MJCF meshes, FK from
``/coordinator/joint_state`` + ``/odom``) into a browser at
http://localhost:<port>/.

This is render-only — the viewer subscribes to existing LCM topics and
does not feed back into the control path.  Teleop continues to come
from the existing command-center dashboard.
"""

from __future__ import annotations

from pathlib import Path as FilePath
import threading
import time
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import JointState
from dimos.utils.logging_config import setup_logger
from dimos.visualization.viser.robot_meshes import (
    RobotMeshes,
    apply_state,
    dimos_joint_to_mjcf,
    load_robot_meshes,
)
from dimos.visualization.viser.splat import SplatAlignment, load_splat

logger = setup_logger()


class ViserRenderModule(Module):
    """Viser viewer that overlays the live robot on a Gaussian splat.

    Inputs:
        joint_state: per-joint q values from the coordinator.
        odom: base pose from the sim (or future real-hw) adapter.
    """

    joint_state: In[JointState]
    odom: In[PoseStamped]

    def __init__(
        self,
        splat_path: str | FilePath,
        mjcf_path: str | FilePath,
        *,
        port: int = 8082,
        alignment_yaml: str | FilePath | None = None,
        render_hz: float = 30.0,
        cfg: GlobalConfig = global_config,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._global_config = cfg
        self._splat_path = FilePath(splat_path)
        self._mjcf_path = FilePath(mjcf_path)
        self._alignment_yaml = FilePath(alignment_yaml) if alignment_yaml else None
        self._port = port
        self._render_dt = 1.0 / float(render_hz)

        # Mutable shared state — written from In subscribers, read from
        # the render loop.  Plain dict + lock; values are lightweight.
        self._state_lock = threading.Lock()
        self._latest_joints: dict[str, float] = {}
        self._latest_base_pos: np.ndarray | None = None
        self._latest_base_wxyz: np.ndarray | None = None

        self._server: Any = None  # viser.ViserServer
        self._body_frames: dict[int, Any] = {}  # body_id -> viser frame handle
        self._robot: RobotMeshes | None = None
        self._render_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    @rpc
    def start(self) -> None:
        super().start()

        import viser

        alignment = (
            SplatAlignment.from_yaml(self._alignment_yaml)
            if self._alignment_yaml and self._alignment_yaml.exists()
            else SplatAlignment()
        )

        logger.info(f"Viser: loading splat from {self._splat_path}")
        splat = load_splat(self._splat_path, alignment=alignment)
        logger.info(f"Viser: loaded {len(splat.centers)} Gaussians")

        logger.info(f"Viser: loading robot meshes from {self._mjcf_path}")
        from dimos.simulation.mujoco.model import get_assets

        self._robot = load_robot_meshes(self._mjcf_path, assets=get_assets())
        logger.info(
            f"Viser: {len(self._robot.geoms)} visual meshes across "
            f"{len(self._robot.body_names)} bodies"
        )

        self._server = viser.ViserServer(host="0.0.0.0", port=self._port)
        logger.info(f"Viser viewer: http://localhost:{self._port}/")

        self._server.scene.add_gaussian_splats(
            "/splat",
            centers=splat.centers,
            covariances=splat.covariances,
            rgbs=splat.rgbs,
            opacities=splat.opacities,
        )

        # One frame per body; meshes are added as children so they
        # follow when the body frame moves.
        for body_id, body_name in enumerate(self._robot.body_names):
            self._body_frames[body_id] = self._server.scene.add_frame(
                f"/robot/{body_name}",
                show_axes=False,
            )
        for i, geom in enumerate(self._robot.geoms):
            color_rgb = (
                int(geom.rgba[0] * 255),
                int(geom.rgba[1] * 255),
                int(geom.rgba[2] * 255),
            )
            self._server.scene.add_mesh_simple(
                f"/robot/{geom.body_name}/geom_{i}",
                vertices=geom.vertices,
                faces=geom.faces,
                color=color_rgb,
                opacity=float(geom.rgba[3]) if geom.rgba[3] > 0 else 1.0,
                position=tuple(geom.local_pos),
                wxyz=tuple(geom.local_wxyz),
            )

        try:
            unsub = self.joint_state.subscribe(self._on_joint_state)
            self._disposables.add(Disposable(unsub))
        except Exception as e:
            logger.warning(f"Viser: joint_state subscribe failed: {e}")

        try:
            unsub = self.odom.subscribe(self._on_odom)
            self._disposables.add(Disposable(unsub))
        except Exception as e:
            logger.warning(f"Viser: odom subscribe failed: {e}")

        self._render_thread = threading.Thread(
            target=self._render_loop, name="viser-render", daemon=True
        )
        self._render_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._render_thread and self._render_thread.is_alive():
            self._render_thread.join(timeout=2.0)
        if self._server is not None:
            try:
                self._server.stop()
            except Exception:
                pass
        super().stop()

    def _on_joint_state(self, msg: JointState) -> None:
        names = list(msg.name)
        positions = list(msg.position)
        if not names or len(names) != len(positions):
            return
        with self._state_lock:
            for n, q in zip(names, positions, strict=False):
                self._latest_joints[dimos_joint_to_mjcf(n)] = float(q)

    def _on_odom(self, msg: PoseStamped) -> None:
        with self._state_lock:
            self._latest_base_pos = np.array(
                [msg.position.x, msg.position.y, msg.position.z],
                dtype=np.float64,
            )
            # PoseStamped quaternion is (x, y, z, w); MuJoCo / Viser want (w, x, y, z).
            self._latest_base_wxyz = np.array(
                [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z],
                dtype=np.float64,
            )

    def _render_loop(self) -> None:
        assert self._robot is not None
        next_tick = time.monotonic()
        while not self._stop_event.is_set():
            with self._state_lock:
                joints = dict(self._latest_joints)
                base_pos = None if self._latest_base_pos is None else self._latest_base_pos.copy()
                base_wxyz = (
                    None if self._latest_base_wxyz is None else self._latest_base_wxyz.copy()
                )

            try:
                apply_state(
                    self._robot,
                    base_pos=base_pos,
                    base_wxyz=base_wxyz,
                    joint_positions=joints,
                )
                xpos = self._robot.data.xpos
                xquat = self._robot.data.xquat
                for body_id, frame in self._body_frames.items():
                    frame.position = tuple(float(x) for x in xpos[body_id])
                    frame.wxyz = tuple(float(x) for x in xquat[body_id])
            except Exception as e:
                logger.debug(f"Viser render tick failed: {e}")

            next_tick += self._render_dt
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()


viser_render = ViserRenderModule.blueprint

__all__ = ["ViserRenderModule", "viser_render"]
