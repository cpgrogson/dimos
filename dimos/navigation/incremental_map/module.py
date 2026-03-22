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

"""IncrementalMap: lightweight global map module without full PGO.

Builds a global occupancy / point-cloud map incrementally from lidar + odometry
without requiring GTSAM.  Loop closure is detected via KD-tree proximity search
and verified (optionally) with lightweight point-to-point ICP.  On closure the
accumulated voxel map is rebuilt from corrected keyframe poses.

Design rationale: see dimos/navigation/incremental_map/DESIGN.md
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time

import numpy as np
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# ─── Config ──────────────────────────────────────────────────────────────────


class IncrementalMapConfig(ModuleConfig):
    """Configuration for IncrementalMap module."""

    # Voxel size for map accumulation and downsampling (metres)
    voxel_size: float = 0.15

    # Keyframe detection thresholds
    key_trans: float = 0.5
    """Accept new keyframe when robot has moved this many metres."""
    key_deg: float = 10.0
    """Accept new keyframe when robot has rotated this many degrees."""

    # Loop closure detection
    loop_search_radius: float = 3.0
    """Search within this radius (metres) for loop closure candidates."""
    loop_time_thresh: float = 10.0
    """Require at least this many seconds to have elapsed before detecting loop."""
    loop_score_thresh: float = 0.5
    """ICP mean-squared-error threshold (lower = more selective)."""
    loop_submap_half_range: int = 3
    """Number of keyframes on each side of candidate used as ICP target submap."""
    icp_max_iter: int = 30
    """Max ICP iterations for loop-closure verification."""
    icp_max_dist: float = 5.0
    """Max correspondence distance for ICP."""
    min_loop_detect_duration: float = 5.0
    """Minimum seconds between consecutive loop detection attempts."""

    # Map publishing
    map_publish_rate: float = 0.5
    """Rate at which the global map is published (Hz)."""

    # Body-frame input mode
    registered_input: bool = True
    """If True, incoming registered_scan is world-frame; transform back to body first."""


# ─── Internal data structures ────────────────────────────────────────────────


@dataclass
class _Keyframe:
    r_local: np.ndarray  # 3×3 rotation in raw-odom frame
    t_local: np.ndarray  # 3-vec translation in raw-odom frame
    r_global: np.ndarray  # 3×3 corrected rotation
    t_global: np.ndarray  # 3-vec corrected translation
    timestamp: float
    body_cloud: np.ndarray  # Nx3 points in body frame (downsampled)


# ─── Helpers ─────────────────────────────────────────────────────────────────


def _voxel_downsample(pts: np.ndarray, voxel_size: float) -> np.ndarray:
    """Voxel-grid downsampling (keep one point per voxel cell)."""
    if len(pts) == 0 or voxel_size <= 0:
        return pts
    keys = np.floor(pts / voxel_size).astype(np.int32)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


def _icp_verify(
    source: np.ndarray,
    target: np.ndarray,
    max_iter: int = 30,
    max_dist: float = 5.0,
) -> tuple[np.ndarray, float]:
    """Lightweight point-to-point ICP. Returns (4x4 transform, fitness score).

    Fitness score is the mean squared distance of inlier correspondences.
    Lower is better; inf means alignment failed.
    """
    if len(source) == 0 or len(target) == 0:
        return np.eye(4), float("inf")

    tree = KDTree(target)
    T = np.eye(4, dtype=np.float64)
    src = source.astype(np.float64).copy()

    for _ in range(max_iter):
        dists, idxs = tree.query(src)
        mask = dists < max_dist
        if mask.sum() < 5:
            return T, float("inf")

        p = src[mask]
        q = target[idxs[mask]].astype(np.float64)

        cp = p.mean(axis=0)
        cq = q.mean(axis=0)
        H = (p - cp).T @ (q - cq)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        t = cq - R @ cp

        dT = np.eye(4, dtype=np.float64)
        dT[:3, :3] = R
        dT[:3, 3] = t
        T = dT @ T
        src = (R @ src.T).T + t

        if np.linalg.norm(t) < 1e-6:
            break

    dists_final, _ = tree.query(src)
    mask = dists_final < max_dist
    fitness = float(np.mean(dists_final[mask] ** 2)) if mask.sum() > 0 else float("inf")
    return T, fitness


# ─── Core algorithm ──────────────────────────────────────────────────────────


class _IncrementalMapCore:
    """Pure-algorithm core (no Module I/O dependencies)."""

    def __init__(self, config: IncrementalMapConfig) -> None:
        self._cfg = config
        self._keyframes: list[_Keyframe] = []
        self._last_loop_time: float = 0.0
        # Correction offset from loop closure (applied to raw odom to get global pose)
        self._r_offset: np.ndarray = np.eye(3)
        self._t_offset: np.ndarray = np.zeros(3)
        self._loop_count: int = 0

    # ── Keyframe management ──

    def _is_keyframe(self, r: np.ndarray, t: np.ndarray) -> bool:
        if not self._keyframes:
            return True
        last = self._keyframes[-1]
        dt = np.linalg.norm(t - last.t_local)
        q_cur = Rotation.from_matrix(r).as_quat()
        q_last = Rotation.from_matrix(last.r_local).as_quat()
        dot = float(np.clip(np.dot(q_cur, q_last), -1.0, 1.0))
        ddeg = float(np.degrees(2.0 * np.arccos(abs(dot))))
        return bool(dt > self._cfg.key_trans or ddeg > self._cfg.key_deg)

    def add_scan(
        self,
        r_local: np.ndarray,
        t_local: np.ndarray,
        body_cloud: np.ndarray,
        timestamp: float,
    ) -> bool:
        """Try to add a keyframe. Returns True if a keyframe was added."""
        if not self._is_keyframe(r_local, t_local):
            return False

        r_global = self._r_offset @ r_local
        t_global = self._r_offset @ t_local + self._t_offset

        kf = _Keyframe(
            r_local=r_local.copy(),
            t_local=t_local.copy(),
            r_global=r_global.copy(),
            t_global=t_global.copy(),
            timestamp=timestamp,
            body_cloud=_voxel_downsample(body_cloud, self._cfg.voxel_size),
        )
        self._keyframes.append(kf)
        return True

    # ── Loop closure ──

    def _get_submap(self, idx: int, half_range: int) -> np.ndarray:
        lo = max(0, idx - half_range)
        hi = min(len(self._keyframes) - 1, idx + half_range)
        parts = []
        for i in range(lo, hi + 1):
            kf = self._keyframes[i]
            world = (kf.r_global @ kf.body_cloud.T).T + kf.t_global
            parts.append(world)
        if not parts:
            return np.empty((0, 3))
        return _voxel_downsample(np.vstack(parts), self._cfg.voxel_size)

    def detect_and_correct_loop(self) -> bool:
        """Search for loop closure and correct if found. Returns True if corrected."""
        n = len(self._keyframes)
        if n < 5:
            return False

        cur_kf = self._keyframes[-1]
        cur_time = cur_kf.timestamp

        # Rate-limit loop detection
        if cur_time - self._last_loop_time < self._cfg.min_loop_detect_duration:
            return False

        # Build KD-tree of previous keyframe positions (exclude recent ones)
        past_positions = np.array([kf.t_global for kf in self._keyframes[:-1]])
        tree = KDTree(past_positions)

        # Find candidates within search radius
        candidates = tree.query_ball_point(cur_kf.t_global, self._cfg.loop_search_radius)
        if not candidates:
            return False

        # Filter by time gap
        valid_candidates = [
            i
            for i in candidates
            if abs(cur_time - self._keyframes[i].timestamp) > self._cfg.loop_time_thresh
        ]
        if not valid_candidates:
            return False

        # Pick the closest candidate
        best_idx = min(
            valid_candidates,
            key=lambda i: np.linalg.norm(cur_kf.t_global - self._keyframes[i].t_global),
        )

        # ICP verification
        cur_idx = n - 1
        target_submap = self._get_submap(best_idx, self._cfg.loop_submap_half_range)
        source_scan = self._get_submap(cur_idx, 0)

        transform, fitness = _icp_verify(
            source_scan,
            target_submap,
            max_iter=self._cfg.icp_max_iter,
            max_dist=self._cfg.icp_max_dist,
        )

        if fitness > self._cfg.loop_score_thresh:
            logger.debug(
                f"[IncrementalMap] Loop candidate {best_idx}<->{cur_idx} rejected "
                f"(fitness={fitness:.4f} > {self._cfg.loop_score_thresh})"
            )
            return False

        # Compute correction: ICP transform applied to current global pose
        R_icp = transform[:3, :3]
        t_icp = transform[:3, 3]

        # The ICP brought source (current) to align with target (best_idx)
        # Correction: new_global = R_icp @ old_global + t_icp
        new_r_cur = R_icp @ cur_kf.r_global
        new_t_cur = R_icp @ cur_kf.t_global + t_icp

        # Compute incremental correction offset so future poses use corrected frame
        # new_global = R_offset_new @ r_local
        # R_offset_new = new_r_cur @ r_local.T
        self._r_offset = new_r_cur @ cur_kf.r_local.T
        self._t_offset = new_t_cur - self._r_offset @ cur_kf.t_local

        # Recompute all keyframe global poses with the correction
        for kf in self._keyframes:
            kf.r_global = self._r_offset @ kf.r_local
            kf.t_global = self._r_offset @ kf.t_local + self._t_offset

        self._last_loop_time = cur_time
        self._loop_count += 1
        logger.info(
            f"[IncrementalMap] Loop closure #{self._loop_count}: "
            f"keyframe {best_idx}<->{cur_idx} (fitness={fitness:.4f})"
        )
        return True

    # ── Map building ──

    def build_global_map(self) -> np.ndarray:
        """Return voxel-downsampled global point cloud from all keyframes."""
        if not self._keyframes:
            return np.empty((0, 3), dtype=np.float32)
        parts = []
        for kf in self._keyframes:
            world = (kf.r_global @ kf.body_cloud.T).T + kf.t_global
            parts.append(world)
        cloud = np.vstack(parts).astype(np.float32)
        return _voxel_downsample(cloud, self._cfg.voxel_size)

    def get_corrected_pose(
        self, r_local: np.ndarray, t_local: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Apply current correction offset to a raw-odom pose."""
        return (
            self._r_offset @ r_local,
            self._r_offset @ t_local + self._t_offset,
        )

    @property
    def num_keyframes(self) -> int:
        return len(self._keyframes)

    @property
    def loop_count(self) -> int:
        return self._loop_count


# ─── Module ──────────────────────────────────────────────────────────────────


class IncrementalMap(Module[IncrementalMapConfig]):
    """Lightweight incremental global map module (no GTSAM required).

    Builds a global point-cloud map incrementally from lidar + odometry.
    Detects loop closures via KD-tree proximity + ICP verification.
    On loop closure, rebuilds the map from corrected keyframe poses.

    Ports:
        raw_odom (In[PoseStamped]): Raw pose from robot (matches GO2Connection output).
        registered_scan (In[PointCloud2]): World-frame registered lidar.
        global_map (Out[PointCloud2]): Accumulated corrected map.
        corrected_odom (Out[Odometry]): Loop-closure-corrected odometry.
        odom (Out[PoseStamped]): Corrected pose as PoseStamped for downstream consumers.
    """

    default_config = IncrementalMapConfig

    raw_odom: In[PoseStamped]
    registered_scan: In[PointCloud2]

    global_map: Out[PointCloud2]
    corrected_odom: Out[Odometry]
    odom: Out[PoseStamped]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._core: _IncrementalMapCore | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._last_r = np.eye(3)
        self._last_t = np.zeros(3)
        self._last_ts = 0.0
        self._has_odom = False
        self._last_map_publish = 0.0

    def __getstate__(self) -> dict[str, object]:
        state: dict[str, object] = super().__getstate__()  # type: ignore[no-untyped-call]
        for k in ("_lock", "_thread", "_core"):
            state.pop(k, None)
        return state

    def __setstate__(self, state: dict[str, object]) -> None:
        super().__setstate__(state)
        self._lock = threading.Lock()
        self._thread = None
        self._core = None

    @rpc
    def start(self) -> None:
        super().start()
        self._core = _IncrementalMapCore(self.config)
        self.raw_odom.subscribe(self._on_odom)
        self.registered_scan.subscribe(self._on_scan)
        self._running = True
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()
        logger.info("[IncrementalMap] started")

    @rpc
    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        super().stop()

    def _on_odom(self, msg: PoseStamped) -> None:
        pose = msg.pose
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        r = Rotation.from_quat(q).as_matrix()
        t = np.array([pose.x, pose.y, pose.z])
        ts = msg.ts if msg.ts else time.time()
        with self._lock:
            self._last_r = r
            self._last_t = t
            self._last_ts = ts
            self._has_odom = True
            core = self._core

        # Always publish corrected odom (applies loop closure offset if any).
        if core is not None:
            r_corr, t_corr = core.get_corrected_pose(r, t)
            self._publish_corrected_odom(r_corr, t_corr, ts)

    def _on_scan(self, cloud: PointCloud2) -> None:
        points, _ = cloud.as_numpy()
        if len(points) == 0:
            return

        with self._lock:
            if not self._has_odom:
                return
            r_local = self._last_r.copy()
            t_local = self._last_t.copy()
            ts = self._last_ts

        core = self._core
        assert core is not None

        # Convert world-frame scan back to body frame if needed
        if self.config.registered_input:
            body_pts = (r_local.T @ (points[:, :3].T - t_local[:, None])).T
        else:
            body_pts = points[:, :3].copy()

        added = core.add_scan(r_local, t_local, body_pts, ts)

        if added:
            loop_closed = core.detect_and_correct_loop()
            if loop_closed:
                logger.info(
                    f"[IncrementalMap] Map corrected after loop closure "
                    f"(keyframes={core.num_keyframes})"
                )
            # Publish corrected odom on every keyframe
            r_corr, t_corr = core.get_corrected_pose(r_local, t_local)
            self._publish_corrected_odom(r_corr, t_corr, ts)

    def _publish_corrected_odom(self, r: np.ndarray, t: np.ndarray, ts: float) -> None:
        q = Rotation.from_matrix(r).as_quat()
        pose = Pose(
            position=[float(t[0]), float(t[1]), float(t[2])],
            orientation=[float(q[0]), float(q[1]), float(q[2]), float(q[3])],
        )
        self.corrected_odom.publish(
            Odometry(
                ts=ts,
                frame_id="map",
                child_frame_id="sensor",
                pose=pose,
            )
        )
        # Also publish as PoseStamped for downstream consumers (planner, explorer).
        self.odom.publish(
            PoseStamped(
                ts=ts,
                frame_id="map",
                pose=pose,
            )
        )

    def _publish_loop(self) -> None:
        """Publish global map at configured rate."""
        core = self._core
        assert core is not None
        rate = self.config.map_publish_rate
        interval = 1.0 / rate if rate > 0 else 2.0

        while self._running:
            t0 = time.monotonic()
            now = time.time()

            if now - self._last_map_publish > interval and core.num_keyframes > 0:
                cloud_np = core.build_global_map()
                if len(cloud_np) > 0:
                    self.global_map.publish(
                        PointCloud2.from_numpy(cloud_np, frame_id="map", timestamp=now)
                    )
                    logger.debug(
                        f"[IncrementalMap] Map published: {len(cloud_np)} pts, "
                        f"{core.num_keyframes} keyframes, loops={core.loop_count}"
                    )
                self._last_map_publish = now

            elapsed = time.monotonic() - t0
            sleep_time = max(0.05, interval - elapsed)
            time.sleep(sleep_time)


__all__ = [
    "IncrementalMap",
    "IncrementalMapConfig",
    "_IncrementalMapCore",
    "_icp_verify",
    "_voxel_downsample",
]
