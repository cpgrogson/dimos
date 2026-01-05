# Copyright 2025 Dimensional Inc.
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

import threading
from typing import TYPE_CHECKING, Any

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs import Vector3
    from dimos.perception.detection.type.detection3d.object import Object

logger = setup_logger()


class ObjectDB:
    """Spatial memory database for 3D object detections.

    Maintains two tiers of objects internally:
    - _pending_objects: Recently detected objects (detection_count < threshold)
    - _objects: Confirmed permanent objects (detection_count >= threshold)

    Deduplication uses two heuristics:
    1. track_id match from YOLOE tracker (immediate match)
    2. Center distance within threshold (spatial proximity match)
    """

    def __init__(
        self,
        distance_threshold: float = 0.5,
        min_detections_for_permanent: int = 5,
        require_same_name_for_distance_match: bool = True,
        pending_ttl_seconds: float = 30.0,
    ) -> None:
        self._distance_threshold = distance_threshold
        self._min_detections = min_detections_for_permanent
        self._require_same_name_for_distance_match = require_same_name_for_distance_match
        self._pending_ttl_seconds = float(pending_ttl_seconds)

        # Internal storage - keyed by object_id
        self._pending_objects: dict[str, Object] = {}
        self._objects: dict[str, Object] = {}  # Permanent objects

        # track_id -> object_id mapping for fast lookup
        self._track_id_map: dict[int, str] = {}

        self._lock = threading.RLock()

    # ─────────────────────────────────────────────────────────────────
    # Public Methods
    # ─────────────────────────────────────────────────────────────────

    def add_objects(self, objects: list[Object]) -> list[Object]:
        """Add multiple objects to the database with deduplication.

        Args:
            objects: List of Object instances from object_scene_registration

        Returns:
            List of updated/created Object instances
        """
        # Opportunistically prune stale pending objects to cap memory usage.
        # Many false positives never reach permanence; without pruning, they accumulate forever.
        now = max((float(o.ts) for o in objects), default=0.0)
        if now <= 0.0:
            import time as _time

            now = _time.time()
        self._prune_stale_pending(now)

        results: list[Object] = []
        for obj in objects:
            updated_obj = self._add_object(obj)
            results.append(updated_obj)
        return results

    def get_objects(self) -> list[Object]:
        """Get all permanent objects (detection_count >= threshold)."""
        with self._lock:
            return list(self._objects.values())

    def get_by_track_id(self, track_id: int) -> Object | None:
        """Get object by track_id (searches both pending and permanent objects).

        Args:
            track_id: The track_id to search for

        Returns:
            Object instance or None if not found
        """
        with self._lock:
            obj_id = self._track_id_map.get(track_id)
            if obj_id:
                return self._objects.get(obj_id) or self._pending_objects.get(obj_id)
            return None

    def get_by_object_id(self, object_id: str) -> Object | None:
        """Get an object by stable object_id (searches pending and permanent)."""
        with self._lock:
            return self._objects.get(object_id) or self._pending_objects.get(object_id)

    def is_permanent(self, object_id: str) -> bool:
        """Check if an object_id refers to a permanent object."""
        with self._lock:
            return object_id in self._objects

    def find_by_name(self, name: str) -> list[Object]:
        """Find all permanent objects with matching name."""
        with self._lock:
            return [obj for obj in self._objects.values() if obj.name == name]

    def find_nearest(
        self,
        position: Vector3,
        name: str | None = None,
    ) -> Object | None:
        """Find nearest permanent object to a position, optionally filtered by name.

        Args:
            position: Position to search from
            name: Optional name filter

        Returns:
            Nearest Object or None if no objects found
        """
        with self._lock:
            candidates = [
                obj
                for obj in self._objects.values()
                if obj.center is not None and (name is None or obj.name == name)
            ]

            if not candidates:
                return None

            return min(candidates, key=lambda obj: position.distance(obj.center))

    def clear(self) -> None:
        """Clear all objects from the database."""
        with self._lock:
            self._pending_objects.clear()
            self._objects.clear()
            self._track_id_map.clear()
            logger.info("ObjectDB cleared")

    def get_stats(self) -> dict[str, int]:
        """Get statistics about the database."""
        with self._lock:
            return {
                "pending_count": len(self._pending_objects),
                "permanent_count": len(self._objects),
                "total_count": len(self._pending_objects) + len(self._objects),
            }

    # ─────────────────────────────────────────────────────────────────
    # Internal Methods
    # ─────────────────────────────────────────────────────────────────

    def _add_object(self, obj: Object) -> Object:
        """Add single object with deduplication logic.

        Priority:
        1. Match by track_id (immediate)
        2. Match by center distance + same name
        3. No match -> create new pending object
        """
        with self._lock:
            # Try to find a matching object
            existing = self._find_matching_object(obj)

            if existing is not None:
                # Update existing object with new detection
                existing.update_object(obj)

                # Update track_id mapping if track_id changed
                if obj.track_id >= 0:
                    self._track_id_map[obj.track_id] = existing.object_id

                # Check if should promote to permanent
                self._check_promotion(existing)

                logger.debug(
                    f"Updated object {existing.object_id} ({existing.name}), "
                    f"detections: {existing.detections_count}"
                )
                return existing
            else:
                # No match - create new pending object
                self._pending_objects[obj.object_id] = obj

                # Track the track_id mapping
                if obj.track_id >= 0:
                    self._track_id_map[obj.track_id] = obj.object_id

                logger.info(f"Created new pending object {obj.object_id} ({obj.name})")
                return obj

    def _find_matching_object(self, obj: Object) -> Object | None:
        """Find existing object that matches the new detection.

        Priority:
        1. Match by track_id (immediate)
        2. Match by center distance + same name
        """
        # Priority 1: Match by track_id
        if obj.track_id >= 0:
            matched = self._match_by_track_id(obj.track_id)
            if matched is not None:
                return matched

        # Priority 2: Match by distance
        return self._match_by_distance(obj)

    def _match_by_track_id(self, track_id: int) -> Object | None:
        """Find object with matching track_id from YOLOE."""
        if track_id < 0:
            return None

        object_id = self._track_id_map.get(track_id)
        if object_id is None:
            return None

        # Check in permanent objects first
        if object_id in self._objects:
            return self._objects[object_id]

        # Check in pending objects
        if object_id in self._pending_objects:
            return self._pending_objects[object_id]

        return None

    def _match_by_distance(self, obj: Object) -> Object | None:
        """Find object within distance threshold (optionally requiring same name)."""
        if obj.center is None:
            return None

        # Combine all objects and filter by name and valid center
        all_objects = list(self._objects.values()) + list(self._pending_objects.values())
        candidates = [
            o
            for o in all_objects
            if o.center is not None
            and ((not self._require_same_name_for_distance_match) or (o.name == obj.name))
            and obj.center.distance(o.center) < self._distance_threshold
        ]

        if not candidates:
            return None

        return min(candidates, key=lambda o: obj.center.distance(o.center))

    def _check_promotion(self, obj: Object) -> None:
        """Move object from pending to permanent if threshold met."""
        if obj.detections_count >= self._min_detections:
            # Check if it's in pending
            if obj.object_id in self._pending_objects:
                # Promote to permanent
                del self._pending_objects[obj.object_id]
                self._objects[obj.object_id] = obj
                logger.info(
                    f"Promoted object {obj.object_id} ({obj.name}) to permanent, "
                    f"total detections: {len(self._objects)}"
                )

    def _prune_stale_pending(self, now_ts: float) -> None:
        """Remove pending objects that haven't been updated recently (TTL)."""
        if self._pending_ttl_seconds <= 0:
            return
        with self._lock:
            stale_ids: list[str] = []
            for object_id, obj in self._pending_objects.items():
                try:
                    age = float(now_ts) - float(obj.ts)
                except Exception:
                    age = 0.0
                if age > self._pending_ttl_seconds:
                    stale_ids.append(object_id)

            if not stale_ids:
                return

            for object_id in stale_ids:
                try:
                    del self._pending_objects[object_id]
                except KeyError:
                    pass

            # Clean up track_id map entries pointing at pruned objects
            stale_id_set = set(stale_ids)
            to_delete = [tid for tid, oid in self._track_id_map.items() if oid in stale_id_set]
            for tid in to_delete:
                del self._track_id_map[tid]

    # ─────────────────────────────────────────────────────────────────
    # Agent encoding
    # ─────────────────────────────────────────────────────────────────

    def agent_encode(self) -> list[dict[str, Any]]:
        """Encode permanent objects for agent consumption."""
        with self._lock:
            return [obj.agent_encode() for obj in self._objects.values()]

    def __len__(self) -> int:
        """Return number of permanent objects."""
        with self._lock:
            return len(self._objects)

    def __repr__(self) -> str:
        with self._lock:
            return f"ObjectDB(permanent={len(self._objects)}, pending={len(self._pending_objects)})"


__all__ = ["ObjectDB"]
