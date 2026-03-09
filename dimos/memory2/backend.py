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

import threading
import time
from typing import TYPE_CHECKING, Any, Generic, Protocol, TypeVar, runtime_checkable

from reactivex.disposable import Disposable

from dimos.memory2.type import Observation

if TYPE_CHECKING:
    from collections.abc import Iterator

    from reactivex.abc import DisposableBase

    from dimos.memory2.buffer import BackpressureBuffer
    from dimos.memory2.filter import StreamQuery

T = TypeVar("T")


@runtime_checkable
class Backend(Protocol[T]):
    """Data source protocol for stored observations.

    The backend is fully responsible for applying query filters.
    How it does so (SQL, R-tree, Python predicates) is its business.
    """

    @property
    def name(self) -> str: ...

    def iterate(self, query: StreamQuery) -> Iterator[Observation[T]]: ...

    def append(
        self,
        payload: T,
        *,
        ts: float | None = None,
        pose: Any | None = None,
        tags: dict[str, Any] | None = None,
    ) -> Observation[T]: ...

    def count(self, query: StreamQuery) -> int: ...

    def subscribe(self, buf: BackpressureBuffer[Observation[T]]) -> DisposableBase: ...


class ListBackend(Generic[T]):
    """In-memory backend for experimentation. Thread-safe."""

    def __init__(self, name: str = "<memory>") -> None:
        self._name = name
        self._observations: list[Observation[T]] = []
        self._next_id = 0
        self._lock = threading.Lock()
        self._subscribers: list[BackpressureBuffer[Observation[T]]] = []

    @property
    def name(self) -> str:
        return self._name

    def append(
        self,
        payload: T,
        *,
        ts: float | None = None,
        pose: Any | None = None,
        tags: dict[str, Any] | None = None,
    ) -> Observation[T]:
        with self._lock:
            obs: Observation[T] = Observation(
                id=self._next_id,
                ts=ts if ts is not None else time.time(),
                pose=pose,
                tags=tags or {},
                _data=payload,
            )
            self._next_id += 1
            self._observations.append(obs)
            subs = list(self._subscribers)

        # Notify outside lock to avoid deadlocks
        for buf in subs:
            buf.put(obs)

        return obs

    def iterate(self, query: StreamQuery) -> Iterator[Observation[T]]:
        """Snapshot + apply all filters/ordering/offset/limit in Python.

        If query.live_buffer is set, subscribes before backfill, then
        switches to a live tail that blocks for new observations.
        """
        buf = query.live_buffer
        if buf is not None:
            # Subscribe BEFORE backfill to avoid missing items
            sub = self.subscribe(buf)
            return self._iterate_live(query, buf, sub)
        return self._iterate_snapshot(query)

    def _iterate_snapshot(self, query: StreamQuery) -> Iterator[Observation[T]]:
        with self._lock:
            snapshot = list(self._observations)

        # Apply filters
        for f in query.filters:
            snapshot = [obs for obs in snapshot if f.matches(obs)]

        # Ordering
        if query.order_field:
            key = query.order_field
            snapshot.sort(
                key=lambda obs: getattr(obs, key) if getattr(obs, key, None) is not None else 0,
                reverse=query.order_desc,
            )

        # Offset
        if query.offset_val:
            snapshot = snapshot[query.offset_val :]

        # Limit
        if query.limit_val is not None:
            snapshot = snapshot[: query.limit_val]

        yield from snapshot

    def _iterate_live(
        self,
        query: StreamQuery,
        buf: BackpressureBuffer[Observation[T]],
        sub: DisposableBase,
    ) -> Iterator[Observation[T]]:
        from dimos.memory2.buffer import ClosedError

        # Backfill phase — use snapshot query (without live) for the backfill
        last_id = -1
        for obs in self._iterate_snapshot(query):
            last_id = max(last_id, obs.id)
            yield obs

        # Live tail
        filters = query.filters
        try:
            while True:
                obs = buf.take()
                if obs.id <= last_id:
                    continue
                last_id = obs.id
                if filters and not all(f.matches(obs) for f in filters):
                    continue
                yield obs
        except (ClosedError, StopIteration):
            sub.dispose()

    def count(self, query: StreamQuery) -> int:
        return sum(1 for _ in self.iterate(query))

    def subscribe(self, buf: BackpressureBuffer[Observation[T]]) -> DisposableBase:
        with self._lock:
            self._subscribers.append(buf)

        def _unsubscribe() -> None:
            with self._lock:
                try:
                    self._subscribers.remove(buf)
                except ValueError:
                    pass

        return Disposable(action=_unsubscribe)
