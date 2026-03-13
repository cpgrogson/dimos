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

from dataclasses import dataclass, replace
from itertools import islice
import json
import re
import sqlite3
import threading
from typing import TYPE_CHECKING, Any, Generic, TypeVar

from dimos.core.resource import CompositeResource
from dimos.memory2.backend import BackendConfig
from dimos.memory2.blobstore.sqlite import SqliteBlobStore
from dimos.memory2.codecs.base import Codec, codec_for, codec_from_id, codec_id
from dimos.memory2.filter import (
    AfterFilter,
    AtFilter,
    BeforeFilter,
    NearFilter,
    TagsFilter,
    TimeRangeFilter,
    _xyz,
)
from dimos.memory2.livechannel.subject import SubjectChannel
from dimos.memory2.store import Session, Store, StoreConfig
from dimos.memory2.type import _UNLOADED, Observation
from dimos.protocol.service.spec import Configurable

if TYPE_CHECKING:
    from collections.abc import Iterator

    from reactivex.abc import DisposableBase

    from dimos.memory2.backend import Backend, LiveChannel
    from dimos.memory2.buffer import BackpressureBuffer
    from dimos.memory2.filter import Filter, StreamQuery

T = TypeVar("T")

_IDENT_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


# ── Helpers ──────────────────────────────────────────────────────


def _validate_identifier(name: str) -> None:
    if not _IDENT_RE.match(name):
        raise ValueError(f"Invalid stream name: {name!r}")


def _decompose_pose(pose: Any) -> tuple[float, ...] | None:
    if pose is None:
        return None
    if hasattr(pose, "position"):
        pos = pose.position
        orient = getattr(pose, "orientation", None)
        x, y, z = float(pos.x), float(pos.y), float(getattr(pos, "z", 0.0))
        if orient is not None:
            return (x, y, z, float(orient.x), float(orient.y), float(orient.z), float(orient.w))
        return (x, y, z, 0.0, 0.0, 0.0, 1.0)
    if isinstance(pose, (list, tuple)):
        vals = [float(v) for v in pose]
        while len(vals) < 7:
            vals.append(0.0 if len(vals) < 6 else 1.0)
        return tuple(vals[:7])
    return None


def _reconstruct_pose(
    x: float | None,
    y: float | None,
    z: float | None,
    qx: float | None,
    qy: float | None,
    qz: float | None,
    qw: float | None,
) -> tuple[float, ...] | None:
    if x is None:
        return None
    return (x, y or 0.0, z or 0.0, qx or 0.0, qy or 0.0, qz or 0.0, qw or 1.0)


def _compile_filter(f: Filter, stream: str, prefix: str = "") -> tuple[str, list[Any]] | None:
    """Compile a filter to SQL WHERE clause. Returns None for non-pushable filters.

    ``stream`` is the raw stream name (for R*Tree table references).
    ``prefix`` is a column qualifier (e.g. ``"meta."`` for JOIN queries).
    """
    if isinstance(f, AfterFilter):
        return (f"{prefix}ts > ?", [f.t])
    if isinstance(f, BeforeFilter):
        return (f"{prefix}ts < ?", [f.t])
    if isinstance(f, TimeRangeFilter):
        return (f"{prefix}ts >= ? AND {prefix}ts <= ?", [f.t1, f.t2])
    if isinstance(f, AtFilter):
        return (f"ABS({prefix}ts - ?) <= ?", [f.t, f.tolerance])
    if isinstance(f, TagsFilter):
        clauses = []
        params: list[Any] = []
        for k, v in f.tags.items():
            if not _IDENT_RE.match(k):
                raise ValueError(f"Invalid tag key: {k!r}")
            clauses.append(f"json_extract({prefix}tags, '$.{k}') = ?")
            params.append(v)
        return (" AND ".join(clauses), params)
    if isinstance(f, NearFilter):
        pose = f.pose
        if pose is None:
            return None
        if hasattr(pose, "position"):
            pose = pose.position
        cx, cy, cz = _xyz(pose)
        r = f.radius
        # R*Tree bounding-box pre-filter + exact squared-distance check
        rtree_sql = (
            f'{prefix}id IN (SELECT id FROM "{stream}_rtree" '
            f"WHERE x_min >= ? AND x_max <= ? "
            f"AND y_min >= ? AND y_max <= ? "
            f"AND z_min >= ? AND z_max <= ?)"
        )
        dist_sql = (
            f"(({prefix}pose_x - ?) * ({prefix}pose_x - ?) + "
            f"({prefix}pose_y - ?) * ({prefix}pose_y - ?) + "
            f"({prefix}pose_z - ?) * ({prefix}pose_z - ?) <= ?)"
        )
        return (
            f"{rtree_sql} AND {dist_sql}",
            [
                cx - r,
                cx + r,
                cy - r,
                cy + r,
                cz - r,
                cz + r,  # R*Tree bbox
                cx,
                cx,
                cy,
                cy,
                cz,
                cz,
                r * r,  # squared distance
            ],
        )
    # PredicateFilter — not pushable
    return None


def _compile_query(
    query: StreamQuery,
    table: str,
    *,
    join_blob: bool = False,
) -> tuple[str, list[Any], list[Filter]]:
    """Compile a StreamQuery to SQL.

    Returns (sql, params, python_filters) where python_filters must be
    applied as post-filters in Python.
    """
    prefix = "meta." if join_blob else ""
    if join_blob:
        select = f'SELECT meta.id, meta.ts, meta.pose_x, meta.pose_y, meta.pose_z, meta.pose_qx, meta.pose_qy, meta.pose_qz, meta.pose_qw, json(meta.tags), blob.data FROM "{table}" AS meta JOIN "{table}_blob" AS blob ON blob.id = meta.id'
    else:
        select = f'SELECT id, ts, pose_x, pose_y, pose_z, pose_qx, pose_qy, pose_qz, pose_qw, json(tags) FROM "{table}"'

    where_parts: list[str] = []
    params: list[Any] = []
    python_filters: list[Filter] = []

    for f in query.filters:
        compiled = _compile_filter(f, table, prefix)
        if compiled is not None:
            sql_part, sql_params = compiled
            where_parts.append(sql_part)
            params.extend(sql_params)
        else:
            python_filters.append(f)

    sql = select
    if where_parts:
        sql += " WHERE " + " AND ".join(where_parts)

    # ORDER BY
    if query.order_field:
        if not _IDENT_RE.match(query.order_field):
            raise ValueError(f"Invalid order_field: {query.order_field!r}")
        direction = "DESC" if query.order_desc else "ASC"
        sql += f" ORDER BY {prefix}{query.order_field} {direction}"
    else:
        sql += f" ORDER BY {prefix}id ASC"

    # Only push LIMIT/OFFSET to SQL when there are no Python post-filters
    if not python_filters:
        if query.limit_val is not None:
            if query.offset_val:
                sql += f" LIMIT {query.limit_val} OFFSET {query.offset_val}"
            else:
                sql += f" LIMIT {query.limit_val}"
        elif query.offset_val:
            sql += f" LIMIT -1 OFFSET {query.offset_val}"

    return (sql, params, python_filters)


def _compile_count(
    query: StreamQuery,
    table: str,
) -> tuple[str, list[Any], list[Filter]]:
    """Compile a StreamQuery to a COUNT SQL query."""
    where_parts: list[str] = []
    params: list[Any] = []
    python_filters: list[Filter] = []

    for f in query.filters:
        compiled = _compile_filter(f, table)
        if compiled is not None:
            sql_part, sql_params = compiled
            where_parts.append(sql_part)
            params.extend(sql_params)
        else:
            python_filters.append(f)

    sql = f'SELECT COUNT(*) FROM "{table}"'
    if where_parts:
        sql += " WHERE " + " AND ".join(where_parts)

    return (sql, params, python_filters)


# ── SqliteBackend ────────────────────────────────────────────────


class SqliteBackend(Configurable[BackendConfig], CompositeResource, Generic[T]):
    """SQLite-backed observation storage for a single stream (table).

    Owns its ``sqlite3.Connection``.  When disposed (via
    ``CompositeResource.stop()``), the connection is closed.
    """

    default_config: type[BackendConfig] = BackendConfig

    def __init__(self, conn: sqlite3.Connection, name: str, **kwargs: Any) -> None:
        Configurable.__init__(self, **kwargs)
        CompositeResource.__init__(self)
        self._conn = conn
        self._name = name
        self._codec: Codec[Any] = self.config.codec  # type: ignore[assignment]
        self._channel: LiveChannel[T] = self.config.live_channel or SubjectChannel()
        self._lock = threading.Lock()
        self._tag_indexes: set[str] = set()

    @property
    def name(self) -> str:
        return self._name

    @property
    def live_channel(self) -> LiveChannel[T]:
        return self._channel

    @property
    def _join_blobs(self) -> bool:
        if not self.config.eager_blobs:
            return False
        bs = self.config.blob_store
        return isinstance(bs, SqliteBlobStore) and bs._conn is self._conn

    def _make_loader(self, row_id: int) -> Any:
        bs = self.config.blob_store
        if bs is None:
            raise RuntimeError("BlobStore required but not configured")
        name, codec = self._name, self._codec

        def loader() -> Any:
            raw = bs.get(name, row_id)
            return codec.decode(raw)

        return loader

    def _row_to_obs(self, row: tuple[Any, ...], *, has_blob: bool = False) -> Observation[T]:
        if has_blob:
            row_id, ts, px, py, pz, qx, qy, qz, qw, tags_json, blob_data = row
        else:
            row_id, ts, px, py, pz, qx, qy, qz, qw, tags_json = row
            blob_data = None

        pose = _reconstruct_pose(px, py, pz, qx, qy, qz, qw)
        tags = json.loads(tags_json) if tags_json else {}

        if has_blob and blob_data is not None:
            data = self._codec.decode(blob_data)
            return Observation(id=row_id, ts=ts, pose=pose, tags=tags, _data=data)

        return Observation(
            id=row_id,
            ts=ts,
            pose=pose,
            tags=tags,
            _data=_UNLOADED,
            _loader=self._make_loader(row_id),  # type: ignore[arg-type]
        )

    # ── Write ────────────────────────────────────────────────────

    def _ensure_tag_indexes(self, tags: dict[str, Any]) -> None:
        """Auto-create expression indexes for any new tag keys."""
        for key in tags:
            if key not in self._tag_indexes and _IDENT_RE.match(key):
                self._conn.execute(
                    f'CREATE INDEX IF NOT EXISTS "{self._name}_tag_{key}" '
                    f"ON \"{self._name}\"(json_extract(tags, '$.{key}'))"
                )
                self._tag_indexes.add(key)

    def append(self, obs: Observation[T]) -> Observation[T]:
        encoded = self._codec.encode(obs._data)
        pose = _decompose_pose(obs.pose)
        tags_json = json.dumps(obs.tags) if obs.tags else "{}"

        with self._lock:
            if obs.tags:
                self._ensure_tag_indexes(obs.tags)
            if pose:
                px, py, pz, qx, qy, qz, qw = pose
            else:
                px = py = pz = qx = qy = qz = qw = None  # type: ignore[assignment]

            try:
                cur = self._conn.execute(
                    f'INSERT INTO "{self._name}" (ts, pose_x, pose_y, pose_z, pose_qx, pose_qy, pose_qz, pose_qw, tags) '
                    "VALUES (?, ?, ?, ?, ?, ?, ?, ?, jsonb(?))",
                    (obs.ts, px, py, pz, qx, qy, qz, qw, tags_json),
                )
                row_id = cur.lastrowid
                assert row_id is not None

                bs = self.config.blob_store
                if bs is None:
                    raise RuntimeError("BlobStore required but not configured")
                bs.put(self._name, row_id, encoded)

                # R*Tree spatial index
                if pose:
                    self._conn.execute(
                        f'INSERT INTO "{self._name}_rtree" (id, x_min, x_max, y_min, y_max, z_min, z_max) '
                        "VALUES (?, ?, ?, ?, ?, ?, ?)",
                        (row_id, px, px, py, py, pz, pz),
                    )

                vs = self.config.vector_store
                if vs is not None:
                    emb = getattr(obs, "embedding", None)
                    if emb is not None:
                        vs.put(self._name, row_id, emb)

                self._conn.commit()
            except BaseException:
                self._conn.rollback()
                raise

        obs.id = row_id
        self._channel.notify(obs)
        return obs

    # ── Read ─────────────────────────────────────────────────────

    def iterate(self, query: StreamQuery) -> Iterator[Observation[T]]:
        if query.search_vec is not None and query.live_buffer is not None:
            raise TypeError("Cannot combine .search() with .live() — search is a batch operation.")
        buf = query.live_buffer
        if buf is not None:
            sub = self._channel.subscribe(buf)
            return self._iterate_live(query, buf, sub)
        return self._iterate_snapshot(query)

    def _iterate_snapshot(self, query: StreamQuery) -> Iterator[Observation[T]]:
        if query.search_text is not None:
            raise NotImplementedError("search_text is not supported by SqliteBackend")

        if query.search_vec is not None and self.config.vector_store is not None:
            yield from self._vector_search(query)
            return

        join = self._join_blobs
        sql, params, python_filters = _compile_query(query, self._name, join_blob=join)

        cur = self._conn.execute(sql, params)
        cur.arraysize = self.config.page_size
        it: Iterator[Observation[T]] = (self._row_to_obs(r, has_blob=join) for r in cur)

        # Apply Python post-filters
        if python_filters:
            it = (obs for obs in it if all(f.matches(obs) for f in python_filters))

            # Apply LIMIT/OFFSET in Python when we couldn't push to SQL
            if query.offset_val:
                it = islice(it, query.offset_val, None)
            if query.limit_val is not None:
                it = islice(it, query.limit_val)

        yield from it

    def _vector_search(self, query: StreamQuery) -> Iterator[Observation[T]]:
        vs = self.config.vector_store
        assert vs is not None and query.search_vec is not None

        hits = vs.search(self._name, query.search_vec, query.search_k or 10)
        if not hits:
            return

        ids = [h[0] for h in hits]

        # Batch-fetch metadata
        join = self._join_blobs
        placeholders = ",".join("?" * len(ids))
        if join:
            sql = (
                f"SELECT meta.id, meta.ts, meta.pose_x, meta.pose_y, meta.pose_z, "
                f"meta.pose_qx, meta.pose_qy, meta.pose_qz, meta.pose_qw, json(meta.tags), blob.data "
                f'FROM "{self._name}" AS meta '
                f'JOIN "{self._name}_blob" AS blob ON blob.id = meta.id '
                f"WHERE meta.id IN ({placeholders})"
            )
        else:
            sql = (
                f"SELECT id, ts, pose_x, pose_y, pose_z, "
                f"pose_qx, pose_qy, pose_qz, pose_qw, json(tags) "
                f'FROM "{self._name}" WHERE id IN ({placeholders})'
            )

        rows = self._conn.execute(sql, ids).fetchall()
        obs_by_id: dict[int, Observation[T]] = {}
        for r in rows:
            obs = self._row_to_obs(r, has_blob=join)
            obs_by_id[obs.id] = obs

        # Preserve VectorStore ranking order, promoting to EmbeddedObservation
        ranked: list[Observation[T]] = []
        for obs_id, sim in hits:
            match = obs_by_id.get(obs_id)
            if match is not None:
                ranked.append(
                    match.derive(data=match.data, embedding=query.search_vec, similarity=sim)
                )

        # Apply remaining query ops (skip vector search)
        rest = replace(query, search_vec=None, search_k=None)
        yield from rest.apply(iter(ranked))

    def _iterate_live(
        self,
        query: StreamQuery,
        buf: BackpressureBuffer[Observation[T]],
        sub: DisposableBase,
    ) -> Iterator[Observation[T]]:
        from dimos.memory2.buffer import ClosedError

        try:
            # Backfill phase
            last_id = -1
            for obs in self._iterate_snapshot(query):
                last_id = max(last_id, obs.id)
                yield obs

            # Live tail
            filters = query.filters
            while True:
                obs = buf.take()
                if obs.id <= last_id:
                    continue
                last_id = obs.id
                if filters and not all(f.matches(obs) for f in filters):
                    continue
                yield obs
        except (ClosedError, StopIteration):
            pass
        finally:
            sub.dispose()

    def count(self, query: StreamQuery) -> int:
        if query.search_vec:
            return sum(1 for _ in self.iterate(query))

        sql, params, python_filters = _compile_count(query, self._name)
        if python_filters:
            return sum(1 for _ in self.iterate(query))

        row = self._conn.execute(sql, params).fetchone()
        return int(row[0]) if row else 0

    def stop(self) -> None:
        super().stop()
        self._conn.close()


# ── SqliteSession ────────────────────────────────────────────────


class SqliteSession(Session):
    """Session owning a SQLite database.

    Each backend gets its own ``sqlite3.Connection`` so SQLite WAL mode
    handles cross-stream concurrency natively.  A separate
    ``_registry_conn`` is used only for the ``_streams`` registry table.
    """

    def __init__(self, db_path: str, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._db_path = db_path

        # Dedicated connection for the stream registry table
        self._registry_conn = self._open_connection()
        self._registry_conn.execute(
            "CREATE TABLE IF NOT EXISTS _streams ("
            "    name           TEXT PRIMARY KEY,"
            "    payload_module TEXT NOT NULL,"
            "    codec_id       TEXT NOT NULL"
            ")"
        )
        self._registry_conn.commit()

    def _open_connection(self) -> sqlite3.Connection:
        """Open a new WAL-mode connection with sqlite-vec loaded."""
        import sqlite_vec

        conn = sqlite3.connect(self._db_path, check_same_thread=False)
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA synchronous=NORMAL")
        conn.enable_load_extension(True)
        sqlite_vec.load(conn)
        conn.enable_load_extension(False)
        return conn

    @staticmethod
    def _codec_id(c: Codec[Any]) -> str:
        return codec_id(c)

    @staticmethod
    def _codec_from_id(codec_id_str: str, payload_module: str) -> Codec[Any]:
        return codec_from_id(codec_id_str, payload_module)

    def _create_backend(
        self, name: str, payload_type: type[Any] | None = None, **config: Any
    ) -> Backend[Any]:
        _validate_identifier(name)

        # Look up existing stream in registry
        row = self._registry_conn.execute(
            "SELECT payload_module, codec_id FROM _streams WHERE name = ?", (name,)
        ).fetchone()

        if row is not None:
            stored_module, stored_codec_id = row
            if payload_type is not None:
                actual_module = f"{payload_type.__module__}.{payload_type.__qualname__}"
                if actual_module != stored_module:
                    raise ValueError(
                        f"Stream {name!r} was created with type {stored_module}, "
                        f"but opened with {actual_module}"
                    )
            raw_codec = config.get("codec")
            if isinstance(raw_codec, str):
                codec = codec_from_id(raw_codec, stored_module)
            elif raw_codec is not None:
                codec = raw_codec
            else:
                codec = self._codec_from_id(stored_codec_id, stored_module)
        else:
            if payload_type is None:
                raise TypeError(f"Stream {name!r} does not exist yet — payload_type is required")
            payload_module = f"{payload_type.__module__}.{payload_type.__qualname__}"
            raw_codec = config.get("codec")
            if isinstance(raw_codec, str):
                codec = codec_from_id(raw_codec, payload_module)
            elif raw_codec is not None:
                codec = raw_codec
            else:
                codec = codec_for(payload_type)
            self._registry_conn.execute(
                "INSERT INTO _streams (name, payload_module, codec_id) VALUES (?, ?, ?)",
                (name, payload_module, self._codec_id(codec)),
            )
            self._registry_conn.commit()

        # Each backend gets its own connection for WAL-mode concurrency
        backend_conn = self._open_connection()

        # Create metadata table
        backend_conn.execute(
            f'CREATE TABLE IF NOT EXISTS "{name}" ('
            "    id      INTEGER PRIMARY KEY AUTOINCREMENT,"
            "    ts      REAL    NOT NULL UNIQUE,"
            "    pose_x  REAL, pose_y REAL, pose_z REAL,"
            "    pose_qx REAL, pose_qy REAL, pose_qz REAL, pose_qw REAL,"
            "    tags    BLOB    DEFAULT (jsonb('{}'))"
            ")"
        )
        # R*Tree spatial index for pose queries
        backend_conn.execute(
            f'CREATE VIRTUAL TABLE IF NOT EXISTS "{name}_rtree" USING rtree('
            "    id,"
            "    x_min, x_max,"
            "    y_min, y_max,"
            "    z_min, z_max"
            ")"
        )
        backend_conn.commit()

        # Create per-backend stores wrapping the backend's own connection
        if "blob_store" not in config or config["blob_store"] is None:
            config["blob_store"] = SqliteBlobStore(backend_conn)
        if "vector_store" not in config or config["vector_store"] is None:
            from dimos.memory2.vectorstore.sqlite import SqliteVectorStore

            config["vector_store"] = SqliteVectorStore(backend_conn)
        config["codec"] = codec

        backend: SqliteBackend[Any] = SqliteBackend(backend_conn, name, **config)
        self.register_disposables(backend)
        return backend

    def list_streams(self) -> list[str]:
        db_names = {
            row[0] for row in self._registry_conn.execute("SELECT name FROM _streams").fetchall()
        }
        return sorted(db_names | set(self._streams.keys()))

    def delete_stream(self, name: str) -> None:
        self._streams.pop(name, None)
        self._registry_conn.execute(f'DROP TABLE IF EXISTS "{name}"')
        self._registry_conn.execute(f'DROP TABLE IF EXISTS "{name}_blob"')
        self._registry_conn.execute(f'DROP TABLE IF EXISTS "{name}_vec"')
        self._registry_conn.execute(f'DROP TABLE IF EXISTS "{name}_rtree"')
        self._registry_conn.execute("DELETE FROM _streams WHERE name = ?", (name,))
        self._registry_conn.commit()

    def stop(self) -> None:
        super().stop()  # disposes owned backends (closes their connections)
        self._registry_conn.close()


# ── SqliteStore ──────────────────────────────────────────────────


@dataclass
class SqliteStoreConfig(StoreConfig):
    """Config for SQLite-backed store."""

    path: str = "memory.db"


class SqliteStore(Store):
    """Store backed by a SQLite database file."""

    default_config: type[SqliteStoreConfig] = SqliteStoreConfig
    config: SqliteStoreConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

    def session(self, **kwargs: Any) -> SqliteSession:
        return SqliteSession(self.config.path, **kwargs)
