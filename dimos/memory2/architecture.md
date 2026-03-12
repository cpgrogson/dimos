# memory2

Observation storage and streaming layer for DimOS. Pull-based, lazy, composable.

## Architecture

```
             Live Sensor Data
                    ↓
Store → Session → Stream → [filters / transforms / terminals] → Stream  → [filters / transforms / terminals] → Stream → Live hooks
                    ↓                                              ↓                                             ↓
                 Backend (ListBackend, SqliteBackend)           Backend                                      In Memory
```

**Store** owns a storage location (file, in-memory). **Session** manages named streams over a shared connection. **Stream** is the query/iteration surface — lazy until a terminal is called.


Supporting Systems:

- BlobStore — separates large payloads from metadata. FileBlobStore (files on disk) and SqliteBlobStore (blob table per stream). Supports lazy loading.
- Codecs — codec_for() auto-selects: JpegCodec for images (TurboJPEG, ~10-20x compression), LcmCodec for DimOS messages, PickleCodec fallback.
- Transformers — Transformer[T,R] ABC wrapping iterator-to-iterator. EmbedImages/EmbedText enrich observations with embeddings. QualityWindow keeps best per time window.
- Backpressure Buffers — KeepLast, Bounded, DropNew, Unbounded — bridge push/pull for live mode.


## Modules

| Module         | What                                                              |
|----------------|-------------------------------------------------------------------|
| `stream.py`    | Stream node — filters, transforms, terminals                      |
| `backend.py`   | Backend protocol, LiveChannel / VectorStore / BlobStore ABCs     |
| `filter.py`    | StreamQuery dataclass, filter types, Python query execution       |
| `transform.py` | Transformer ABC, FnTransformer, FnIterTransformer, QualityWindow  |
| `buffer.py`    | Backpressure buffers for live mode (KeepLast, Bounded, Unbounded) |
| `store.py`     | Store / Session (Configurable), StoreConfig / SessionConfig      |
| `type.py`      | Observation, EmbeddedObservation dataclasses                      |
| `embed.py`     | EmbedImages / EmbedText transformers                              |

## Subpackages

| Package      | What                                                 | Docs                                             |
|--------------|------------------------------------------------------|--------------------------------------------------|
| `impl/`         | Backend implementations (ListBackend, SqliteBackend) | [impl/README.md](impl/README.md)                 |
| `livechannel/`  | Live notification channels (SubjectChannel)          |                                                  |
| `blobstore/`    | Pluggable blob storage (file, sqlite)                | [blobstore/blobstore.md](blobstore/blobstore.md) |
| `codecs/`       | Encode/decode for storage (pickle, JPEG, LCM)        | [codecs/README.md](codecs/README.md)             |

## Docs

| Doc | What |
|-----|------|
| [streaming.md](streaming.md) | Lazy vs materializing vs terminal — evaluation model, live safety |
| [embeddings.md](embeddings.md) | Embedding layer design — EmbeddedObservation, vector search, EmbedImages/EmbedText |
| [blobstore/blobstore.md](blobstore/blobstore.md) | BlobStore architecture — separate payload storage from metadata |

## Query execution

`StreamQuery` holds the full query spec (filters, text search, vector search, ordering, offset/limit). It also provides `apply(iterator)` — a Python-side execution path that runs all operations as in-memory predicates, brute-force cosine, and list sorts.

This is the **default fallback**. Backends are free to push down operations using store-specific strategies instead:

| Operation      | Python fallback (`StreamQuery.apply`) | Store push-down (example)        |
|----------------|---------------------------------------|----------------------------------|
| Filters        | `filter.matches()` predicates         | SQL WHERE clauses                |
| Text search    | Case-insensitive substring            | FTS5 full-text index             |
| Vector search  | Brute-force cosine similarity         | vec0 / FAISS ANN index           |
| Ordering       | `sorted()` materialization            | SQL ORDER BY                     |
| Offset / limit | `islice()`                            | SQL OFFSET / LIMIT               |

`ListBackend` delegates entirely to `StreamQuery.apply()`. `SqliteBackend` translates the query into SQL and only falls back to Python for operations it can't express natively.

Transform-sourced streams (post `.transform()`) always use `StreamQuery.apply()` since there's no backend to push down to.

## Quick start

```python
from dimos.memory2 import MemoryStore

store = MemoryStore()
with store.session() as session:
    images = session.stream("images")

    # Write
    images.append(frame, ts=time.time(), pose=(x, y, z), tags={"camera": "front"})

    # Query
    recent = images.after(t).limit(10).fetch()
    nearest = images.near(pose, radius=2.0).fetch()
    latest = images.last()

    # Transform (class or bare generator function)
    edges = images.transform(Canny()).save(session.stream("edges"))

    def running_avg(upstream):
        total, n = 0.0, 0
        for obs in upstream:
            total += obs.data; n += 1
            yield obs.derive(data=total / n)
    avgs = stream.transform(running_avg).fetch()

    # Live
    for obs in images.live().transform(process):
        handle(obs)

    # Embed + search
    images.transform(EmbedImages(clip)).save(session.stream("embedded"))
    results = session.stream("embedded").search(query_vec, k=5).fetch()
```

## Implementations

| Backend         | Status   | Storage                                |
|-----------------|----------|----------------------------------------|
| `ListBackend`   | Complete | In-memory (lists + brute-force search) |
| `SqliteBackend` | Stub     | SQLite (WAL, FTS5, vec0)               |
