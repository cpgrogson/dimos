# PR #1643 (rconnect) — Paul Review Fixes

## Commits (local, not pushed)

### 1. `81769d273` — Log exception + unblock stop() on startup failure
- If `_serve()` throws, `_server_ready` was never set → `stop()` blocked 5s
- Now logs exception and sets `_server_ready` in finally
- **Revert:** `git revert 81769d273`

## Reviewer was wrong on
- `_server_ready` race — it IS set inside `async with` (after bind), not before
- `msg.get("x") or 0` — code already uses `msg.get("x", 0)` correctly

## Not addressed (need Jeff's input)
- `vis_module` always bundling `RerunWebSocketServer` — opt-out design choice
- `LCM()` instantiated for non-rerun backends — wasted resource
- `rerun-connect` skipping `WebsocketVisModule` — intentional?
- Default `host = "0.0.0.0"` — intentional for remote viewer use case
- Hardcoded test ports — should use port=0 for parallel safety
