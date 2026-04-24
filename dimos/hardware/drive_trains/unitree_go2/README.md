# Unitree Go2 drive-train adapters

Two adapters live in this folder, serving different control layers of the
same Go2 robot:

| File                                         | Class                          | Layer                                    | Discovery |
|----------------------------------------------|--------------------------------|------------------------------------------|-----------|
| [`adapter.py`](adapter.py)                   | `UnitreeGo2TwistAdapter`       | High-level: Twist (vx, vy, wz) via SportClient + Rage-Mode joystick publisher | Auto — registered under `"unitree_go2"` in the `TwistBaseAdapterRegistry` |
| [`adapter_lowlevel.py`](adapter_lowlevel.py) | `UnitreeGo2LowLevelAdapter`    | Low-level: per-joint `{q, dq, tau, kp, kd}` via `rt/lowcmd` | Manual — instantiated directly, invisible to auto-discovery (filename isn't `adapter.py`) |

Both operate on the same DDS domain and can run in the same process
(pass `assume_dds_initialized=True` to the low-level adapter to skip a
second `ChannelFactoryInitialize`).

---

## Which one should I use?

| Goal                                                  | Use                                 |
|-------------------------------------------------------|-------------------------------------|
| Teleop, navigation, any velocity-commanded behavior   | `UnitreeGo2TwistAdapter` via a blueprint (e.g. `unitree-go2-keyboard-teleop`) |
| Rage Mode (~2.5 m/s forward envelope)                 | Same; set `rage_mode=True` on the adapter (default is off) |
| Custom learned-policy control at the joint level      | `UnitreeGo2LowLevelAdapter` (manual) |
| Direct `LowCmd_` research / replay of recorded torques | `UnitreeGo2LowLevelAdapter` (manual) |

**Do not mix a running `UnitreeGo2TwistAdapter` with a concurrent
`UnitreeGo2LowLevelAdapter`.** The former relies on mcf producing motor
commands; the latter publishes `LowCmd_` that bypasses mcf entirely.
They will fight for the motor rail and the watchdog arbitrations are
undefined. Disconnect one before connecting the other.

---

## Running

Build the CycloneDDS C library via nix (once per machine — creates
`./result` symlink at the repo root, which acts as a GC root):

```bash
nix build nixpkgs#cyclonedds
```

Point your shell / venv at it so `cyclonedds-python` can find the C
library at install and runtime. Easiest: append to `.venv/bin/activate`
so it's set every time you activate the venv:

```bash
NIX_CYCLONEDDS=$(readlink -f ./result)
cat >> .venv/bin/activate <<EOF

# Nix-provided cyclonedds C library
export CYCLONEDDS_HOME=$NIX_CYCLONEDDS
export LD_LIBRARY_PATH="\$CYCLONEDDS_HOME/lib:\${LD_LIBRARY_PATH:-}"
EOF
```

Re-activate the venv (`deactivate && source .venv/bin/activate`) so the
exports take effect, then install the `unitree-dds` extra (pulls
`unitree-sdk2py-dimos` + builds `cyclonedds-python` against the nix lib):

```bash
uv pip install -e ".[unitree-dds]"
```

Alternatives if you don't want to edit the activate script: `export`
both vars in `~/.bashrc`, or use `nix develop` (the flake's shell sets
them automatically), or `direnv` with `.envrc.nix`. See
[`docs/usage/transports/dds.md`](../../../../docs/usage/transports/dds.md).

Set the robot IP and launch a blueprint:

```bash
export ROBOT_IP=192.168.123.161
dimos run unitree-go2-keyboard-teleop         # direct DDS, FreeWalk default
```

Keyboard controls (pygame window must be focused):

| Key     | Action                        |
|---------|-------------------------------|
| `W / S` | Forward / Backward            |
| `Q / E` | Strafe Left / Right           |
| `A / D` | Turn Left / Right             |
| `Shift` | 2× speed boost                |
| `Ctrl`  | 0.5× slow mode                |
| `Space` | Emergency stop                |
| `ESC`   | Quit                          |

Troubleshooting:

| Symptom                               | Fix                                                         |
|---------------------------------------|-------------------------------------------------------------|
| `ModuleNotFoundError: unitree_sdk2py` | `uv pip install -e ".[unitree-dds]"`                        |
| `Could not locate cyclonedds`         | See [`docs/usage/transports/dds.md`](../../../../docs/usage/transports/dds.md) |
| DDS discovery failures                | Verify `ping $ROBOT_IP` succeeds; only one DDS domain active |
| `StandUp()` / `FreeWalk()` fails      | Power-cycle the Go2 on flat ground and retry                |
| Robot ignores velocity commands       | Wait ~5s for `[Go2] Locomotion ready` after startup       |
