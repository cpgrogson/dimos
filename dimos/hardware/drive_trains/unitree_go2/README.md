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
| Rage Mode (~2.5 m/s forward envelope)                 | Same; set `rage_mode=True` (default) on the adapter |
| Custom learned-policy control at the joint level      | `UnitreeGo2LowLevelAdapter` (manual) |
| Direct `LowCmd_` research / replay of recorded torques | `UnitreeGo2LowLevelAdapter` (manual) |
| Drive from a laptop not on the Go2's DDS LAN          | Use the **WebRTC** path instead: `GO2Connection` + `unitree-go2-webrtc-rage-keyboard-teleop` blueprint (see [robot/unitree/connection.py](../../../robot/unitree/connection.py)) |

**Do not mix a running `UnitreeGo2TwistAdapter` with a concurrent
`UnitreeGo2LowLevelAdapter`.** The former relies on mcf producing motor
commands; the latter publishes `LowCmd_` that bypasses mcf entirely.
They will fight for the motor rail and the watchdog arbitrations are
undefined. Disconnect one before connecting the other.

---

## Firmware assumptions

Everything in this folder was developed against Go2 firmware where
`mcf_main` is the only running sport controller — `legged_sport`,
`ai_sport`, and `advanced_sport` are installed but dormant. On this
firmware:

- `MotionSwitcher.SelectMode("normal"/"ai"/"advanced")` returns
  7002/7004. Only `"mcf"` activates. We therefore **do not attempt
  mode switching at runtime** — `_verify_sport_mode_active` just accepts
  whatever `CheckMode` reports.
- `AiController::Move` (api_id 1008) doesn't dispatch to `FsmRageMode`.
  Rage's velocity input comes via the wireless-controller joystick
  buffer on `rt/wirelesscontroller_unprocessed` — see the next section.

Full reverse-engineering trail: [`data/notes/go2_firmware_modes.md`](../../../../data/notes/go2_firmware_modes.md).

If you're on a firmware that exposes more than one mode, the earlier
multi-candidate mode-switching logic is gone from `adapter.py`. Recover
it from git history if you need it (`git log -- adapter.py` will show
the `switch_mode` / `stand_down_and_release` / `_SPORT_MODE_CANDIDATES`
era).

---

## `adapter.py` internals

### TwistBase contract

Implements [`TwistBaseAdapter`](../spec.py) with 3 DOF: `[vx, vy, wz]`
mapped to virtual joints `go2/vx`, `go2/vy`, `go2/wz`. `ControlCoordinator`
with `adapter_type="unitree_go2"` wraps it as a `ConnectedTwistBase` and
drives it from the tick loop.

### Session pattern

Connection state lives in a `_Session` dataclass. Two locks:

- `_session_lock` guards the `self._session` reference (creation / tear-down).
- `session.lock` serializes SportClient RPCs and guards `latest_state`
  (mutated by the DDS callback thread).

**Rule:** never take `_session_lock` while holding `session.lock`. The
DDS callback already holds `session.lock` briefly during state updates;
taking the outer lock from inside the inner would deadlock with any
disconnect path.

### Boot sequence (`connect()`)

```
ChannelFactoryInitialize(0)
  → MotionSwitcherClient.Init + 1.5 s settle
  → ChannelSubscriber("rt/sportmodestate") for telemetry
  → _verify_sport_mode_active()    # just CheckMode, accept any active mode
  → SportClient.Init + 2.0 s settle
  → _initialize_locomotion()  # StandUp → FreeWalk → SpeedLevel(1)
  → set_rage_mode(True) if configured    # optional
  → print_status()
```

### Rage Mode path

`set_rage_mode(True)` enables FsmRageMode and spawns a background thread
publishing `WirelessController_` messages on
`rt/wirelesscontroller_unprocessed` at 100 Hz. The FSM reads its
velocity input from that topic, not from `SportClient.Move`.

Write path once Rage is on:

```
keyboard_teleop publishes Twist on /cmd_vel
  → ControlCoordinator → adapter.write_velocities(vx, vy, wz)
  → session.rage_cmd = (vx, vy, wz)     # cache only
  → 100 Hz thread maps (vx, vy, wz) → (ly, lx, rx), clips [-1, 1]
  → publish WirelessController_ → rt/wirelesscontroller_unprocessed
  → mcf → FsmRageMode policy → rt/lowcmd → motors
```

### Tunable constants

All in the class body as `_RAGE_*` constants:

| Constant           | Default | Purpose                                                       |
|--------------------|---------|---------------------------------------------------------------|
| `_RAGE_UP_VX`      | 2.5     | m/s — forward envelope (from `rage_mode_export_cfg.json`)     |
| `_RAGE_UP_VY`      | 1.0     | m/s — lateral envelope                                        |
| `_RAGE_UP_VYAW`    | 5.0     | rad/s — yaw envelope                                          |
| `_RAGE_PUBLISH_HZ` | 100.0   | publisher rate; raise to 200 if app or RC out-publishes us    |
| `_RAGE_LY_SIGN`    | +1.0    | forward stick axis sign (+1 on our firmware)                  |
| `_RAGE_LX_SIGN`    | −1.0    | lateral stick axis sign (ROS +y = left → Unitree −lx)         |
| `_RAGE_RX_SIGN`    | −1.0    | yaw stick axis sign (ROS +z = CCW → Unitree −rx)              |

Flip a sign if a key drives the robot the wrong way on a different
firmware build.

### Public surface

- Lifecycle: `connect`, `disconnect`, `is_connected`
- Tick-loop protocol: `read_velocities`, `read_odometry`,
  `write_velocities`, `write_stop`, `write_enable`, `read_enabled`,
  `get_dof`
- Diagnostics: `check_mode`, `get_sport_state`, `get_status`, `print_status`
- Tuning: `set_speed_level`, `set_rage_mode`

---

## `adapter_lowlevel.py` internals

Standalone — caller instantiates it directly, not via the registry.

### Preconditions (caller's responsibility)

1. Robot is sat / damped before `connect()` is called.
2. `MotionSwitcher.CheckMode` returns an empty name (no sport
   controller active). On our firmware mcf is usually active on
   boot; since mode-switching is no longer exposed in `adapter.py`,
   you'll need to release mcf via another tool (physical RC, Unitree
   app, or a git-restored `stand_down_and_release`) before the
   low-level adapter will accept the `connect()`.

### Watchdog

`rt/lowcmd` has no built-in watchdog — a dead publisher leaves the
last torque latched, which is unsafe. This adapter runs a 100 Hz
background thread that auto-damps (`kp=0, kd=1`) if `flush()` is
silent for more than `_WATCHDOG_TIMEOUT_S` (0.2 s). `emergency_damp()`
itself does NOT update the flush timestamp, so the watchdog keeps
firing as long as the user stays silent.

### Joint layout

Canonical Unitree Go2 order, exposed as `GO2_JOINT_INDEX`:

```
FR hip=0,  thigh=1,  calf=2
FL hip=3,  thigh=4,  calf=5
RR hip=6,  thigh=7,  calf=8
RL hip=9,  thigh=10, calf=11
```

`motor_cmd[]` has 20 slots; slots 12..19 are always written disabled
with zero gains.

### API

- `connect` / `disconnect` / `is_connected`
- Staging: `write_joint_cmd(idx, q, dq, tau, kp, kd)` — caches one
  joint, does NOT publish.
- Batch + publish: `write_joint_array(qs, dqs, taus, kps, kds)` — all
  12 joints at once, auto-flushes, resets the watchdog.
- Manual publish: `flush()` — CRCs and publishes the staged buffer.
- Readback: `read_joint_state()` → dict with motors / imu / foot_force.
- Safety: `emergency_damp()` publishes a damping LowCmd on all joints.

---

## Guidelines for modifications

1. **Never break the tick-loop contract.** `read_velocities`,
   `write_velocities`, etc. are called at 100 Hz by `ControlCoordinator`.
   Don't add logging inside them without a rate limit, don't add
   blocking calls, don't take `_session_lock` on the hot path.

2. **Don't re-introduce mode switching without evidence it works.**
   Every `SelectMode` call on our firmware is a 4 s wait for a 7002
   rejection. If you need it for a different firmware variant, gate
   it on a detected capability, don't blanket-add it.

3. **Rage state must never hold the SportClient session lock during
   publishes.** The joystick thread reads `session.rage_cmd` without
   a lock by design (atomic tuple assignment in Python). If you need
   to change that, add a separate lock — don't reuse `session.lock`
   because the publish rate is 100 Hz and would starve SportClient RPCs.

4. **For any new AI-controller api_id (beyond Rage 2059)**, use
   `_call_sport_api(api_id, payload)` — it handles `_RegistApi` for
   you, which the public `SportClient._Call` does not. Without
   registering, `_Call` returns 3103 (`RPC_ERR_CLIENT_API_NOT_REG`)
   before the RPC leaves the process.

5. **Sign conventions.** Always go through `_RAGE_LY_SIGN` etc. —
   don't hardcode flips at the callsite. One firmware-variant flip
   is a constant change; N hardcoded flips is a bug hunt.

6. **When the Unitree app is connected, expect competition.** Both
   `sbus_handle` (physical RC) and `unitreeWebRTCClientMaster` (app)
   publish to `rt/wirelesscontroller_unprocessed`. DDS default QoS
   is last-write-wins. Either close the app, keep RC sticks
   centered, or out-rate the competition by raising `_RAGE_PUBLISH_HZ`.

7. **Low-level adapter expects no mode to be active.** Don't connect
   it while mcf is running — the motors will fight. The adapter
   checks `MotionSwitcher.CheckMode` and refuses to connect if a
   controller is loaded.

---

## Related files

- [`data/notes/go2_firmware_modes.md`](../../../../data/notes/go2_firmware_modes.md) —
  full reverse-engineering trail: mcf architecture, Rage Mode
  discovery, api_id extraction methodology, signs verification.
- [`dimos/robot/unitree/connection.py`](../../../robot/unitree/connection.py) —
  WebRTC-based `UnitreeWebRTCConnection`. Same Rage path, different
  transport (speaks the mobile app's protocol).
- [`dimos/robot/unitree/go2/blueprints/basic/`](../../../robot/unitree/go2/blueprints/basic/) —
  user-facing blueprints that wire these adapters into coordinators:
  - `unitree_go2_keyboard_teleop.py` — direct DDS path.
  - `unitree_go2_webrtc_rage_keyboard_teleop.py` — WebRTC + Rage.
- [`data/notes/velocity_recorder*.py`](../../../../data/notes/) +
  `velocity_plot*.py` — recording tools for tuning the velocity path.
