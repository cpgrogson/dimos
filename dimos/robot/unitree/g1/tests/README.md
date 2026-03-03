# G1 Onboard Connection Tests

Test scripts for the G1 native SDK onboard connection.

## Available Tests

### 1. State Diagnostic (`test_state.py`)

Query and display the robot's current FSM state.

```bash
cd /home/unitree/dimos
source .venv/bin/activate
python -m dimos.robot.unitree.g1.tests.test_state
```

Shows:
- Current FSM ID
- FSM Mode
- Balance Mode
- FSM ID reference guide

### 2. Simple Movement Test (`test_move.py`)

Basic test that moves the robot forward briefly.

```bash
python -m dimos.robot.unitree.g1.tests.test_move
```

What it does:
- Initializes connection
- Prompts for confirmation
- Moves forward 0.2 m/s for 1 second
- Stops and cleans up

### 3. Interactive REPL (`test_repl.py`)

Full interactive control interface for manual testing.

```bash
python -m dimos.robot.unitree.g1.tests.test_repl
```

Commands:
- `1` - Stand up
- `2` - Lie down
- `3` - Move forward
- `4` - Move backward
- `5` - Strafe left
- `6` - Strafe right
- `7` - Rotate left
- `8` - Rotate right
- `9` - Emergency stop
- `s` - Show current state
- `q` - Quit

## Quick Start

```bash
cd /home/unitree/dimos
source .venv/bin/activate

# Check robot state first
python -m dimos.robot.unitree.g1.tests.test_state

# Then use interactive control
python -m dimos.robot.unitree.g1.tests.test_repl
```

## Network Configuration

Make sure you're connected to the robot:
- Robot IP: `192.168.123.164` (via Ethernet on `eth0`)
- Your IP should be on same subnet (e.g., `192.168.123.100`)

## Troubleshooting

If commands fail:
1. Check network connection: `ping 192.168.123.164`
2. Verify interface: `ip addr show eth0`
3. Check robot state: `python -m dimos.robot.unitree.g1.tests.test_state`
4. Look at logs - standup commands now show detailed state transitions

## FSM State Reference

```
0   = Zero Torque (robot is limp/unpowered)
1   = Damp (passive damping mode - safe state)
3   = Sit
200 = Start (AI mode active - ready for commands)
702 = Lie2StandUp (stand from lying flat)
706 = Squat2StandUp (toggle between standing/squatting)
```
