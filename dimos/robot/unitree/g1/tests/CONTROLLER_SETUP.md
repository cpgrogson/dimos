# G1 Controller Setup for SDK Control

## Critical Requirement

**The G1 robot MUST be activated via the physical controller before SDK commands will work.**

## Problem

The robot boots into "Damp State" by default. In this state:
- Robot is inactive
- AI Sport client is not running
- SDK commands (standup, move, etc.) are **acknowledged but not executed**
- FSM state transitions don't happen

## Solution: Activate the Robot

Use the physical controller to enable Active Mode:

### Steps:
1. **Press L1 + A** on the controller
2. **Then press L1 + UP**
3. Wait for the robot to respond
4. Now SDK commands will work

## Operational Modes

### Active Mode ✓
- **Enable**: L1 + A, then L1 + UP
- **Status**: AI Sport client is running
- **SDK**: Commands work normally
- **Use for**: Normal SDK development and testing

### Debug Mode ✗
- **Enable**: L2 + R2, then L2 + A, then L2 + B
- **Status**: AI Sport client is **disabled**
- **SDK**: High-level commands **DO NOT WORK**
- **Use for**: Low-level motor control only (not supported in Python SDK)
- **To exit**: Reboot the robot

### Damp State (Boot Default) ✗
- **Status**: Robot inactive
- **SDK**: Commands acknowledged but not executed
- **To exit**: Activate via controller (L1 + A, L1 + UP)

## Verification

After activating, check the logs when running SDK code:

```bash
python -m dimos.robot.unitree.g1.tests.test_repl
```

You should see:
```
✓ Motion mode 'ai' selected successfully
```

If you see:
```
✗ Failed to select mode 'ai': code=...
```

The robot is not in Active Mode. Use the controller to activate it first.

## Troubleshooting

### Commands are acknowledged but robot doesn't move
**Symptom**: FSM commands return code 0, but state doesn't change
**Cause**: Robot is in Damp state or Debug mode
**Solution**: Activate via controller (L1 + A, then L1 + UP)

### "Failed to select mode" error
**Symptom**: SelectMode returns non-zero code
**Cause**: AI Sport client is not running
**Solution**:
1. Check if robot is in Debug mode - if so, reboot
2. If in Damp state, activate via controller

### Robot was working but suddenly stops responding
**Symptom**: Commands stopped working after using controller
**Cause**: Accidentally entered Debug mode
**Solution**: Reboot the robot, then activate properly

## Important Notes

- **EDU version required**: Only G1 EDU supports SDK control
- **Firmware**: Requires v1.3.0 or higher
- **No programmatic activation**: You cannot activate the robot purely via SDK - controller is required
- **Debug mode caveat**: Once in Debug mode, you must reboot to restore SDK control

## Quick Reference

```
Active Mode (SDK works):
  L1 + A  →  L1 + UP  →  SDK commands work

Debug Mode (SDK broken):
  L2 + R2  →  L2 + A  →  L2 + B  →  Reboot to fix
```

## Source

Based on [unitree_sdk2_python Issue #43](https://github.com/unitreerobotics/unitree_sdk2_python/issues/43)
