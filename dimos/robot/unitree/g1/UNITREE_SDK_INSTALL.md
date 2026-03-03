# Unitree SDK2 Python - Installation Summary

Installation completed successfully on February 14, 2026.

## Installation Locations

### CycloneDDS (DDS Middleware)
- **Source**: `/home/unitree/cyclonedds`
- **Install**: `/home/unitree/cyclonedds/install`
- **Environment Variable**: `CYCLONEDDS_HOME=/home/unitree/cyclonedds/install`

### Unitree SDK2 Python
- **Location**: `/opt/unitree_sdk2_python`
- **Installation Type**: Editable (development mode)
- **Virtual Environment**: `/home/unitree/dimos/.venv`

### Documentation
- **Setup Guide**: `/home/unitree/dimos/docs/usage/robot_setup/unitree_g1.md`

## Quick Start

### Activate Virtual Environment
```bash
cd /home/unitree/dimos
source .venv/bin/activate
```

### Test Import
```bash
python -c "import unitree_sdk2py; print('SDK loaded successfully!')"
```

### Run G1 Examples
```bash
# Replace <interface> with your network interface (e.g., eth0, wlan0)

# G1 7-DOF Arm Control
python /opt/unitree_sdk2_python/example/g1/high_level/g1_arm7_sdk_dds_example.py <interface>

# G1 Locomotion Control
python /opt/unitree_sdk2_python/example/g1/high_level/g1_loco_client_example.py <interface>
```

## Environment Variables (Optional)

Add to `~/.bashrc` for convenience:
```bash
export CYCLONEDDS_HOME="$HOME/cyclonedds/install"
export UNITREE_SDK2_PYTHON="/opt/unitree_sdk2_python"
export PATH="$CYCLONEDDS_HOME/bin:$PATH"
export LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:$LD_LIBRARY_PATH"
```

## Verification Commands

### Check SDK Installation
```bash
source .venv/bin/activate
pip show unitree_sdk2py
```

### Test All Imports
```bash
python -c "
from unitree_sdk2py import g1
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
print('All imports successful!')
"
```

## Dependencies Installed

- **cyclonedds** == 0.10.2 (DDS middleware)
- **numpy** < 2.0, >= 1.26 (numerical computing)
- **opencv-python** (computer vision)

## Network Configuration

- **Robot IP (Ethernet)**: 192.168.123.164
- **Your IP (Ethernet)**: 192.168.123.100
- **SSH**: `ssh unitree@192.168.123.164` (password: 123)

## Available Examples

### G1 High-Level Control
- `/opt/unitree_sdk2_python/example/g1/high_level/g1_arm7_sdk_dds_example.py`
- `/opt/unitree_sdk2_python/example/g1/high_level/g1_arm5_sdk_dds_example.py`
- `/opt/unitree_sdk2_python/example/g1/high_level/g1_arm_action_example.py`
- `/opt/unitree_sdk2_python/example/g1/high_level/g1_loco_client_example.py`

### G1 Low-Level Control
- `/opt/unitree_sdk2_python/example/g1/low_level/` (requires disabling high-level services)

### DDS Communication Tests
- `/opt/unitree_sdk2_python/example/helloworld/publisher.py`
- `/opt/unitree_sdk2_python/example/helloworld/subscriber.py`

## Troubleshooting

If you encounter issues, refer to the comprehensive guide:
```bash
cat /home/unitree/dimos/docs/usage/robot_setup/unitree_g1.md
```

## Resources

- [Unitree Documentation](https://support.unitree.com/home/en/developer)
- [SDK GitHub](https://github.com/unitreerobotics/unitree_sdk2_python)
- [CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds)

---
For detailed setup instructions, see: `/home/unitree/dimos/docs/usage/robot_setup/unitree_g1.md`
