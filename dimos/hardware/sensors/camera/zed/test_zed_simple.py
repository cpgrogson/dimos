#!/usr/bin/env python3
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

"""Simple ZED camera test - run AFTER installing ZED SDK."""

import sys

print("=" * 60)
print("ZED MINI TEST")
print("=" * 60)

# First, check if ZED SDK Python is available
print("\n1. Checking ZED SDK Python bindings...")
try:
    import pyzed.sl as sl

    print("   ✓ pyzed.sl imported successfully")
except ImportError as e:
    print(f"   ✗ Cannot import pyzed.sl: {e}")
    print("\n   To fix:")
    print("   1. Download ZED SDK for Jetson/ARM:")
    print("      https://www.stereolabs.com/developers/release")
    print("   2. Install the .run file:")
    print("      chmod +x ZED_SDK_*.run")
    print("      ./ZED_SDK_*.run -- silent runtime_only")
    print("   3. Add to Python path in your .bashrc:")
    print("      export PYTHONPATH=$PYTHONPATH:/usr/local/zed/lib/python")
    print("   4. Or create symlink in venv:")
    print("      ln -s /usr/local/zed/lib/python/pyzed /path/to/venv/lib/python3.X/site-packages/")
    sys.exit(1)

# List available cameras
print("\n2. Scanning for ZED cameras...")
devices = sl.Camera.get_device_list()

if not devices:
    print("   ✗ No ZED cameras found")
    print("\n   Troubleshooting:")
    print("     - Is the ZED Mini plugged in via USB 3.0?")
    print("     - Try: lsusb | grep -i zed")
    print("     - Check dmesg: dmesg | tail -20")
    sys.exit(1)

print(f"   ✓ Found {len(devices)} ZED camera(s):")
for i, dev in enumerate(devices):
    print(f"\n     Device {i}:")
    print(f"       Serial: {dev.serial_number}")
    print(f"       Model: {dev.camera_model}")
    print(f"       State: {dev.camera_state}")

# Try to open first camera
print("\n3. Testing ZED Mini...")
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.camera_fps = 15
init_params.depth_mode = sl.DEPTH_MODE.NEURAL
init_params.coordinate_units = sl.UNIT.METER

print("   Opening camera...")
err = zed.open(init_params)

if err != sl.ERROR_CODE.SUCCESS:
    print(f"   ✗ Failed to open camera: {err}")
    sys.exit(1)

print("   ✓ Camera opened successfully!")

# Get camera info
info = zed.get_camera_information()
print("\n   Camera Info:")
print(f"     Model: {info.camera_model}")
print(f"     Serial: {info.serial_number}")
print(f"     Firmware: {info.camera_configuration.firmware_version}")
print(
    f"     Resolution: {info.camera_configuration.resolution.width}x{info.camera_configuration.resolution.height}"
)

# Grab a few frames
print("\n4. Capturing test frames...")
runtime_params = sl.RuntimeParameters()
image = sl.Mat()

for i in range(5):
    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image, sl.VIEW.LEFT)
        print(f"   Frame {i + 1}: {image.get_width()}x{image.get_height()}")
    else:
        print(f"   Frame {i + 1}: Failed to grab")

zed.close()

print("\n✓ SUCCESS! ZED Mini is working correctly!")
print("\n" + "=" * 60)
