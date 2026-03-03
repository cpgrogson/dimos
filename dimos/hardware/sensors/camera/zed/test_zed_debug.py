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

"""ZED Camera Debug Script - Check what's wrong with ZED setup."""

import subprocess
import sys

print("=" * 60)
print("ZED CAMERA DEBUG SCRIPT")
print("=" * 60)

# 1. Check Python version
print(f"\n1. Python Version: {sys.version}")

# 2. Check if pyzed is installed
print("\n2. Checking pyzed installation...")
try:
    import pyzed.sl as sl

    print("   ✓ pyzed is installed")
    print(f"   Version: {sl.__version__ if hasattr(sl, '__version__') else 'unknown'}")
except ImportError as e:
    print("   ✗ pyzed is NOT installed")
    print(f"   Error: {e}")
    print("\n   To install ZED SDK:")
    print("   1. Download ZED SDK from: https://www.stereolabs.com/developers/release")
    print("   2. Choose SDK for your platform (Linux ARM for Jetson/Unitree)")
    print("   3. Install the SDK")
    print("   4. Run: python -m pip install pyzed")

# 3. Check USB devices
print("\n3. Checking USB devices...")
try:
    result = subprocess.run(["lsusb"], capture_output=True, text=True)
    lines = result.stdout.split("\n")
    print(f"   Found {len([l for l in lines if l])} USB devices")

    # Look for ZED-specific devices
    zed_devices = [
        l for l in lines if "ZED" in l.upper() or "STEREO" in l.upper() or "2b03" in l.lower()
    ]
    if zed_devices:
        print("   ✓ Possible ZED camera found:")
        for dev in zed_devices:
            print(f"     {dev}")
    else:
        print("   ⚠ No obvious ZED camera detected")
        print("   All USB devices:")
        for line in lines[:10]:  # Show first 10
            if line:
                print(f"     {line}")
except Exception as e:
    print(f"   Error checking USB: {e}")

# 4. Check video devices
print("\n4. Checking video devices...")
try:
    result = subprocess.run(
        ["ls", "-l", "/dev/video*"], capture_output=True, text=True, shell=False
    )
    devices = result.stdout.split("\n")
    video_devs = [d for d in devices if "video" in d]
    print(f"   Found {len(video_devs)} video devices:")
    for dev in video_devs[:8]:  # Show first 8
        if dev:
            print(f"     {dev}")
except Exception as e:
    print(f"   Error: {e}")

# 5. Try to list ZED cameras (if SDK is installed)
print("\n5. Checking for ZED cameras via SDK...")
try:
    import pyzed.sl as sl

    # Get camera list
    cameras = sl.Camera.get_device_list()
    if cameras:
        print(f"   ✓ Found {len(cameras)} ZED camera(s):")
        for i, cam in enumerate(cameras):
            print(f"     [{i}] Serial: {cam.serial_number}, Model: {cam.camera_model}")
    else:
        print("   ⚠ No ZED cameras detected by SDK")
        print("   Possible issues:")
        print("     - Camera not plugged in")
        print("     - USB cable issue")
        print("     - Insufficient power")
        print("     - Driver issue")
except ImportError:
    print("   ⚠ Cannot check - pyzed not installed")
except Exception as e:
    print(f"   Error: {e}")

# 6. Try to open a ZED camera
print("\n6. Attempting to open ZED camera...")
try:
    import pyzed.sl as sl

    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 15

    err = zed.open(init_params)
    if err == sl.ERROR_CODE.SUCCESS:
        print("   ✓ SUCCESS! ZED camera opened successfully!")

        # Get camera info
        info = zed.get_camera_information()
        print(f"   Model: {info.camera_model}")
        print(f"   Serial: {info.serial_number}")
        print(f"   Firmware: {info.camera_configuration.firmware_version}")
        print(
            f"   Resolution: {info.camera_configuration.resolution.width}x{info.camera_configuration.resolution.height}"
        )

        zed.close()
    else:
        print("   ✗ Failed to open ZED camera")
        print(f"   Error code: {err}")
        print(f"   Error message: {err!s}")

        # Provide specific error guidance
        if err == sl.ERROR_CODE.CAMERA_NOT_DETECTED:
            print("\n   Camera not detected. Check:")
            print("     - Is the camera plugged in via USB 3.0?")
            print("     - Try a different USB port")
            print("     - Check USB cable")
        elif err == sl.ERROR_CODE.INVALID_FUNCTION_PARAMETERS:
            print("\n   Invalid parameters. This shouldn't happen with default settings.")
        elif err == sl.ERROR_CODE.CAMERA_DETECTION_ISSUE:
            print("\n   Camera detection issue. Try:")
            print("     - Unplug and replug the camera")
            print("     - Check dmesg for USB errors: dmesg | tail -20")
except ImportError:
    print("   ⚠ Cannot test - pyzed not installed")
except Exception as e:
    print(f"   Error: {e}")
    import traceback

    traceback.print_exc()

print("\n" + "=" * 60)
print("DIAGNOSIS COMPLETE")
print("=" * 60)
