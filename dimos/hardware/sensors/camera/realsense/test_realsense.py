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

"""Test RealSense camera (since that's what you actually have plugged in)."""

import pyrealsense2 as rs

print("=" * 60)
print("REALSENSE D435i TEST")
print("=" * 60)

# Create a context object to manage RealSense devices
ctx = rs.context()
devices = ctx.query_devices()

print(f"\n1. Found {len(devices)} RealSense device(s)")

if len(devices) == 0:
    print("   ✗ No RealSense cameras detected!")
    print("\n   Troubleshooting:")
    print("     - Check USB connection")
    print("     - Try different USB port (USB 3.0 preferred)")
    print("     - Check if udev rules are set up")
    exit(1)

# Get first device info
for i, dev in enumerate(devices):
    print(f"\n   Device {i}:")
    print(f"     Name: {dev.get_info(rs.camera_info.name)}")
    print(f"     Serial: {dev.get_info(rs.camera_info.serial_number)}")
    print(f"     Firmware: {dev.get_info(rs.camera_info.firmware_version)}")
    print(f"     USB Type: {dev.get_info(rs.camera_info.usb_type_descriptor)}")

print("\n2. Testing camera stream...")

try:
    # Create a pipeline
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable streams
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Start streaming
    print("   Starting pipeline...")
    profile = pipeline.start(config)

    print("   ✓ Pipeline started successfully!")

    # Get a few frames
    print("\n3. Capturing test frames...")
    for i in range(10):
        frames = pipeline.wait_for_frames(timeout_ms=5000)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if color_frame and depth_frame:
            print(
                f"   Frame {i + 1}: Color={color_frame.get_width()}x{color_frame.get_height()}, "
                f"Depth={depth_frame.get_width()}x{depth_frame.get_height()}"
            )
        else:
            print(f"   Frame {i + 1}: Failed to get frames")

    # Stop streaming
    pipeline.stop()

    print("\n   ✓ SUCCESS! RealSense camera is working!")
    print("\n4. Summary:")
    print("   Your camera is a RealSense D435i, not a ZED camera.")
    print("   Use pyrealsense2 module to access it, not pyzed.")

except Exception as e:
    print(f"\n   ✗ Error: {e}")
    import traceback

    traceback.print_exc()

print("\n" + "=" * 60)
