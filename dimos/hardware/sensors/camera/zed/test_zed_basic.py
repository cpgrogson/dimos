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

"""Basic ZED test without NEURAL depth mode."""

import pyzed.sl as sl

print("Testing ZED with basic depth mode (no AI models needed)...")

zed = sl.Camera()
init = sl.InitParameters()
init.camera_resolution = sl.RESOLUTION.HD720
init.camera_fps = 15
init.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE instead of NEURAL
init.coordinate_units = sl.UNIT.METER

err = zed.open(init)

if err != sl.ERROR_CODE.SUCCESS:
    print(f"✗ Failed: {err}")
    exit(1)

print("✓✓✓ ZED camera opened successfully!")

info = zed.get_camera_information()
print("\nCamera Info:")
print(f"  Model: {info.camera_model}")
print(f"  Serial: {info.serial_number}")
print(f"  Firmware: {info.camera_configuration.firmware_version}")

# Grab a few frames
runtime = sl.RuntimeParameters()
image = sl.Mat()
depth = sl.Mat()

print("\nCapturing frames...")
for i in range(5):
    if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image, sl.VIEW.LEFT)
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        print(
            f"  Frame {i + 1}: {image.get_width()}x{image.get_height()}, depth range: {depth.get_data().min():.2f}-{depth.get_data().max():.2f}m"
        )

zed.close()
print("\n✓✓✓ ZED Mini is fully working!")
