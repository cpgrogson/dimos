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

"""Download ZED camera calibration file."""

import sys

serial_number = "15054972"

print(f"Downloading calibration for ZED camera S/N: {serial_number}")
print("This requires internet connection...")

# Try using wget to download calibration
url = f"https://calib.stereolabs.com/?SN={serial_number}"
print(f"\nCalibration URL: {url}")
print("\nOption 1: Download via browser and save to:")
print(f"  /usr/local/zed/settings/SN{serial_number}.conf")

print("\nOption 2: Skip calibration and use default (less accurate):")
print("  Set disable_self_calib=True in ZEDCameraConfig")

print("\nOption 3: Run ZED Depth Viewer if available:")
print(f"  ZED_Depth_Viewer --dc {serial_number}")

sys.exit(0)
