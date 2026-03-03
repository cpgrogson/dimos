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

"""Final ZED camera test - use this in your code."""

import numpy as np
import pyzed.sl as sl


def test_zed():
    print("Opening ZED Mini...")

    zed = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = 15
    init.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # or ULTRA, QUALITY
    init.coordinate_units = sl.UNIT.METER

    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open camera: {err}")
        return False

    print(f"✓ Camera opened: {zed.get_camera_information().camera_model}")

    # Capture frames
    runtime = sl.RuntimeParameters()
    image = sl.Mat()
    depth = sl.Mat()

    print("\nCapturing 10 frames...")
    for i in range(10):
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            # Get RGB image
            zed.retrieve_image(image, sl.VIEW.LEFT)
            img_data = image.get_data()[:, :, :3]  # RGB

            # Get depth map
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            depth_data = depth.get_data()

            # Filter out invalid depth values
            valid_depth = depth_data[~np.isnan(depth_data) & ~np.isinf(depth_data)]
            if len(valid_depth) > 0:
                print(
                    f"  Frame {i + 1}: RGB {img_data.shape}, Depth range: {valid_depth.min():.2f}-{valid_depth.max():.2f}m"
                )
            else:
                print(f"  Frame {i + 1}: RGB {img_data.shape}, Depth: initializing...")
        else:
            print(f"  Frame {i + 1}: Failed to grab")

    zed.close()
    print("\n✓✓✓ ZED Mini test complete!")
    return True


if __name__ == "__main__":
    test_zed()
