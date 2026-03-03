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

"""
Quick test script to move G1 forward using onboard SDK connection.
"""

import time

from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.robot.unitree.g1.onboard_connection import G1OnboardConnection


def test_move_forward():
    print("Initializing G1 onboard connection...")
    conn = G1OnboardConnection(network_interface="eth0", mode="ai")

    try:
        print("Starting connection...")
        conn.start()
        time.sleep(1)

        print("\n⚠️  Make sure robot is standing and area is clear!")
        input("Press Enter to continue or Ctrl+C to abort...")

        # Create a small forward velocity command
        twist = Twist(
            linear=Vector3(x=0.2, y=0.0, z=0.0),  # 0.2 m/s forward
            angular=Vector3(x=0.0, y=0.0, z=0.0),  # no rotation
        )

        print("\n🤖 Moving forward for 1 second...")
        success = conn.move(twist, duration=1.0)

        if success:
            print("✓ Movement command sent successfully")
            time.sleep(1.5)  # Wait for movement to complete
        else:
            print("✗ Movement command failed")

        print("🛑 Stopping robot...")
        conn.stop()
        time.sleep(0.5)

        print("✓ Test complete!")

    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted! Stopping robot...")
        conn.stop()
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback

        traceback.print_exc()
        conn.stop()
    finally:
        print("Cleaning up...")
        conn.disconnect()


if __name__ == "__main__":
    print("=" * 60)
    print("G1 Forward Movement Test")
    print("=" * 60)
    test_move_forward()
