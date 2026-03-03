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
Interactive REPL for testing G1 onboard connection.
Use this to manually test robot commands.
"""

import time

from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.robot.unitree.g1.onboard_connection import G1OnboardConnection


def print_menu():
    print("\n" + "=" * 60)
    print("G1 Robot Control REPL")
    print("=" * 60)
    print("Commands:")
    print("  1 - Stand up")
    print("  2 - Lie down")
    print("  3 - Move forward (0.2 m/s for 1s)")
    print("  4 - Move backward (0.2 m/s for 1s)")
    print("  5 - Strafe left (0.2 m/s for 1s)")
    print("  6 - Strafe right (0.2 m/s for 1s)")
    print("  7 - Rotate left (0.3 rad/s for 1s)")
    print("  8 - Rotate right (0.3 rad/s for 1s)")
    print("  9 - Stop")
    print("  s - Show robot state")
    print("  q - Quit")
    print("=" * 60)


def main():
    print("\n🤖 Initializing G1 onboard connection...")
    conn = G1OnboardConnection(network_interface="eth0", mode="ai")

    try:
        conn.start()
        print("✓ Connection started")
        time.sleep(1)

        print("\n⚠️  WARNING: Ensure area is clear around robot!")
        input("Press Enter to continue...")

        # Helper function to show FSM state
        def show_state():
            state = conn.get_state()
            print(f"  Current state: {state}")
            return state

        while True:
            print_menu()
            cmd = input("\nEnter command: ").strip().lower()

            if cmd == "q":
                print("Exiting...")
                break

            elif cmd == "1":
                print("🤖 Standing up...")
                success = conn.standup()
                print(f"  {'✓' if success else '✗'} Standup completed")

            elif cmd == "2":
                print("🤖 Lying down...")
                success = conn.liedown()
                print(f"  {'✓' if success else '✗'} Liedown completed")

            elif cmd == "3":
                print("🤖 Moving forward...")
                twist = Twist(linear=Vector3(0.2, 0, 0), angular=Vector3(0, 0, 0))
                success = conn.move(twist, duration=1.0)
                print(f"  {'✓' if success else '✗'} Move command sent")
                time.sleep(1.5)

            elif cmd == "4":
                print("🤖 Moving backward...")
                twist = Twist(linear=Vector3(-0.2, 0, 0), angular=Vector3(0, 0, 0))
                success = conn.move(twist, duration=1.0)
                print(f"  {'✓' if success else '✗'} Move command sent")
                time.sleep(1.5)

            elif cmd == "5":
                print("🤖 Strafing left...")
                twist = Twist(linear=Vector3(0, 0.2, 0), angular=Vector3(0, 0, 0))
                success = conn.move(twist, duration=1.0)
                print(f"  {'✓' if success else '✗'} Move command sent")
                time.sleep(1.5)

            elif cmd == "6":
                print("🤖 Strafing right...")
                twist = Twist(linear=Vector3(0, -0.2, 0), angular=Vector3(0, 0, 0))
                success = conn.move(twist, duration=1.0)
                print(f"  {'✓' if success else '✗'} Move command sent")
                time.sleep(1.5)

            elif cmd == "7":
                print("🤖 Rotating left...")
                twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0.3))
                success = conn.move(twist, duration=1.0)
                print(f"  {'✓' if success else '✗'} Move command sent")
                time.sleep(1.5)

            elif cmd == "8":
                print("🤖 Rotating right...")
                twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, -0.3))
                success = conn.move(twist, duration=1.0)
                print(f"  {'✓' if success else '✗'} Move command sent")
                time.sleep(1.5)

            elif cmd == "9":
                print("🛑 Stopping...")
                conn.stop()
                print("  ✓ Stop command sent")

            elif cmd == "s":
                print("📊 Querying robot state...")
                show_state()

            else:
                print("❓ Unknown command")

    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted!")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback

        traceback.print_exc()
    finally:
        print("\n🛑 Stopping and cleaning up...")
        conn.stop()
        time.sleep(0.5)
        conn.disconnect()
        print("✓ Done")


if __name__ == "__main__":
    main()
