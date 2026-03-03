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
Query G1 robot state to diagnose issues.
"""

import json
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_api import (
    ROBOT_API_ID_LOCO_GET_BALANCE_MODE,
    ROBOT_API_ID_LOCO_GET_FSM_ID,
    ROBOT_API_ID_LOCO_GET_FSM_MODE,
)
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


def main():
    print("=" * 60)
    print("G1 Robot State Diagnostic")
    print("=" * 60)

    # Initialize DDS
    print("\n1. Initializing DDS on eth0...")
    ChannelFactoryInitialize(0, "eth0")

    # Create client
    print("2. Creating LocoClient...")
    client = LocoClient()
    client.SetTimeout(10.0)
    client.Init()

    # Register GET APIs to query state
    print("3. Registering state query APIs...")
    client._RegistApi(ROBOT_API_ID_LOCO_GET_FSM_ID, 0)
    client._RegistApi(ROBOT_API_ID_LOCO_GET_FSM_MODE, 0)
    client._RegistApi(ROBOT_API_ID_LOCO_GET_BALANCE_MODE, 0)

    time.sleep(0.5)

    # Query current state
    print("\n4. Querying robot state...")

    try:
        # Get FSM ID
        code, data = client._Call(ROBOT_API_ID_LOCO_GET_FSM_ID, "{}")
        if code == 0 and data:
            fsm_data = json.loads(data) if isinstance(data, str) else data
            print(f"\n  FSM ID: {fsm_data}")
            print(f"    Code: {code}")
        else:
            print(f"  ✗ Failed to get FSM ID: code={code}, data={data}")
    except Exception as e:
        print(f"  ✗ Error getting FSM ID: {e}")

    try:
        # Get FSM Mode
        code, data = client._Call(ROBOT_API_ID_LOCO_GET_FSM_MODE, "{}")
        if code == 0 and data:
            mode_data = json.loads(data) if isinstance(data, str) else data
            print(f"\n  FSM Mode: {mode_data}")
            print(f"    Code: {code}")
        else:
            print(f"  ✗ Failed to get FSM Mode: code={code}, data={data}")
    except Exception as e:
        print(f"  ✗ Error getting FSM Mode: {e}")

    try:
        # Get Balance Mode
        code, data = client._Call(ROBOT_API_ID_LOCO_GET_BALANCE_MODE, "{}")
        if code == 0 and data:
            balance_data = json.loads(data) if isinstance(data, str) else data
            print(f"\n  Balance Mode: {balance_data}")
            print(f"    Code: {code}")
        else:
            print(f"  ✗ Failed to get Balance Mode: code={code}, data={data}")
    except Exception as e:
        print(f"  ✗ Error getting Balance Mode: {e}")

    # Print FSM ID meanings
    print("\n" + "=" * 60)
    print("FSM ID Reference:")
    print("=" * 60)
    print("  0 = Zero Torque (limp)")
    print("  1 = Damp (passive)")
    print("  3 = Sit")
    print("  200 = Start (AI mode)")
    print("  702 = Lie2StandUp")
    print("  706 = Squat2StandUp / StandUp2Squat (toggle)")
    print("=" * 60)

    print("\n5. Testing commands...")
    print("\n  Current state retrieved. You can now:")
    print("    - Use FSM ID 706 to stand up from squat")
    print("    - Use FSM ID 702 to stand up from lying")
    print("    - Use FSM ID 1 (Damp) before standing if robot is stiff")


if __name__ == "__main__":
    main()
