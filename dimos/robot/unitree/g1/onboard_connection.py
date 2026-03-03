#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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
Native Unitree SDK2-based connection for G1 robot onboard control.
"""

import json
import threading
import time
from typing import Any

from reactivex import operators as ops
from reactivex.observable import Observable
from reactivex.subject import Subject

from dimos.core import rpc
from dimos.core.resource import Resource
from dimos.msgs.geometry_msgs import Pose, Quaternion, Transform, Twist, Vector3
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.robot.unitree.type.lowstate import LowStateMsg
from dimos.utils.decorators.decorators import simple_mcache
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()

# Lazy import of Unitree SDK to avoid import errors when SDK not available
_unitree_sdk_imported = False
_LocoClient = None
_ChannelSubscriber = None
_ChannelFactoryInitialize = None
_LowState_ = None
_LOCO_API_IDS = None


def _import_unitree_sdk() -> None:
    """Lazy import of Unitree SDK modules."""
    global _unitree_sdk_imported, _LocoClient, _ChannelSubscriber
    global _ChannelFactoryInitialize, _LowState_, _LOCO_API_IDS

    if _unitree_sdk_imported:
        return

    try:
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
            MotionSwitcherClient,
        )
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelSubscriber,
        )
        from unitree_sdk2py.g1.loco.g1_loco_api import (
            ROBOT_API_ID_LOCO_GET_BALANCE_MODE,
            ROBOT_API_ID_LOCO_GET_FSM_ID,
            ROBOT_API_ID_LOCO_GET_FSM_MODE,
        )
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

        globals()["_MotionSwitcherClient"] = MotionSwitcherClient
        _LocoClient = LocoClient
        _ChannelSubscriber = ChannelSubscriber
        _ChannelFactoryInitialize = ChannelFactoryInitialize
        _LowState_ = LowState_
        _LOCO_API_IDS = {
            "GET_FSM_ID": ROBOT_API_ID_LOCO_GET_FSM_ID,
            "GET_FSM_MODE": ROBOT_API_ID_LOCO_GET_FSM_MODE,
            "GET_BALANCE_MODE": ROBOT_API_ID_LOCO_GET_BALANCE_MODE,
        }
        _unitree_sdk_imported = True
        logger.info("Unitree SDK2 imported successfully")
    except ImportError as e:
        logger.error(f"Failed to import Unitree SDK2: {e}")
        raise ImportError(
            "Unitree SDK2 not found. Please install it following the guide at "
            "~/dimos/docs/usage/setup_robot/unitree_g1.md"
        ) from e


class G1OnboardConnection(Resource):
    """
    Native SDK2-based connection for G1 robot.

    This class provides the same interface as UnitreeWebRTCConnection but uses
    the native Unitree SDK2 for DDS-based communication. Suitable for onboard
    control running directly on the robot.

    Args:
        network_interface: Network interface name (e.g., "eth0", "wlan0")
        mode: Operation mode ("ai" for AI balance mode, "normal" for standard)
    """

    def __init__(self, network_interface: str = "eth0", mode: str = "ai") -> None:
        """Initialize the onboard connection."""
        _import_unitree_sdk()

        self.network_interface = network_interface
        self.mode = mode
        self.stop_timer: threading.Timer | None = None
        self.cmd_vel_timeout = 0.2
        self._running = False

        # Initialize DDS channel factory
        logger.info(f"Initializing DDS on interface: {network_interface}")
        _ChannelFactoryInitialize(0, network_interface)  # type: ignore[misc]

        # Create motion switcher client (required before using LocoClient)
        MotionSwitcherClient = globals().get("_MotionSwitcherClient")
        if MotionSwitcherClient:
            self.motion_switcher = MotionSwitcherClient()
            self.motion_switcher.SetTimeout(5.0)
            self.motion_switcher.Init()
            logger.info("Motion switcher initialized")
        else:
            self.motion_switcher = None
            logger.warning("Motion switcher not available")

        # Create locomotion client
        self.loco_client = _LocoClient()  # type: ignore[misc]
        self.loco_client.SetTimeout(10.0)
        self.loco_client.Init()

        # Register GET APIs for querying state
        if _LOCO_API_IDS:
            self.loco_client._RegistApi(_LOCO_API_IDS["GET_FSM_ID"], 0)
            self.loco_client._RegistApi(_LOCO_API_IDS["GET_FSM_MODE"], 0)
            self.loco_client._RegistApi(_LOCO_API_IDS["GET_BALANCE_MODE"], 0)

        # State subscribers
        self._lowstate_subject: Subject[Any] = Subject()
        self._lowstate_subscriber: Any = None
        self._mode_selected = False

        logger.info(f"G1 onboard connection initialized in {mode} mode")

    def start(self) -> None:
        """Start the connection and subscribers."""
        if self._running:
            return

        self._running = True

        # Check current mode
        if self.motion_switcher:
            try:
                code, result = self.motion_switcher.CheckMode()
                if code == 0 and result:
                    current_mode = result.get("name", "none")
                    logger.info(f"Current motion mode: {current_mode}")

                    if current_mode and current_mode != "none":
                        logger.warning(
                            f"Robot is in '{current_mode}' mode. "
                            "If SDK commands don't work, you may need to activate via controller: "
                            "L1+A then L1+UP"
                        )
            except Exception as e:
                logger.debug(f"Could not check current mode: {e}")

        # Select motion mode (required before FSM commands will work)
        if self.motion_switcher and not self._mode_selected:
            logger.info(f"Selecting motion mode: {self.mode}")
            code, _ = self.motion_switcher.SelectMode(self.mode)
            if code == 0:
                logger.info(f"✓ Motion mode '{self.mode}' selected successfully")
                self._mode_selected = True
                time.sleep(0.5)  # Give time for mode to activate

                # Verify mode was selected
                code, result = self.motion_switcher.CheckMode()
                if code == 0 and result:
                    active_mode = result.get("name", "unknown")
                    logger.info(f"Active mode after selection: {active_mode}")
            else:
                logger.error(
                    f"✗ Failed to select mode '{self.mode}': code={code}\n"
                    "  The robot may need to be activated via controller first:\n"
                    "  1. Press L1 + A on the controller\n"
                    "  2. Then press L1 + UP\n"
                    "  This enables the AI Sport client required for SDK control."
                )

        # Initialize low state subscriber
        self._lowstate_subscriber = _ChannelSubscriber("rt/lowstate", _LowState_)  # type: ignore[misc]
        self._lowstate_subscriber.Init(self._lowstate_handler, 10)

        logger.info("G1 onboard connection started")

    def stop(self) -> None:
        """Stop the robot and cleanup resources."""
        # Cancel any pending timers
        if self.stop_timer:
            self.stop_timer.cancel()
            self.stop_timer = None

        # Send stop command
        try:
            self.loco_client.StopMove()
        except Exception as e:
            logger.error(f"Error stopping robot: {e}")

        self._running = False
        logger.info("G1 onboard connection stopped")

    def disconnect(self) -> None:
        """Disconnect and cleanup all resources."""
        self.stop()
        # Note: SDK doesn't provide explicit cleanup methods

    def _get_fsm_id(self) -> int | None:
        """
        Query current FSM (Finite State Machine) ID.

        Returns:
            Current FSM ID or None if query fails

        FSM IDs:
            0 = Zero Torque (limp)
            1 = Damp (passive/damping mode)
            3 = Sit
            200 = Start (AI mode active)
            702 = Lie2StandUp
            706 = Squat2StandUp / StandUp2Squat (toggle)
        """
        if not _LOCO_API_IDS:
            logger.warning("LOCO API IDs not available")
            return None

        try:
            code, data = self.loco_client._Call(_LOCO_API_IDS["GET_FSM_ID"], "{}")
            if code == 0 and data:
                result = json.loads(data) if isinstance(data, str) else data
                fsm_id = result.get("data") if isinstance(result, dict) else result
                logger.debug(f"Current FSM ID: {fsm_id}")
                return fsm_id
            else:
                logger.warning(f"Failed to get FSM ID: code={code}, data={data}")
                return None
        except Exception as e:
            logger.error(f"Error getting FSM ID: {e}")
            return None

    def get_state(self) -> str:
        """
        Get human-readable robot state.

        Returns:
            String describing current robot state
        """
        fsm_id = self._get_fsm_id()

        if fsm_id is None:
            return "Unknown (query failed)"

        fsm_meanings = {
            0: "Zero Torque (limp)",
            1: "Damp (passive)",
            3: "Sit",
            200: "Start (AI mode)",
            702: "Lie2StandUp",
            706: "Squat2StandUp/StandUp2Squat",
        }

        meaning = fsm_meanings.get(fsm_id, f"Unknown FSM {fsm_id}")
        return f"FSM {fsm_id}: {meaning}"

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """
        Send movement command to the robot.

        Args:
            twist: Twist message with linear and angular velocities
                - linear.x: Forward/backward velocity (m/s)
                - linear.y: Left/right (strafe) velocity (m/s)
                - angular.z: Rotation velocity (rad/s)
            duration: How long to move (seconds). If 0, continuous until next command

        Returns:
            True if command sent successfully, False otherwise
        """
        # Extract velocities
        vx = twist.linear.x
        vy = twist.linear.y
        vyaw = twist.angular.z

        # Cancel existing timer
        if self.stop_timer:
            self.stop_timer.cancel()
            self.stop_timer = None

        try:
            if duration > 0:
                # Move for specific duration - let SDK handle timing
                # Don't use auto-stop timer for timed movements
                logger.info(f"Moving: vx={vx}, vy={vy}, vyaw={vyaw}, duration={duration}")
                code = self.loco_client.SetVelocity(vx, vy, vyaw, duration)
                if code != 0:
                    logger.warning(f"SetVelocity returned code: {code}")
                    return False
            else:
                # Continuous movement - set up auto-stop timer as safety feature
                def auto_stop() -> None:
                    try:
                        logger.debug("Auto-stop timer triggered")
                        self.loco_client.StopMove()
                    except Exception as e:
                        logger.error(f"Auto-stop failed: {e}")

                self.stop_timer = threading.Timer(self.cmd_vel_timeout, auto_stop)
                self.stop_timer.daemon = True
                self.stop_timer.start()

                logger.info(f"Continuous move: vx={vx}, vy={vy}, vyaw={vyaw}")
                self.loco_client.Move(vx, vy, vyaw, continous_move=True)

            return True
        except Exception as e:
            logger.error(f"Failed to send movement command: {e}")
            return False

    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[str, Any]:
        """
        Generic request publishing (for compatibility with WebRTC interface).

        This method provides compatibility with the WebRTC interface but maps
        to native SDK calls where possible.

        Args:
            topic: Request topic (e.g., "rt/api/sport/request")
            data: Request data dictionary with "api_id" and optional "parameter"

        Returns:
            Response dictionary
        """
        api_id = data.get("api_id")
        parameter = data.get("parameter", {})

        logger.debug(f"publish_request: topic={topic}, api_id={api_id}")

        # Map API IDs to SDK methods
        # Based on g1_loco_api.py constants
        try:
            if api_id == 7101:  # SET_FSM_ID
                fsm_id = parameter.get("data", 0)
                code = self.loco_client.SetFsmId(fsm_id)
                return {"code": code}
            elif api_id == 7105:  # SET_VELOCITY
                velocity = parameter.get("velocity", [0, 0, 0])
                duration = parameter.get("duration", 1.0)
                code = self.loco_client.SetVelocity(velocity[0], velocity[1], velocity[2], duration)
                return {"code": code}
            else:
                logger.warning(f"Unsupported API ID: {api_id}")
                return {"code": -1, "error": "unsupported_api"}
        except Exception as e:
            logger.error(f"publish_request failed: {e}")
            return {"code": -1, "error": str(e)}

    @rpc
    def standup(self) -> bool:
        """Stand up the robot based on mode."""
        try:
            if self.mode == "ai":
                return self.standup_ai()
            else:
                return self.standup_normal()
        except Exception as e:
            logger.error(f"Standup failed: {e}")
            return False

    def standup_ai(self) -> bool:
        """Stand up in AI balance mode."""
        try:
            # Check current state
            fsm_id = self._get_fsm_id()
            logger.info(f"Current state before standup: {self.get_state()}")

            # If robot is in zero torque (limp), enable damping first
            if fsm_id == 0:
                logger.info("Robot in zero torque, enabling damp mode...")
                code = self.loco_client.SetFsmId(1)  # Damp
                logger.info(f"  SetFsmId(1) returned: {code}")
                time.sleep(1.0)
                logger.info(f"  After damp: {self.get_state()}")

            # If already in AI mode (200), skip Start
            if fsm_id != 200:
                logger.info("Starting AI mode (FSM 200)...")
                code = self.loco_client.SetFsmId(200)  # Start
                logger.info(f"  SetFsmId(200) returned: {code}")
                time.sleep(1.5)
                logger.info(f"  After start: {self.get_state()}")

            # Stand up from squat
            logger.info("Executing Squat2StandUp (FSM 706)...")
            code = self.loco_client.SetFsmId(706)
            logger.info(f"  SetFsmId(706) returned: {code}")
            time.sleep(3.0)

            # Verify standing
            logger.info(f"Final state: {self.get_state()}")

            return True
        except Exception as e:
            logger.error(f"AI standup failed: {e}")
            import traceback

            traceback.print_exc()
            return False

    def standup_normal(self) -> bool:
        """Stand up in normal mode."""
        try:
            # Check current state
            logger.info(f"Current state before standup: {self.get_state()}")

            # Enable damping
            logger.info("Enabling damp mode (FSM 1)...")
            code = self.loco_client.SetFsmId(1)  # Damp
            logger.info(f"  SetFsmId(1) returned: {code}")
            time.sleep(1.0)
            logger.info(f"  After damp: {self.get_state()}")

            # Stand up from squat
            logger.info("Executing Squat2StandUp (FSM 706)...")
            code = self.loco_client.SetFsmId(706)
            logger.info(f"  SetFsmId(706) returned: {code}")
            time.sleep(3.0)

            # Verify standing
            logger.info(f"Final state: {self.get_state()}")

            return True
        except Exception as e:
            logger.error(f"Normal standup failed: {e}")
            import traceback

            traceback.print_exc()
            return False

    @rpc
    def liedown(self) -> bool:
        """Lie down the robot."""
        try:
            self.loco_client.StandUp2Squat()  # FSM 706 (toggles)
            time.sleep(1.0)
            self.loco_client.Damp()  # FSM 1
            return True
        except Exception as e:
            logger.error(f"Liedown failed: {e}")
            return False

    def color(self, color: int = 0, colortime: int = 60) -> bool:
        """
        Control LED color (not supported via SDK).

        This is a stub for API compatibility.
        """
        logger.warning("LED color control not supported via native SDK")
        return False

    # -------------------------------------------------------------------------
    # Observable Streams
    # -------------------------------------------------------------------------

    def _lowstate_handler(self, msg: Any) -> None:
        """Handle incoming low state messages."""
        self._lowstate_subject.on_next(msg)

    @simple_mcache
    def lowstate_stream(self) -> Observable[LowStateMsg]:
        """Get low-level state stream."""
        return backpressure(self._lowstate_subject)

    @simple_mcache
    def odom_stream(self) -> Observable[Pose]:
        """
        Get odometry stream from low state.

        Extracts IMU and body position from low state messages.
        """

        def lowstate_to_pose(lowstate: Any) -> Pose:
            # Extract IMU quaternion for orientation
            imu = lowstate.imu_state
            quat = Quaternion(
                x=imu.quaternion[0], y=imu.quaternion[1], z=imu.quaternion[2], w=imu.quaternion[3]
            )

            # Position from foot contacts (simplified - would need actual SLAM)
            # For now, return identity position with IMU orientation
            return Pose(
                position=Vector3(0.0, 0.0, 0.0),  # TODO: integrate actual odometry
                orientation=quat,
            )

        return backpressure(self.lowstate_stream().pipe(ops.map(lowstate_to_pose)))

    @simple_mcache
    def tf_stream(self) -> Observable[Transform]:
        """Get transform stream for base_link."""

        def pose_to_transform(pose: Pose) -> Transform:
            return Transform(
                translation=pose.position,
                rotation=pose.orientation,
                frame_id="odom",
                child_frame_id="base_link",
            )

        return backpressure(self.odom_stream().pipe(ops.map(pose_to_transform)))

    @simple_mcache
    def video_stream(self) -> Observable[Image]:
        """
        Get video stream (not available via SDK).

        Returns an empty observable. Use separate camera module for video.
        """
        logger.warning("Video stream not available via native SDK - use separate camera module")
        return Observable.empty()

    @simple_mcache
    def lidar_stream(self) -> Observable[PointCloud2]:
        """
        Get lidar stream (not available via SDK).

        Returns an empty observable. Lidar data requires separate subscription.
        """
        logger.warning("Lidar stream not available via native SDK")
        return Observable.empty()

    def get_video_stream(self, fps: int = 30) -> Observable[Image]:
        """Get video stream (API compatibility method)."""
        return self.video_stream()


__all__ = ["G1OnboardConnection"]
