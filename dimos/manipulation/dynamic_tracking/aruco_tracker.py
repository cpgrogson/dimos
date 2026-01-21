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

from dataclasses import dataclass
import math
from threading import Event, Thread
import time
from typing import Any

import cv2
import numpy as np
import rerun as rr
from scipy.spatial.transform import Rotation

from dimos.core import In, Module, ModuleConfig, Out, rpc
from dimos.dashboard.rerun_init import connect_rerun
from dimos.manipulation.control.orchestrator_client import OrchestratorClient
from dimos.manipulation.pinocchio.pin_kinematics import PinocchioIK
from dimos.msgs.geometry_msgs import Pose, Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import JointState
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class ArucoTrackerConfig(ModuleConfig):
    """Configuration for ArUco tracker."""

    marker_size: float = 0.1
    aruco_dict: int = cv2.aruco.DICT_4X4_50
    camera_frame_id: str = "camera_color_optical_frame"  # Frame ID for the camera
    rate: float = 1  # Rate in Hz - defines speed of execution (process loop then sleep)
    max_loops: int = 5  # Maximum number of loops to process
    move_robot_to_aruco: bool = True  # Whether to move the robot to the ArUco marker
    move_robot_to_aruco_rotation: bool = (
        False  # Whether to follow ArUco rotation (False = fixed orientation)
    )
    safety_max_joint_delta_deg: float = 15.0  # Max allowed joint angle change (degrees) per command
    expected_marker_count: int = 4  # Expected number of ArUco markers

    # IK config
    mjcf_path: str = ""  # Path to MJCF file for IK solver (required)
    ee_joint_id: int = 6  # End-effector joint ID in the kinematic chain
    task_name: str = "traj_arm"  # Task name for OrchestratorClient trajectory execution
    min_move_distance_m: float = 0.003  # Minimum EE movement (meters) to trigger trajectory


class ArucoTracker(Module[ArucoTrackerConfig]):
    """
    ArUco marker tracker that detects markers in camera images and computes their transforms.

    Subscribes to camera images and camera info, detects ArUco markers,
    and publishes their transforms relative to the camera frame.

    Uses Pinocchio IK to compute joint angles from target EE pose and sends
    joint positions via OrchestratorClient to the ControlOrchestrator.
    """

    # Transport ports
    color_image: In[Image]
    camera_info: In[CameraInfo]
    joint_state: In[JointState]  # Current joint positions from orchestrator (for IK warm-start)
    annotated_image: Out[Image]

    config: ArucoTrackerConfig
    default_config = ArucoTrackerConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # ArUco detector
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(self.config.aruco_dict)
        self._aruco_params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._aruco_params)

        # params from callback
        self._camera_matrix: np.ndarray | None = None
        self._dist_coeffs: np.ndarray | None = None
        self._latest_image: Image | None = None

        self._stop_event = Event()
        self._processing_thread: Thread | None = None
        self._loop_count = 0

        self._orchestrator_client = OrchestratorClient()
        self._last_q: np.ndarray | None = None

        if not self.config.mjcf_path:
            raise ValueError("mjcf_path must be set for IK solver")
        self._ik_solver = PinocchioIK(
            mjcf_path=self.config.mjcf_path,
            ee_joint_id=self.config.ee_joint_id,
        )
        logger.info(f"IK solver initialized with MJCF: {self.config.mjcf_path}")

    @rpc
    def start(self) -> None:
        """Start the ArUco tracker by subscribing to camera streams."""
        super().start()

        connect_rerun()
        self._disposables.add(self.camera_info.observable().subscribe(self._update_camera_info))
        self._disposables.add(self.color_image.observable().subscribe(self._store_latest_image))
        self._disposables.add(self.joint_state.observable().subscribe(self._update_joint_state))

        # Setup OrchestratorClient for the configured task
        if not self._orchestrator_client.select_task(self.config.task_name):
            logger.warning(f"Failed to select task '{self.config.task_name}' in OrchestratorClient")

        self._loop_count = 0
        self._stop_event.clear()
        self._processing_thread = Thread(
            target=self._processing_loop, daemon=True, name="ArucoTracker"
        )
        self._processing_thread.start()

    @rpc
    def stop(self) -> None:
        """Stop the ArUco tracker."""
        self._stop_event.set()
        if self._processing_thread is not None and self._processing_thread.is_alive():
            self._processing_thread.join(timeout=2.0)
        # Cleanup OrchestratorClient
        self._orchestrator_client.stop()
        super().stop()

    def _store_latest_image(self, image: Image) -> None:
        """Store the latest image for processing."""
        self._latest_image = image

    def _update_camera_info(self, camera_info: CameraInfo) -> None:
        """Update camera intrinsics from CameraInfo message."""
        if len(camera_info.K) == 9:
            fx, _, cx, _, fy, cy, _, _, _ = camera_info.K
            self._camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
            self._dist_coeffs = (
                np.array(camera_info.D, dtype=np.float32) if camera_info.D else np.zeros(5)
            )

    def _update_joint_state(self, joint_state: JointState) -> None:
        """Update last_q from actual robot joint positions."""
        if joint_state.position:
            self._last_q = np.array(joint_state.position)

    def _log_transform_to_rerun(self, transform: Transform) -> None:
        """Log a transform to Rerun."""
        rr.log(f"world/tf/{transform.child_frame_id}", transform.to_rerun())

    def _check_joint_safety(self, q_solution: np.ndarray) -> bool:
        """Check if joint movement is within safety limits"""
        q_new = q_solution.flatten()
        q_old = self._last_q.flatten()
        joint_deltas_deg = np.abs(np.degrees(q_new - q_old))
        max_delta_deg = np.max(joint_deltas_deg)
        logger.debug(f"IK solution (deg): {[f'{np.degrees(angle):.1f}' for angle in q_new]}")

        if max_delta_deg > self.config.safety_max_joint_delta_deg:
            max_joint_idx = np.argmax(joint_deltas_deg)
            logger.error(
                f"Safety check failed: joint {max_joint_idx + 1} delta {max_delta_deg:.1f}° "
                f"exceeds limit {self.config.safety_max_joint_delta_deg:.1f}°"
            )
            return False
        return True

    def _processing_loop(self) -> None:
        """Processing loop that runs at the configured rate."""
        period = 1.0 / self.config.rate
        logger.info(f"ArUco processing loop started at {self.config.rate}Hz")

        while not self._stop_event.is_set() and self._loop_count < self.config.max_loops:
            loop_start = time.time()
            try:
                if self._latest_image is None:
                    time.sleep(period)
                    continue
                self._process_image(self._latest_image)
                self._loop_count += 1
                logger.debug(f"Processed image {self._loop_count}/{self.config.max_loops}")
            except Exception as e:
                logger.error(f"Error in processing loop: {e}")

            elapsed = time.time() - loop_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
        logger.info(f"ArUco processing loop completed after {self._loop_count} iterations")

    def _process_image(self, image: Image) -> None:
        """Process image to detect ArUco markers and average their poses."""
        if self._camera_matrix is None or self._dist_coeffs is None:
            return  # Skip if camera info not ready yet

        # Convert image for visualization (keep original format)
        if image.format.name == "RGB":
            display_image = image.data.copy()
            gray = cv2.cvtColor(image.data, cv2.COLOR_RGB2GRAY)
        elif image.format.name == "BGR":
            display_image = image.data.copy()
            gray = cv2.cvtColor(image.data, cv2.COLOR_BGR2GRAY)
        else:
            display_image = image.data.copy()
            gray = image.data
        corners, ids, _ = self._detector.detectMarkers(gray)

        # Check marker count
        num_detected = 0 if ids is None else len(ids)
        if num_detected != self.config.expected_marker_count:
            logger.debug(
                f"Detected {num_detected} markers, expected {self.config.expected_marker_count}"
            )
            self._publish_annotated_image(display_image, image.format.name)
            return

        # Estimate and average marker poses
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.config.marker_size, self._camera_matrix, self._dist_coeffs
        )
        avg_position, avg_quat = self._average_marker_poses(rvecs, tvecs)

        # Create and publish transforms
        transform = self._set_transforms("avg", avg_position, avg_quat, image.ts)
        if transform is None:
            logger.error("Failed to create transform")
            self._publish_annotated_image(display_image, image.format.name)
            return

        aruco_wrt_robot_base = self.tf.get("base_link", "aruco_avg")
        if aruco_wrt_robot_base is None:
            logger.error("Failed to get aruco_avg wrt base_link")
            self._publish_annotated_image(display_image, image.format.name)
            return

        reach_pose = self._compute_reach_pose(aruco_wrt_robot_base)
        if reach_pose is not None:
            self._send_joint_command(reach_pose)

        # Draw markers on display image (show all detected markers)
        self._draw_markers(
            display_image,
            corners,
            ids,
            rvecs,
            tvecs,
            image.format.name,
        )

    def _compute_reach_pose(self, aruco_wrt_robot_base: Transform) -> Pose | None:
        """Compute the reach pose from ArUco marker position with offset."""
        if not self.config.move_robot_to_aruco:
            return None

        # Position with offset
        t = aruco_wrt_robot_base.translation
        position = Vector3(t.x - 0.05, t.y, t.z + 0.20)

        # Rotation: follow ArUco or use fixed default
        if self.config.move_robot_to_aruco_rotation:
            rpy = aruco_wrt_robot_base.rotation.to_euler()
            roll = ((rpy.x + math.pi + math.pi) % (2 * math.pi)) - math.pi  # wrap to [-π, π]
            pitch = np.clip(rpy.y, -math.pi / 2, math.pi / 2)
            yaw = np.clip(rpy.z + math.pi / 2, -math.pi / 2, math.pi / 2)
            orientation = Quaternion.from_euler(Vector3(roll, pitch, yaw))
        else:
            orientation = Quaternion.from_euler(Vector3(math.pi, 0.0, 0.0))

        pose = Pose(position=position, orientation=orientation)
        logger.debug(f"Reach pose: {pose}")
        return pose

    def _average_marker_poses(
        self, rvecs: np.ndarray, tvecs: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Average multiple marker poses into a single pose."""
        rotations = []
        positions = []
        for i in range(len(rvecs)):
            rot_matrix, _ = cv2.Rodrigues(rvecs[i][0])
            rotations.append(Rotation.from_matrix(rot_matrix))
            positions.append(tvecs[i][0])

        avg_position = np.mean(positions, axis=0)
        avg_rotation = Rotation.concatenate(rotations).mean()
        avg_quat = avg_rotation.as_quat()  # [x, y, z, w]
        return avg_position, avg_quat


    def _send_joint_command(self, goal_pose: Pose) -> None:
        """Compute IK and send joint positions via OrchestratorClient trajectory."""
        if self._last_q is None:
            logger.warning("No joint state received yet, skipping IK command")
            return

        # Solve IK
        q_solution, success = self._ik_solver.solve_ik_from_pose(goal_pose, self._last_q)
        if not success:
            logger.warning("IK did not converge, skipping command")
            return
        if not self._check_joint_safety(q_solution):
            return

        # Skip if movement is too small
        current_ee_pose = self._ik_solver.forward_kinematics(self._last_q)
        goal_position = np.array([goal_pose.x, goal_pose.y, goal_pose.z])
        distance = np.linalg.norm(goal_position - current_ee_pose.translation)
        if distance < self.config.min_move_distance_m:
            logger.debug(f"Movement too small ({distance * 1000:.1f}mm), skipping")
            return

        # Send joint command via OrchestratorClient
        joint_positions = q_solution.flatten().tolist()
        success = self._orchestrator_client.go_to_joint_positions(
            joint_positions, self.config.task_name
        )

        # success return if trajectory was sent
        if success:
            logger.debug(f"Sent trajectory to {[f'{p:.3f}' for p in joint_positions]}")
        else:
            logger.warning("Failed to send trajectory via OrchestratorClient")


    def _set_transforms(
        self, marker_id: int | str, tvec: np.ndarray, quat: np.ndarray, timestamp: float
    ) -> Transform | None:
        # Create ArUco marker transform (camera_optical -> aruco_{marker_id})
        aruco_transform = Transform(
            translation=Vector3(float(tvec[0]), float(tvec[1]), float(tvec[2])),
            rotation=Quaternion(float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])),
            frame_id="camera_color_optical_frame",
            child_frame_id=f"aruco_{marker_id}",
            ts=timestamp,
        )
        self.tf.publish(aruco_transform)
        self._log_transform_to_rerun(aruco_transform)

        # Publish world -> base_link transform (static, but republish for TF polling)
        robot_base_to_world_transform = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="world",
            child_frame_id="base_link",
            ts=timestamp,
        )
        self.tf.publish(robot_base_to_world_transform)
        self._log_transform_to_rerun(robot_base_to_world_transform)

        # Get EE pose from forward kinematics
        if self._last_q is not None:
            ee_se3 = self._ik_solver.forward_kinematics(self._last_q)
            ee_transform = Transform(
                translation=Vector3(*ee_se3.translation),
                rotation=Quaternion.from_rotation_matrix(ee_se3.rotation),
                frame_id="base_link",
                child_frame_id="ee_link",
                ts=timestamp,
            )
            self.tf.publish(ee_transform)
            self._log_transform_to_rerun(ee_transform)
        return aruco_transform


    def _publish_annotated_image(self, display_image: np.ndarray, image_format: str) -> None:
        """Publish the annotated image to subscribers and Rerun."""

        if image_format == "BGR":
            publish_image = cv2.cvtColor(display_image, cv2.COLOR_BGR2RGB)
        else:
            publish_image = display_image
        annotated_msg = Image(
            data=publish_image,
            format=ImageFormat.RGB,
            frame_id=self.config.camera_frame_id,
            ts=time.time(),
        )
        self.annotated_image.publish(annotated_msg)
        rr.log("aruco/annotated", rr.Image(publish_image))


    def _draw_markers(
        self,
        display_image: np.ndarray,
        corners: list[np.ndarray],
        ids: np.ndarray,
        rvecs: np.ndarray,
        tvecs: np.ndarray,
        image_format: str,
    ) -> None:
        """Draw detected markers and axes on the image, then publish."""
        # Draw all detected markers
        cv2.aruco.drawDetectedMarkers(display_image, corners, ids)

        # Draw axes for each marker
        for i in range(len(ids)):
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

            axis_length = self.config.marker_size * 0.5
            cv2.drawFrameAxes(
                display_image,
                self._camera_matrix,
                self._dist_coeffs,
                rvec,
                tvec,
                axis_length,
            )
        self._publish_annotated_image(display_image, image_format)


aruco_tracker = ArucoTracker.blueprint
__all__ = ["ArucoTracker", "ArucoTrackerConfig", "aruco_tracker"]
