# Copyright 2025 Dimensional Inc.
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

from queue import Empty, Queue
import threading

from cv_bridge import CvBridge
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import (
    CameraInfo as ROSCameraInfo,
    CompressedImage as ROSCompressedImage,
    Image as ROSImage,
    PointCloud2 as ROSPointCloud2,
)
from std_msgs.msg import Header as ROSHeader
import tf2_ros
from vision_msgs.msg import (
    Detection2DArray as ROSDetection2DArray,
    Detection3DArray as ROSDetection3DArray,
)

from dimos.core import Module, rpc
from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.sensor_msgs import CameraInfo, Image
from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode
from dimos.perception.detection.mesh_pose_client import MeshPoseClient
from dimos.perception.detection.objectDB import ObjectDB
from dimos.perception.detection.type import ImageDetections2D
from dimos.perception.detection.type.detection3d.object import (
    Object,
    aggregate_pointclouds,
    to_detection3d_array,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ObjectSceneRegistrationModule(Module):
    """Module for detecting objects in camera images using YOLO-E with 2D and 3D detection.

    Optionally supports mesh/pose enhancement via hosted service (SAM3 + SAM3D + FoundationPose).
    """

    _detector: Yoloe2DDetector | None = None
    _node: Node | None = None
    _bridge: CvBridge | None = None
    _spin_thread: threading.Thread | None = None
    _processing_thread: threading.Thread | None = None
    _processing_queue: Queue | None = None
    _running: bool = False
    _camera_info: CameraInfo | None = None
    _tf_buffer: tf2_ros.Buffer | None = None
    _tf_listener: tf2_ros.TransformListener | None = None
    _object_db: ObjectDB | None = None
    _mesh_pose_client: MeshPoseClient | None = None

    def __init__(
        self,
        image_topic: str = "/camera/color/image_raw/compressed",
        depth_topic: str = "/camera/aligned_depth_to_color/image_raw/compressedDepth",
        camera_info_topic: str = "/camera/color/camera_info",
        detections_2d_topic: str = "/object_detections/bbox",
        detections_3d_topic: str = "/object_detections/bbox3d",
        overlay_topic: str = "/object_detections/overlay",
        pointcloud_topic: str = "/object_detections/pointcloud",
        target_frame: str = "map",
        mesh_pose_service_url: str | None = None,
    ) -> None:
        """
        Initialize ObjectSceneRegistrationModule.

        Args:
            image_topic: ROS topic for compressed color images.
            depth_topic: ROS topic for compressed depth images.
            camera_info_topic: ROS topic for camera intrinsics.
            detections_2d_topic: ROS topic to publish 2D detections.
            detections_3d_topic: ROS topic to publish 3D detections.
            overlay_topic: ROS topic to publish detection overlay image.
            pointcloud_topic: ROS topic to publish aggregated pointclouds.
            target_frame: Target TF frame for 3D detections (e.g., "map").
            mesh_pose_service_url: Optional URL of hosted mesh/pose service
                (e.g., "http://localhost:8080"). If provided, detections
                will be enhanced with mesh data and accurate 6D pose.
        """
        super().__init__()
        self._image_topic = image_topic
        self._depth_topic = depth_topic
        self._camera_info_topic = camera_info_topic
        self._detections_2d_topic = detections_2d_topic
        self._detections_3d_topic = detections_3d_topic
        self._overlay_topic = overlay_topic
        self._pointcloud_topic = pointcloud_topic
        self._target_frame = target_frame
        self._mesh_pose_service_url = mesh_pose_service_url
        self._bridge = CvBridge()

    @rpc
    def start(self) -> None:
        super().start()
        self._running = True

        # Initialize ObjectDB for spatial memory
        self._object_db = ObjectDB()
        logger.info("Initialized ObjectDB for spatial memory")

        # Initialize mesh/pose client if URL provided
        if self._mesh_pose_service_url:
            self._mesh_pose_client = MeshPoseClient(
                service_url=self._mesh_pose_service_url,
            )
            if self._mesh_pose_client.health_check():
                logger.info(f"Mesh/pose enhancement enabled via {self._mesh_pose_service_url}")
            else:
                logger.warning(
                    f"Mesh/pose service at {self._mesh_pose_service_url} is not healthy, "
                    "detections will not be enhanced"
                )
                self._mesh_pose_client.close()
                self._mesh_pose_client = None
        else:
            logger.info("Mesh/pose enhancement disabled (no service URL provided)")

        # Initialize detector (uses yoloe-11l-seg-pf.pt for LRPC mode by default)
        self._detector = Yoloe2DDetector(
            prompt_mode=YoloePromptMode.LRPC,
        )
        logger.info("Initialized YOLO-E detector in LRPC (prompt-free) mode")

        # Initialize ROS
        if not rclpy.ok():
            rclpy.init()
        self._node = Node("object_scene_registration")

        # Initialize TF2 buffer and listener
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node)

        # Wait for camera info once (it rarely changes)
        logger.info(f"Waiting for camera_info on {self._camera_info_topic}...")
        success, camera_info_msg = wait_for_message(
            ROSCameraInfo,
            self._node,
            self._camera_info_topic,
            time_to_wait=10.0,
        )
        if success and camera_info_msg:
            self._camera_info = CameraInfo.from_ros_msg(camera_info_msg)
            logger.info(f"Received camera_info: {camera_info_msg.width}x{camera_info_msg.height}")
        else:
            raise RuntimeError(f"Timeout waiting for camera_info on {self._camera_info_topic}")

        # Publishers
        self._detections_2d_pub = self._node.create_publisher(
            ROSDetection2DArray,
            self._detections_2d_topic,
            10,
        )
        self._detections_3d_pub = self._node.create_publisher(
            ROSDetection3DArray,
            self._detections_3d_topic,
            10,
        )
        self._overlay_pub = self._node.create_publisher(
            ROSImage,
            self._overlay_topic,
            10,
        )
        self._pointcloud_pub = self._node.create_publisher(
            ROSPointCloud2,
            self._pointcloud_topic,
            10,
        )

        # Set up synchronized subscribers for color and depth
        self._color_sub = message_filters.Subscriber(
            self._node,
            ROSCompressedImage,
            self._image_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self._depth_sub = message_filters.Subscriber(
            self._node,
            ROSCompressedImage,
            self._depth_topic,
            qos_profile=qos_profile_sensor_data,
        )

        # Synchronize color and depth with approximate time sync
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._color_sub, self._depth_sub],
            queue_size=10,
            slop=0.1,  # 100ms tolerance
        )
        self._sync.registerCallback(self._on_synced_images)

        logger.info("Synchronized subscribers:")
        logger.info(f"  Color: {self._image_topic}")
        logger.info(f"  Depth: {self._depth_topic}")
        logger.info("Publishing:")
        logger.info(f"  2D detections: {self._detections_2d_topic}")
        logger.info(f"  3D detections: {self._detections_3d_topic}")
        logger.info(f"  Overlay: {self._overlay_topic}")
        logger.info(f"  Pointcloud: {self._pointcloud_topic}")

        # Start spin thread
        self._spin_thread = threading.Thread(
            target=self._spin_node,
            daemon=True,
            name="ObjectSceneRegistrationSpinThread",
        )
        self._spin_thread.start()

        # Start processing thread (keeps callback fast, processes in background)
        self._processing_queue = Queue(maxsize=1)
        self._processing_thread = threading.Thread(
            target=self._processing_loop,
            daemon=True,
            name="ObjectSceneRegistrationProcessingThread",
        )
        self._processing_thread.start()

    def _spin_node(self) -> None:
        """Spin the ROS node."""
        while self._running and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.1)

    def _processing_loop(self) -> None:
        """Background thread for heavy image processing."""
        while self._running:
            try:
                color_msg, depth_msg = self._processing_queue.get(timeout=0.5)
                self._process_images(color_msg, depth_msg)
            except Exception:
                logger.exception("Error processing images")

    @rpc
    def stop(self) -> None:
        """Stop the module and clean up resources."""
        self._running = False

        if self._processing_thread and self._processing_thread.is_alive():
            self._processing_thread.join(timeout=1.0)

        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)

        if self._detector:
            self._detector.stop()
            self._detector = None

        if self._mesh_pose_client:
            self._mesh_pose_client.close()
            self._mesh_pose_client = None

        if self._node:
            self._node.destroy_node()
            self._node = None

        if self._object_db:
            self._object_db.clear()
            self._object_db = None

        logger.info("ObjectSceneRegistrationModule stopped")
        super().stop()

    @property
    def object_db(self) -> ObjectDB | None:
        """Access the ObjectDB for querying detected objects."""
        return self._object_db

    def _convert_compressed_depth_image(self, msg: ROSCompressedImage) -> Image | None:
        """Convert ROS compressedDepth image to internal Image type."""
        if not self._bridge:
            return None

        try:
            import cv2

            # compressedDepth format has a 12-byte header followed by PNG data
            # cv_bridge doesn't handle this format correctly
            if "compressedDepth" in msg.format:
                depth_header_size = 12
                raw_data = np.frombuffer(msg.data, dtype=np.uint8)

                if len(raw_data) <= depth_header_size:
                    logger.warning("compressedDepth data too small")
                    return None

                # Decode the PNG portion (after header)
                png_data = raw_data[depth_header_size:]
                depth_cv = cv2.imdecode(png_data, cv2.IMREAD_UNCHANGED)

                if depth_cv is None:
                    logger.warning("Failed to decode compressedDepth PNG data")
                    return None
            else:
                # Regular compressed image
                depth_cv = self._bridge.compressed_imgmsg_to_cv2(msg)

            # Convert to meters if uint16 (assuming mm depth)
            if depth_cv.dtype == np.uint16:
                depth_cv = depth_cv.astype(np.float32) / 1000.0
            elif depth_cv.dtype != np.float32:
                depth_cv = depth_cv.astype(np.float32)

            depth_image = Image.from_numpy(depth_cv)
            depth_image.from_ros_header(msg.header)
            return depth_image
        except Exception as e:
            logger.warning(f"Failed to convert compressed depth image: {e}")
            return None

    def _on_synced_images(
        self, color_msg: ROSCompressedImage, depth_msg: ROSCompressedImage
    ) -> None:
        """Queue synchronized images for processing (fast callback)."""
        if not self._processing_queue:
            return

        # Drop old data if queue is full (we always want the latest)
        if self._processing_queue.full():
            try:
                self._processing_queue.get_nowait()
            except Exception:
                pass

        self._processing_queue.put((color_msg, depth_msg))

    def _process_images(self, color_msg: ROSCompressedImage, depth_msg: ROSCompressedImage) -> None:
        """Process synchronized color and depth images (runs in background thread)."""
        if not self._detector or not self._bridge or not self._camera_info:
            return

        # Convert color image
        cv_image = self._bridge.compressed_imgmsg_to_cv2(color_msg, "rgb8")
        color_image = Image.from_numpy(cv_image)
        color_image.from_ros_header(color_msg.header)

        # Convert compressed depth image
        depth_image = self._convert_compressed_depth_image(depth_msg)
        if depth_image is None:
            return

        # Run 2D detection
        detections_2d: ImageDetections2D = self._detector.process_image(color_image)

        ros_detections_2d = detections_2d.to_ros_detection2d_array()
        self._detections_2d_pub.publish(ros_detections_2d)

        # Publish overlay image
        overlay_image = detections_2d.overlay()
        overlay_msg = self._bridge.cv2_to_imgmsg(overlay_image.to_opencv(), encoding="rgb8")
        overlay_msg.header = color_msg.header
        self._overlay_pub.publish(overlay_msg)

        # Process 3D detections
        self._process_3d_detections(detections_2d, color_image, depth_image, color_msg.header)

    def _process_3d_detections(
        self,
        detections_2d: ImageDetections2D,
        color_image: Image,
        depth_image: Image,
        header: ROSHeader,
    ) -> None:
        """Convert 2D detections to 3D and publish."""
        if self._camera_info is None:
            return

        # Look up transform from camera frame to target frame (e.g., map)
        if self._tf_buffer is not None:
            try:
                ros_transform = self._tf_buffer.lookup_transform(
                    self._target_frame,
                    color_image.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.4),
                )
                camera_transform = Transform.from_ros_transform_stamped(ros_transform)
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                logger.warning("Failed to lookup transform from camera frame to target frame")
                return

        objects = Object.from_2d(
            detections_2d=detections_2d,
            color_image=color_image,
            depth_image=depth_image,
            camera_info=self._camera_info,
            camera_transform=camera_transform,
        )

        if not objects:
            return

        # Add objects to spatial memory database FIRST (deduplication happens here)
        # This returns matched/updated objects so we can skip mesh/pose for known objects
        if self._object_db is not None:
            objects = self._object_db.add_objects(objects)

        # Optionally enhance objects with mesh and accurate 6D pose from hosted service
        # Only process objects that don't already have mesh data (avoids redundant requests)
        if self._mesh_pose_client is not None:
            for obj in objects:
                if obj.has_mesh:
                    continue  # Skip - already has mesh from previous detection
                try:
                    result = self._mesh_pose_client.get_mesh_and_pose(
                        label=obj.name,
                        bbox=obj.bbox,
                        color_image=color_image,
                        depth_image=depth_image,
                        camera_info=self._camera_info,
                    )
                    obj.mesh_obj = result.get("mesh_obj")
                    obj.mesh_dimensions = result.get("mesh_dimensions")
                    obj.fp_position = result.get("fp_position")
                    obj.fp_orientation = result.get("fp_orientation")
                except Exception as e:
                    logger.warning(f"Mesh/pose enhancement failed for {obj.name}: {e}")

        detections_3d = to_detection3d_array(objects)
        ros_detections_3d = detections_3d.to_ros_msg()
        self._detections_3d_pub.publish(ros_detections_3d)

        aggregated_pc = aggregate_pointclouds(self._object_db.get_objects())
        if aggregated_pc is not None:
            ros_pc = aggregated_pc.to_ros_msg()
            self._pointcloud_pub.publish(ros_pc)


object_scene_registration_module = ObjectSceneRegistrationModule.blueprint

__all__ = ["ObjectSceneRegistrationModule", "object_scene_registration_module"]
