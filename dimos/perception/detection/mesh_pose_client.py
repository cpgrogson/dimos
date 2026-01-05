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

"""
MeshPoseClient - HTTP client for the hosted mesh/pose estimation service.

This client communicates with a hosted service that runs:
1. SAM3: Text-prompted segmentation
2. SAM3D: 3D mesh reconstruction with depth-based scaling
3. FoundationPose: 6D pose estimation

The client takes existing DIMOS detection types and returns enhanced
Detection3DMesh objects with mesh data and accurate poses.
"""

from __future__ import annotations

import base64
import io
import logging
from typing import TYPE_CHECKING, Optional
from urllib.parse import urljoin

import numpy as np
from PIL import Image as PILImage
import requests

from dimos.perception.detection.type.detection3d.mesh import Detection3DMesh

if TYPE_CHECKING:
    from dimos_lcm.sensor_msgs import CameraInfo

    from dimos.msgs.sensor_msgs import Image
    from dimos.perception.detection.type.detection3d.imageDetections3DPC import (
        ImageDetections3DPC,
    )
    from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC

logger = logging.getLogger(__name__)


class MeshPoseClient:
    """
    HTTP client for the hosted mesh/pose estimation service.

    This client enhances Detection3DPC objects with:
    - 3D mesh reconstruction (from SAM3D)
    - Accurate 6D pose estimation (from FoundationPose)

    Example usage:
        client = MeshPoseClient("http://mesh-pose-service:8080")

        # Enhance a single detection
        enhanced = client.enhance_detection(
            detection=det3d,
            color_image=rgb,
            depth_image=depth,
            camera_info=camera_info,
        )

        # Save the mesh
        if enhanced.has_mesh:
            enhanced.save_mesh("/tmp/object.obj")
    """

    def __init__(
        self,
        service_url: str = "http://localhost:8080",
        timeout: float = 300.0,
    ):
        """
        Initialize the client.

        Args:
            service_url: Base URL of the mesh/pose gateway service.
            timeout: Request timeout in seconds (default 5 minutes for heavy processing).
        """
        self.service_url = service_url.rstrip("/")
        self.timeout = timeout
        self._session = requests.Session()

    def health_check(self) -> bool:
        """
        Check if the service is healthy.

        Returns:
            True if service responds with 200 OK, False otherwise.
        """
        try:
            resp = self._session.get(
                urljoin(self.service_url, "/health"),
                timeout=5.0,
            )
            return resp.status_code == 200
        except Exception as e:
            logger.warning(f"Health check failed: {e}")
            return False

    def enhance_detection(
        self,
        detection: Detection3DPC,
        color_image: Image,
        depth_image: Image,
        camera_info: CameraInfo,
    ) -> Detection3DMesh:
        """
        Enhance a Detection3DPC with mesh and accurate 6D pose.

        Sends the detection to the hosted service which runs:
        1. SAM3 segmentation using the label as text prompt
        2. SAM3D mesh reconstruction with depth-based scaling
        3. FoundationPose 6D pose estimation

        Args:
            detection: The Detection3DPC to enhance.
            color_image: RGB color image (dimos.msgs.sensor_msgs.Image).
            depth_image: Depth image in meters (dimos.msgs.sensor_msgs.Image).
            camera_info: Camera intrinsics (dimos_lcm.sensor_msgs.CameraInfo).

        Returns:
            Detection3DMesh with mesh_obj and fp_position/fp_orientation if
            the service call succeeds, or a Detection3DMesh with just the
            original detection data if it fails.
        """
        try:
            # Build request payload
            payload = self._build_request_payload(
                detection=detection,
                color_image=color_image,
                depth_image=depth_image,
                camera_info=camera_info,
            )

            logger.info(f"Sending request to mesh/pose service for label='{detection.name}'")

            # Make HTTP request
            resp = self._session.post(
                urljoin(self.service_url, "/process"),
                json=payload,
                timeout=self.timeout,
            )

            if resp.status_code != 200:
                logger.error(f"Service error: {resp.status_code} - {resp.text}")
                return Detection3DMesh.from_detection3d_pc(detection)

            # Parse response
            data = resp.json()
            return self._parse_response(detection, data)

        except requests.exceptions.Timeout:
            logger.error(f"Request timed out after {self.timeout}s")
            return Detection3DMesh.from_detection3d_pc(detection)
        except Exception as e:
            logger.error(f"Request failed: {e}")
            return Detection3DMesh.from_detection3d_pc(detection)

    def enhance_detections(
        self,
        detections: ImageDetections3DPC,
        color_image: Image,
        depth_image: Image,
        camera_info: CameraInfo,
    ) -> list[Detection3DMesh]:
        """
        Enhance multiple detections with mesh and accurate pose.

        Processes detections sequentially. For parallel processing,
        the service itself handles pipeline parallelism internally.

        Args:
            detections: ImageDetections3DPC containing multiple detections.
            color_image: RGB color image.
            depth_image: Depth image in meters.
            camera_info: Camera intrinsics.

        Returns:
            List of Detection3DMesh objects, one for each input detection.
        """
        enhanced = []
        for det in detections.detections:
            result = self.enhance_detection(
                detection=det,
                color_image=color_image,
                depth_image=depth_image,
                camera_info=camera_info,
            )
            enhanced.append(result)
        return enhanced

    def get_mesh_and_pose(
        self,
        label: str,
        bbox: tuple[float, float, float, float],
        color_image: Image,
        depth_image: Image,
        camera_info: CameraInfo,
    ) -> dict:
        """
        Get mesh and pose for an object without requiring Detection3DPC.

        This is a simpler interface for integration with Object class.

        Args:
            label: Object label/name for SAM3 text prompt.
            bbox: 2D bounding box (x1, y1, x2, y2).
            color_image: RGB color image.
            depth_image: Depth image in meters.
            camera_info: Camera intrinsics.

        Returns:
            Dictionary with keys:
            - mesh_obj: bytes | None - Raw .obj mesh data
            - mesh_dimensions: tuple | None - (sx, sy, sz) dimensions
            - fp_position: tuple | None - (x, y, z) position
            - fp_orientation: tuple | None - (x, y, z, w) quaternion

        Raises:
            requests.exceptions.Timeout: If request times out.
            RuntimeError: If service returns an error.
        """
        payload = self._build_payload_from_raw(
            label=label,
            bbox=bbox,
            color_image=color_image,
            depth_image=depth_image,
            camera_info=camera_info,
        )

        logger.info(f"Sending request to mesh/pose service for label='{label}'")

        resp = self._session.post(
            urljoin(self.service_url, "/process"),
            json=payload,
            timeout=self.timeout,
        )

        if resp.status_code != 200:
            raise RuntimeError(f"Service error: {resp.status_code} - {resp.text}")

        data = resp.json()
        return self._parse_raw_response(data, label)

    def _build_payload_from_raw(
        self,
        label: str,
        bbox: tuple[float, float, float, float],
        color_image: Image,
        depth_image: Image,
        camera_info: CameraInfo,
    ) -> dict:
        """Build JSON payload from raw components."""
        import cv2

        # Encode RGB image as base64 PNG
        rgb_np = color_image.to_opencv()
        if rgb_np.ndim == 3 and rgb_np.shape[2] == 3:
            if hasattr(color_image, "_is_bgr") and color_image._is_bgr:
                rgb_np = cv2.cvtColor(rgb_np, cv2.COLOR_BGR2RGB)

        pil_img = PILImage.fromarray(rgb_np.astype(np.uint8))
        buf = io.BytesIO()
        pil_img.save(buf, format="PNG")
        image_rgb_b64 = base64.b64encode(buf.getvalue()).decode("utf-8")

        # Encode depth as base64 raw float32 bytes
        depth_np = depth_image.to_opencv()
        if depth_np.dtype != np.float32:
            depth_np = depth_np.astype(np.float32)
        depth_b64 = base64.b64encode(depth_np.tobytes()).decode("utf-8")

        # Extract camera intrinsics matrix
        K = [
            [camera_info.K[0], camera_info.K[1], camera_info.K[2]],
            [camera_info.K[3], camera_info.K[4], camera_info.K[5]],
            [camera_info.K[6], camera_info.K[7], camera_info.K[8]],
        ]

        return {
            "image_rgb_b64": image_rgb_b64,
            "depth_b64": depth_b64,
            "K": K,
            "label": label,
            "bbox": list(bbox),
        }

    def _parse_raw_response(self, data: dict, label: str) -> dict:
        """Parse service response into a simple dict."""
        mesh_obj = None
        if data.get("mesh_b64"):
            try:
                mesh_obj = base64.b64decode(data["mesh_b64"])
            except Exception as e:
                logger.warning(f"Failed to decode mesh: {e}")

        mesh_dimensions = None
        if data.get("bbox_3d"):
            bbox_3d = data["bbox_3d"]
            mesh_dimensions = (
                bbox_3d.get("sx", 0.0),
                bbox_3d.get("sy", 0.0),
                bbox_3d.get("sz", 0.0),
            )

        fp_position = None
        fp_orientation = None
        if data.get("pose"):
            pose = data["pose"]
            if pose.get("position"):
                pos = pose["position"]
                fp_position = (pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0))
            if pose.get("orientation"):
                ori = pose["orientation"]
                fp_orientation = (
                    ori.get("x", 0.0),
                    ori.get("y", 0.0),
                    ori.get("z", 0.0),
                    ori.get("w", 1.0),
                )

        logger.info(
            f"Received response for '{label}': "
            f"mesh={'yes' if mesh_obj else 'no'}, "
            f"pose={'yes' if fp_position else 'no'}"
        )

        return {
            "mesh_obj": mesh_obj,
            "mesh_dimensions": mesh_dimensions,
            "fp_position": fp_position,
            "fp_orientation": fp_orientation,
        }

    def _build_request_payload(
        self,
        detection: Detection3DPC,
        color_image: Image,
        depth_image: Image,
        camera_info: CameraInfo,
    ) -> dict:
        """Build the JSON payload for the service request."""
        return self._build_payload_from_raw(
            label=detection.name,
            bbox=detection.bbox,
            color_image=color_image,
            depth_image=depth_image,
            camera_info=camera_info,
        )

    def _parse_response(
        self,
        detection: Detection3DPC,
        data: dict,
    ) -> Detection3DMesh:
        """Parse the service response and create Detection3DMesh."""
        # Extract mesh bytes
        mesh_obj = None
        if data.get("mesh_b64"):
            try:
                mesh_obj = base64.b64decode(data["mesh_b64"])
            except Exception as e:
                logger.warning(f"Failed to decode mesh: {e}")

        # Extract mesh dimensions
        mesh_dimensions = None
        if data.get("bbox_3d"):
            bbox_3d = data["bbox_3d"]
            mesh_dimensions = (
                bbox_3d.get("sx", 0.0),
                bbox_3d.get("sy", 0.0),
                bbox_3d.get("sz", 0.0),
            )

        # Extract pose
        fp_position = None
        fp_orientation = None
        if data.get("pose"):
            pose = data["pose"]
            if pose.get("position"):
                pos = pose["position"]
                fp_position = (pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0))
            if pose.get("orientation"):
                ori = pose["orientation"]
                fp_orientation = (
                    ori.get("x", 0.0),
                    ori.get("y", 0.0),
                    ori.get("z", 0.0),
                    ori.get("w", 1.0),
                )

        fp_confidence = data.get("confidence", 1.0)

        logger.info(
            f"Received response for '{detection.name}': "
            f"mesh={'yes' if mesh_obj else 'no'}, "
            f"pose={'yes' if fp_position else 'no'}"
        )

        return Detection3DMesh.from_detection3d_pc(
            detection=detection,
            mesh_obj=mesh_obj,
            mesh_dimensions=mesh_dimensions,
            fp_position=fp_position,
            fp_orientation=fp_orientation,
            fp_confidence=fp_confidence,
        )

    def close(self):
        """Close the HTTP session."""
        self._session.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


__all__ = ["MeshPoseClient"]
