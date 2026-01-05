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
Detection3DMesh - Detection3DPC enhanced with 3D mesh and accurate 6D pose.

This type extends Detection3DPC to include:
- A reconstructed 3D mesh (.obj format)
- Accurate 6D pose from FoundationPose (not just pointcloud center)
- Mesh-derived bounding box dimensions
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Optional

from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC

if TYPE_CHECKING:
    from dimos_lcm.sensor_msgs import CameraInfo

    from dimos.msgs.sensor_msgs import Image


@dataclass
class Detection3DMesh(Detection3DPC):
    """
    Detection3DPC enhanced with 3D mesh and accurate 6D pose.

    This class extends Detection3DPC to include mesh data from SAM3D
    and accurate 6D pose estimation from FoundationPose.

    Attributes:
        mesh_obj: Raw .obj mesh file as bytes (None if mesh not available)
        mesh_dimensions: Bounding box dimensions from mesh (sx, sy, sz) in meters
        fp_position: Position from FoundationPose (x, y, z in camera frame)
        fp_orientation: Orientation quaternion from FoundationPose (x, y, z, w)
        fp_confidence: Confidence score from FoundationPose
    """

    # Mesh data
    mesh_obj: bytes | None = None

    # Mesh-derived dimensions (more accurate than pointcloud-derived)
    mesh_dimensions: tuple[float, float, float] | None = None

    # FoundationPose results (stored separately to allow pose override)
    fp_position: tuple[float, float, float] | None = None
    fp_orientation: tuple[float, float, float, float] | None = None  # (x, y, z, w)
    fp_confidence: float = 1.0

    @property
    def pose(self) -> PoseStamped:
        """
        Get 6D pose, preferring FoundationPose result if available.

        Returns:
            PoseStamped with position and orientation. If FoundationPose
            data is available, uses that; otherwise falls back to
            pointcloud center with identity rotation.
        """
        if self.fp_position is not None and self.fp_orientation is not None:
            return PoseStamped(
                ts=self.ts,
                frame_id=self.frame_id,
                position=Vector3(*self.fp_position),
                orientation=self.fp_orientation,
            )
        # Fallback to parent implementation (pointcloud center, identity rotation)
        return PoseStamped(
            ts=self.ts,
            frame_id=self.frame_id,
            position=self.center,
            orientation=(0.0, 0.0, 0.0, 1.0),
        )

    @property
    def has_mesh(self) -> bool:
        """Check if mesh data is available."""
        return self.mesh_obj is not None and len(self.mesh_obj) > 0

    @property
    def has_accurate_pose(self) -> bool:
        """Check if FoundationPose data is available."""
        return self.fp_position is not None and self.fp_orientation is not None

    def get_bounding_box_dimensions(self) -> tuple[float, float, float]:
        """
        Get bounding box dimensions, preferring mesh-derived if available.

        Returns:
            Tuple of (width, height, depth) in meters.
        """
        if self.mesh_dimensions is not None:
            return self.mesh_dimensions
        # Fallback to pointcloud-derived dimensions
        return super().get_bounding_box_dimensions()

    def save_mesh(self, path: str) -> bool:
        """
        Save mesh to an .obj file.

        Args:
            path: File path to save the mesh to.

        Returns:
            True if saved successfully, False if no mesh data available.
        """
        if not self.has_mesh:
            return False
        try:
            with open(path, "wb") as f:
                f.write(self.mesh_obj)
            return True
        except Exception:
            return False

    def to_repr_dict(self) -> dict[str, Any]:
        """Return dictionary representation for display."""
        parent_dict = super().to_repr_dict()
        return {
            **parent_dict,
            "mesh": "yes" if self.has_mesh else "no",
            "fp_pose": "yes" if self.has_accurate_pose else "no",
        }

    @classmethod
    def from_detection3d_pc(
        cls,
        detection: Detection3DPC,
        mesh_obj: bytes | None = None,
        mesh_dimensions: tuple[float, float, float] | None = None,
        fp_position: tuple[float, float, float] | None = None,
        fp_orientation: tuple[float, float, float, float] | None = None,
        fp_confidence: float = 1.0,
    ) -> Detection3DMesh:
        """
        Create Detection3DMesh from an existing Detection3DPC.

        This is the primary factory method for enhancing a Detection3DPC
        with mesh and pose data from the hosted service.

        Args:
            detection: The source Detection3DPC to enhance.
            mesh_obj: Raw .obj mesh bytes from SAM3D.
            mesh_dimensions: Bounding box dimensions from mesh (sx, sy, sz).
            fp_position: Position from FoundationPose (x, y, z).
            fp_orientation: Orientation quaternion from FoundationPose (x, y, z, w).
            fp_confidence: Confidence score from FoundationPose.

        Returns:
            A new Detection3DMesh with all data from the source detection
            plus the mesh and pose enhancements.
        """
        return cls(
            # From Detection2DBBox (via Detection3D -> Detection3DPC)
            image=detection.image,
            depth=detection.depth,
            bbox=detection.bbox,
            track_id=detection.track_id,
            class_id=detection.class_id,
            confidence=detection.confidence,
            name=detection.name,
            ts=detection.ts,
            # From Detection3D
            transform=detection.transform,
            frame_id=detection.frame_id,
            # From Detection3DPC
            pointcloud=detection.pointcloud,
            # New fields for Detection3DMesh
            mesh_obj=mesh_obj,
            mesh_dimensions=mesh_dimensions,
            fp_position=fp_position,
            fp_orientation=fp_orientation,
            fp_confidence=fp_confidence,
        )


__all__ = ["Detection3DMesh"]
