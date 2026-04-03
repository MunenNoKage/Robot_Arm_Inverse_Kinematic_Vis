"""
Rendering subpackage for 3D visualization.

This module provides:
    - projection: Perspective and orthographic projection matrices
    - camera: Camera model with view matrix construction
"""

from robot_arm.rendering.projection import (
    perspective_matrix,
    orthographic_matrix,
    viewport_transform,
    project_point,
)
from robot_arm.rendering.camera import (
    Camera,
    OrbitCamera,
)

__all__ = [
    "perspective_matrix",
    "orthographic_matrix",
    "viewport_transform",
    "project_point",
    "Camera",
    "OrbitCamera",
]
