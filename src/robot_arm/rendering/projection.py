"""
3D Projection Matrices for Rendering.

Implements perspective and orthographic projection to transform 3D world
coordinates to 2D screen coordinates.

Mathematical Background:
    The rendering pipeline transforms points through several coordinate systems:
        1. Model Space → World Space (model matrix)
        2. World Space → Camera/View Space (view matrix)
        3. View Space → Clip Space (projection matrix)
        4. Clip Space → NDC (perspective divide)
        5. NDC → Screen Space (viewport transform)

    For perspective projection, the 4×4 matrix creates the illusion of depth:
        - Objects further away appear smaller
        - Parallel lines converge at vanishing points

    The projection matrix maps the view frustum to a unit cube [-1,1]³.

Author: Mykola Balyk

File Location: src/robot_arm/rendering/projection.py
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np
from numpy.typing import NDArray

from robot_arm.core.vectors import Vector3, Vector4


def perspective_matrix(
    fov_y: float,
    aspect_ratio: float,
    near: float,
    far: float
) -> NDArray[np.floating]:
    """
    Create perspective projection matrix.

    The perspective matrix transforms view-space coordinates to clip-space,
    creating the foreshortening effect where distant objects appear smaller.

    Matrix form:
        | f/aspect   0       0                   0               |
        |    0       f       0                   0               |
        |    0       0   (far+near)/(near-far)  2·far·near/(near-far) |
        |    0       0      -1                   0               |

    where f = 1/tan(fov_y/2) = cot(fov_y/2)

    After multiplication, the w component becomes -z, so perspective divide
    (x/w, y/w, z/w) creates the depth effect.

    Args:
        fov_y: Vertical field of view in radians
        aspect_ratio: Width/height ratio of the viewport
        near: Distance to near clipping plane (must be > 0)
        far: Distance to far clipping plane (must be > near)

    Returns:
        4×4 perspective projection matrix

    Raises:
        ValueError: If near/far planes are invalid

    Example:
        >>> P = perspective_matrix(math.radians(60), 16/9, 0.1, 100)
        >>> P @ view_space_point  # Returns clip-space coordinates
    """
    # TODO: Implement perspective projection matrix (Mykola)
    #
    # Steps:
    # 1. Validate: near > 0, far > near
    # 2. f = 1.0 / math.tan(fov_y / 2.0)
    # 3. Build 4×4 matrix with elements as shown in docstring
    # 4. Return as numpy array
    raise NotImplementedError("perspective_matrix: Mykola to implement")


def orthographic_matrix(
    left: float,
    right: float,
    bottom: float,
    top: float,
    near: float,
    far: float
) -> NDArray[np.floating]:
    """
    Create orthographic projection matrix.

    Orthographic projection preserves parallel lines (no foreshortening).
    Useful for technical/engineering visualizations and 2D rendering.

    Matrix form:
        | 2/(r-l)    0        0      -(r+l)/(r-l) |
        |    0    2/(t-b)     0      -(t+b)/(t-b) |
        |    0       0    -2/(f-n)   -(f+n)/(f-n) |
        |    0       0        0           1        |

    Args:
        left, right: X-axis bounds
        bottom, top: Y-axis bounds
        near, far: Z-axis bounds

    Returns:
        4×4 orthographic projection matrix
    """
    # TODO: Implement orthographic projection matrix (Mykola)
    raise NotImplementedError("orthographic_matrix: Mykola to implement")


def viewport_transform(
    width: int,
    height: int,
    x_offset: int = 0,
    y_offset: int = 0
) -> NDArray[np.floating]:
    """
    Create viewport transformation matrix.

    Maps Normalized Device Coordinates (NDC) in [-1,1] to screen pixels.

    NDC origin is at center, +Y up.
    Screen origin is typically top-left, +Y down.

    Transform:
        x_screen = (x_ndc + 1) * width/2 + x_offset
        y_screen = (1 - y_ndc) * height/2 + y_offset  (flips Y)

    Args:
        width: Viewport width in pixels
        height: Viewport height in pixels
        x_offset: Horizontal offset from origin
        y_offset: Vertical offset from origin

    Returns:
        4×4 viewport transformation matrix
    """
    # TODO: Implement viewport transform (Mykola)
    raise NotImplementedError("viewport_transform: Mykola to implement")


def project_point(
    point: Vector3,
    view_matrix: NDArray[np.floating],
    projection_matrix: NDArray[np.floating],
    viewport_matrix: NDArray[np.floating] | None = None
) -> Tuple[float, float, float]:
    """
    Project a 3D point to 2D screen coordinates.

    Applies the full rendering pipeline:
        1. View transform: world → camera space
        2. Projection: camera → clip space
        3. Perspective divide: clip → NDC
        4. Viewport (optional): NDC → screen

    Args:
        point: 3D point in world coordinates
        view_matrix: 4×4 view/camera matrix
        projection_matrix: 4×4 projection matrix
        viewport_matrix: Optional 4×4 viewport matrix

    Returns:
        (x, y, depth) tuple:
            - If viewport_matrix provided: x,y in pixels
            - Otherwise: x,y in NDC [-1,1]
            - depth is the normalized Z for depth testing
    """
    # TODO: Implement full projection pipeline (Mykola)
    #
    # Steps:
    # 1. Convert point to homogeneous: p = [x, y, z, 1]
    # 2. Apply view: p_view = view_matrix @ p
    # 3. Apply projection: p_clip = projection_matrix @ p_view
    # 4. Perspective divide: p_ndc = p_clip[:3] / p_clip[3]
    # 5. If viewport_matrix: p_screen = viewport_matrix @ [p_ndc, 1]
    # 6. Return (x, y, z)
    raise NotImplementedError("project_point: Mykola to implement")


def project_line_segment(
    start: Vector3,
    end: Vector3,
    view_matrix: NDArray[np.floating],
    projection_matrix: NDArray[np.floating],
    viewport_matrix: NDArray[np.floating] | None = None
) -> Tuple[Tuple[float, float], Tuple[float, float], bool]:
    """
    Project a 3D line segment to 2D screen coordinates.

    Handles near-plane clipping to avoid artifacts when the line
    crosses behind the camera.

    Args:
        start, end: 3D endpoints of the line segment
        view_matrix: 4×4 view/camera matrix
        projection_matrix: 4×4 projection matrix
        viewport_matrix: Optional 4×4 viewport matrix

    Returns:
        ((x1, y1), (x2, y2), visible):
            - Projected 2D endpoints
            - visible is False if line is entirely behind camera
    """
    # TODO: Implement line projection with clipping (Mykola)
    raise NotImplementedError("project_line_segment: Mykola to implement")


@dataclass
class ProjectionConfig:
    """
    Configuration for projection system.

    Stores all parameters needed to set up the projection pipeline.
    """

    # Perspective parameters
    fov_y: float = math.radians(60)  # 60° vertical FOV
    aspect_ratio: float = 16 / 9
    near_plane: float = 0.1
    far_plane: float = 100.0

    # Viewport parameters
    screen_width: int = 1280
    screen_height: int = 720

    def get_perspective_matrix(self) -> NDArray[np.floating]:
        """Get configured perspective matrix."""
        return perspective_matrix(
            self.fov_y,
            self.aspect_ratio,
            self.near_plane,
            self.far_plane
        )

    def get_viewport_matrix(self) -> NDArray[np.floating]:
        """Get configured viewport matrix."""
        return viewport_transform(self.screen_width, self.screen_height)

    def update_aspect_ratio(self, width: int, height: int) -> None:
        """Update aspect ratio from new screen dimensions."""
        self.screen_width = width
        self.screen_height = height
        self.aspect_ratio = width / height if height > 0 else 1.0
