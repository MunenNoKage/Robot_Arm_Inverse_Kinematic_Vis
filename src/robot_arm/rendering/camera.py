"""
Camera System for 3D Visualization.

Implements camera models that produce the view matrix for the rendering
pipeline. The view matrix transforms world coordinates to camera/eye space.

Mathematical Background:
    The view matrix is the inverse of the camera's world transform.
    It's constructed from an orthonormal basis (right, up, forward) and
    the camera position:

        View = | right.x   right.y   right.z   -dot(right, eye) |
               | up.x      up.y      up.z      -dot(up, eye)    |
               | -fwd.x   -fwd.y    -fwd.z     dot(fwd, eye)    |
               |   0        0         0             1            |

    The orthonormal basis is computed from:
        - forward = normalize(target - eye)
        - right = normalize(forward × up_hint)
        - up = right × forward

Author: Mykola Balyk

File Location: src/robot_arm/rendering/camera.py
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from robot_arm.core.vectors import Vector3


@dataclass
class Camera:
    """
    Basic camera with position, target, and up vector.

    Computes the view matrix for the rendering pipeline.

    Attributes:
        position: Camera position in world coordinates (eye point)
        target: Point the camera looks at (center of interest)
        up: World up direction hint (usually Y-axis)
    """

    position: Vector3
    target: Vector3
    up: Vector3 = Vector3(0.0, 1.0, 0.0)

    def get_forward(self) -> Vector3:
        """Get normalized forward direction (toward target)."""
        return (self.target - self.position).normalized()

    def get_right(self) -> Vector3:
        """Get normalized right direction."""
        forward = self.get_forward()
        return forward.cross(self.up).normalized()

    def get_up_corrected(self) -> Vector3:
        """Get corrected up vector (perpendicular to forward and right)."""
        right = self.get_right()
        forward = self.get_forward()
        return right.cross(forward)

    def get_view_matrix(self) -> NDArray[np.floating]:
        """
        Compute the 4×4 view matrix.

        The view matrix transforms world coordinates to camera space
        where:
            - Camera is at origin
            - Camera looks down -Z axis
            - +Y is up, +X is right

        Construction:
            1. Build orthonormal basis (right, up, -forward)
            2. Create rotation matrix from basis vectors
            3. Create translation to move world to camera origin
            4. View = Rotation · Translation

        Returns:
            4×4 view matrix
        """
        # TODO: Implement view matrix construction (Mykola)
        #
        # Steps:
        # 1. forward = (target - position).normalized()
        # 2. right = (forward × up).normalized()
        # 3. up_corrected = right × forward
        # 4. Build matrix:
        #    Row 0: right.x, right.y, right.z, -right·position
        #    Row 1: up.x, up.y, up.z, -up·position
        #    Row 2: -fwd.x, -fwd.y, -fwd.z, forward·position
        #    Row 3: 0, 0, 0, 1
        raise NotImplementedError("Camera.get_view_matrix: Mykola to implement")

    def move_forward(self, distance: float) -> None:
        """Move camera forward (toward target)."""
        direction = self.get_forward()
        self.position = self.position + direction * distance

    def move_right(self, distance: float) -> None:
        """Move camera to the right."""
        direction = self.get_right()
        self.position = self.position + direction * distance

    def move_up(self, distance: float) -> None:
        """Move camera up."""
        direction = self.get_up_corrected()
        self.position = self.position + direction * distance


@dataclass
class OrbitCamera:
    """
    Orbit camera that rotates around a target point.

    Useful for interactive visualization where the user wants to
    view the robot arm from different angles.

    The camera position is computed from spherical coordinates:
        x = r · sin(θ) · cos(φ)
        y = r · cos(θ)
        z = r · sin(θ) · sin(φ)

    where:
        - r is the distance to target
        - θ (theta) is the polar angle (0 at top)
        - φ (phi) is the azimuthal angle (rotation around Y)

    Attributes:
        target: Point to orbit around
        distance: Distance from target
        azimuth: Horizontal rotation angle (radians)
        elevation: Vertical angle from horizontal (radians)
        up: World up direction

    Example:
        >>> camera = OrbitCamera(Vector3.zero(), distance=10.0)
        >>> camera.rotate(0.1, 0.05)  # Mouse drag
        >>> view = camera.get_view_matrix()
    """

    target: Vector3
    distance: float = 10.0
    azimuth: float = 0.0      # Rotation around Y axis
    elevation: float = 0.5    # Angle from horizontal (radians)
    up: Vector3 = Vector3(0.0, 1.0, 0.0)

    # Limits to prevent gimbal lock and camera flip
    min_elevation: float = -math.pi / 2 + 0.01
    max_elevation: float = math.pi / 2 - 0.01
    min_distance: float = 1.0
    max_distance: float = 100.0

    def get_position(self) -> Vector3:
        """
        Compute camera position from spherical coordinates.

        Returns:
            Camera position in world coordinates
        """
        # TODO: Implement spherical to Cartesian conversion (Mykola)
        #
        # x = target.x + distance · cos(elevation) · sin(azimuth)
        # y = target.y + distance · sin(elevation)
        # z = target.z + distance · cos(elevation) · cos(azimuth)
        raise NotImplementedError("OrbitCamera.get_position: Mykola to implement")

    def get_view_matrix(self) -> NDArray[np.floating]:
        """
        Compute view matrix for orbit camera.

        Returns:
            4×4 view matrix
        """
        # TODO: Implement using Camera class (Mykola)
        # Create a Camera with computed position and self.target
        raise NotImplementedError("OrbitCamera.get_view_matrix: Mykola to implement")

    def rotate(self, delta_azimuth: float, delta_elevation: float) -> None:
        """
        Rotate the camera around the target.

        Args:
            delta_azimuth: Change in horizontal angle
            delta_elevation: Change in vertical angle
        """
        self.azimuth += delta_azimuth
        self.elevation = max(
            self.min_elevation,
            min(self.max_elevation, self.elevation + delta_elevation)
        )

    def zoom(self, delta: float) -> None:
        """
        Zoom in/out by changing distance.

        Args:
            delta: Positive zooms in (decreases distance)
        """
        self.distance = max(
            self.min_distance,
            min(self.max_distance, self.distance - delta)
        )

    def pan(self, delta_x: float, delta_y: float) -> None:
        """
        Pan the camera (move target).

        Args:
            delta_x: Horizontal pan
            delta_y: Vertical pan
        """
        # TODO: Implement pan in screen-aligned directions (Mykola)
        # Get right and up vectors, move target accordingly
        raise NotImplementedError("OrbitCamera.pan: Mykola to implement")

    def reset(self) -> None:
        """Reset camera to default orientation."""
        self.azimuth = 0.0
        self.elevation = 0.5
        self.distance = 10.0


def look_at(eye: Vector3, target: Vector3, up: Vector3 = Vector3.unit_y()) -> NDArray[np.floating]:
    """
    Create view matrix using look-at parameters.

    Convenience function for creating a view matrix without
    instantiating a Camera object.

    Args:
        eye: Camera position
        target: Point to look at
        up: World up direction

    Returns:
        4×4 view matrix
    """
    camera = Camera(position=eye, target=target, up=up)
    return camera.get_view_matrix()
