"""
3D and 4D Vector Operations for Robot Arm Kinematics.

This module provides lightweight vector classes optimized for clarity and
educational purposes, demonstrating core linear algebra operations:
    - Vector addition, subtraction, scalar multiplication
    - Dot product, cross product
    - Normalization, magnitude
    - Homogeneous coordinate conversion

Author: Team (shared utility)

File Location: src/robot_arm/core/vectors.py
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterator

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True, slots=True)
class Vector3:
    """
    Immutable 3D vector for positions and directions.

    Represents a point or direction in 3D Euclidean space. All operations
    return new Vector3 instances (immutability ensures thread-safety and
    prevents accidental modifications during kinematic computations).

    Attributes:
        x: X component
        y: Y component
        z: Z component

    Example:
        >>> v1 = Vector3(1.0, 2.0, 3.0)
        >>> v2 = Vector3(4.0, 5.0, 6.0)
        >>> v1 + v2
        Vector3(x=5.0, y=7.0, z=9.0)
        >>> v1.dot(v2)
        32.0
    """

    x: float
    y: float
    z: float

    @classmethod
    def zero(cls) -> Vector3:
        """Return the zero vector (0, 0, 0)."""
        return cls(0.0, 0.0, 0.0)

    @classmethod
    def unit_x(cls) -> Vector3:
        """Return unit vector along X-axis."""
        return cls(1.0, 0.0, 0.0)

    @classmethod
    def unit_y(cls) -> Vector3:
        """Return unit vector along Y-axis."""
        return cls(0.0, 1.0, 0.0)

    @classmethod
    def unit_z(cls) -> Vector3:
        """Return unit vector along Z-axis."""
        return cls(0.0, 0.0, 1.0)

    @classmethod
    def from_array(cls, arr: NDArray[np.floating]) -> Vector3:
        """Create Vector3 from numpy array."""
        return cls(float(arr[0]), float(arr[1]), float(arr[2]))

    def __add__(self, other: Vector3) -> Vector3:
        """Vector addition: v1 + v2."""
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Vector3) -> Vector3:
        """Vector subtraction: v1 - v2."""
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> Vector3:
        """Scalar multiplication: v * s."""
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)

    def __rmul__(self, scalar: float) -> Vector3:
        """Scalar multiplication: s * v."""
        return self * scalar

    def __truediv__(self, scalar: float) -> Vector3:
        """Scalar division: v / s."""
        return Vector3(self.x / scalar, self.y / scalar, self.z / scalar)

    def __neg__(self) -> Vector3:
        """Negation: -v."""
        return Vector3(-self.x, -self.y, -self.z)

    def __iter__(self) -> Iterator[float]:
        """Allow unpacking: x, y, z = v."""
        yield self.x
        yield self.y
        yield self.z

    def dot(self, other: Vector3) -> float:
        """
        Compute dot product with another vector.

        The dot product a·b = |a||b|cos(θ) is fundamental for:
            - Computing angles between vectors
            - Projecting one vector onto another
            - Building orthonormal bases (camera view matrix)

        Args:
            other: Vector to compute dot product with

        Returns:
            Scalar dot product value
        """
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: Vector3) -> Vector3:
        """
        Compute cross product with another vector.

        The cross product a×b produces a vector perpendicular to both inputs.
        Used extensively in:
            - Jacobian computation: axis × (end_effector - joint_position)
            - Camera coordinate frame construction
            - Normal vector computation

        Args:
            other: Vector to compute cross product with

        Returns:
            Perpendicular vector with magnitude |a||b|sin(θ)
        """
        return Vector3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    @property
    def magnitude(self) -> float:
        """Compute Euclidean length (L2 norm) of the vector."""
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    @property
    def magnitude_squared(self) -> float:
        """Compute squared magnitude (avoids sqrt for comparisons)."""
        return self.x**2 + self.y**2 + self.z**2

    def normalized(self) -> Vector3:
        """
        Return unit vector in the same direction.

        Raises:
            ValueError: If vector has zero magnitude
        """
        mag = self.magnitude
        if mag < 1e-10:
            raise ValueError("Cannot normalize zero-length vector")
        return self / mag

    def to_array(self) -> NDArray[np.floating]:
        """Convert to numpy array for matrix operations."""
        return np.array([self.x, self.y, self.z], dtype=np.float64)

    def to_homogeneous(self, w: float = 1.0) -> "Vector4":
        """
        Convert to homogeneous coordinates (4D).

        Homogeneous coordinates allow representing both position (w=1)
        and direction (w=0) vectors, enabling unified transformation
        matrices that combine rotation and translation.

        Args:
            w: Homogeneous component (1.0 for points, 0.0 for directions)

        Returns:
            4D vector [x, y, z, w]
        """
        return Vector4(self.x, self.y, self.z, w)


@dataclass(frozen=True, slots=True)
class Vector4:
    """
    Immutable 4D vector for homogeneous coordinates.

    Homogeneous coordinates extend 3D vectors with a 4th component 'w',
    enabling:
        - Unified rotation + translation in a single 4×4 matrix multiply
        - Perspective division after projection (x/w, y/w, z/w)
        - Distinguishing points (w=1) from directions (w=0)

    Attributes:
        x, y, z: Spatial components
        w: Homogeneous component
    """

    x: float
    y: float
    z: float
    w: float

    @classmethod
    def from_point(cls, point: Vector3) -> "Vector4":
        """Create homogeneous point (w=1) from 3D position."""
        return cls(point.x, point.y, point.z, 1.0)

    @classmethod
    def from_direction(cls, direction: Vector3) -> "Vector4":
        """Create homogeneous direction (w=0) from 3D vector."""
        return cls(direction.x, direction.y, direction.z, 0.0)

    @classmethod
    def from_array(cls, arr: NDArray[np.floating]) -> "Vector4":
        """Create Vector4 from numpy array."""
        return cls(float(arr[0]), float(arr[1]), float(arr[2]), float(arr[3]))

    def to_vector3(self) -> Vector3:
        """
        Convert back to 3D by perspective divide.

        For points (w≠0): divides x,y,z by w
        For directions (w≈0): returns x,y,z directly
        """
        if abs(self.w) > 1e-10:
            return Vector3(self.x / self.w, self.y / self.w, self.z / self.w)
        return Vector3(self.x, self.y, self.z)

    def to_array(self) -> NDArray[np.floating]:
        """Convert to numpy array for matrix operations."""
        return np.array([self.x, self.y, self.z, self.w], dtype=np.float64)

    def __iter__(self) -> Iterator[float]:
        """Allow unpacking."""
        yield self.x
        yield self.y
        yield self.z
        yield self.w
