"""
Transformation Matrices for 3D Robot Arm Kinematics.

This module implements:
    - Rotation matrices from SO(3): Rx, Ry, Rz
    - Translation matrices
    - Homogeneous 4×4 transformation matrices
    - Transform composition (chaining)

Mathematical Background:
    The Special Orthogonal Group SO(3) consists of all 3×3 orthogonal matrices
    with determinant +1 (proper rotations). Key properties:
        - R^T = R^(-1)  (inverse is transpose)
        - det(R) = 1    (preserves orientation)
        - R·R^T = I     (orthonormality)

    We embed these in 4×4 homogeneous form to unify rotation and translation.

Author: Nazar Pasichnyk (Primary), Team (review)

File Location: src/robot_arm/core/transforms.py
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np
from numpy.typing import NDArray

from robot_arm.core.vectors import Vector3, Vector4


def rotation_x(theta: float) -> NDArray[np.floating]:
    """
    Create 3×3 rotation matrix around X-axis.

    Rotates by angle theta (radians) using the right-hand rule:
        - Positive theta rotates Y toward Z
        - The X-axis remains fixed

    Matrix form:
        | 1    0        0     |
        | 0   cos(θ)  -sin(θ) |
        | 0   sin(θ)   cos(θ) |

    Args:
        theta: Rotation angle in radians

    Returns:
        3×3 numpy rotation matrix

    Example:
        >>> R = rotation_x(math.pi / 2)  # 90° rotation
        >>> R @ [0, 1, 0]  # Y-axis becomes Z-axis
        array([0, 0, 1])
    """
    # TODO: Implement rotation matrix around X-axis (Nazar)
    # Use math.cos, math.sin for clarity
    # Return numpy array with dtype=np.float64
    raise NotImplementedError("rotation_x: Nazar to implement")


def rotation_y(theta: float) -> NDArray[np.floating]:
    """
    Create 3×3 rotation matrix around Y-axis.

    Rotates by angle theta (radians) using the right-hand rule:
        - Positive theta rotates Z toward X
        - The Y-axis remains fixed

    Matrix form:
        |  cos(θ)  0  sin(θ) |
        |    0     1    0    |
        | -sin(θ)  0  cos(θ) |

    Args:
        theta: Rotation angle in radians

    Returns:
        3×3 numpy rotation matrix
    """
    # TODO: Implement rotation matrix around Y-axis (Nazar)
    raise NotImplementedError("rotation_y: Nazar to implement")


def rotation_z(theta: float) -> NDArray[np.floating]:
    """
    Create 3×3 rotation matrix around Z-axis.

    Rotates by angle theta (radians) using the right-hand rule:
        - Positive theta rotates X toward Y
        - The Z-axis remains fixed

    Matrix form:
        | cos(θ)  -sin(θ)  0 |
        | sin(θ)   cos(θ)  0 |
        |   0        0     1 |

    Args:
        theta: Rotation angle in radians

    Returns:
        3×3 numpy rotation matrix
    """
    # TODO: Implement rotation matrix around Z-axis (Nazar)
    raise NotImplementedError("rotation_z: Nazar to implement")


def rotation_axis_angle(axis: Vector3, theta: float) -> NDArray[np.floating]:
    """
    Create 3×3 rotation matrix for rotation around arbitrary axis.

    Uses Rodrigues' rotation formula:
        R = I + sin(θ)·K + (1 - cos(θ))·K²

    where K is the skew-symmetric matrix form of the axis:
        K = |  0   -az   ay |
            |  az   0   -ax |
            | -ay   ax   0  |

    This is useful for joints with arbitrary rotation axes.

    Args:
        axis: Unit vector defining rotation axis
        theta: Rotation angle in radians

    Returns:
        3×3 numpy rotation matrix
    """
    # TODO: Implement Rodrigues' formula (Nazar - optional enhancement)
    raise NotImplementedError("rotation_axis_angle: Nazar to implement (optional)")


def translation_matrix(tx: float, ty: float, tz: float) -> NDArray[np.floating]:
    """
    Create 4×4 homogeneous translation matrix.

    Matrix form:
        | 1  0  0  tx |
        | 0  1  0  ty |
        | 0  0  1  tz |
        | 0  0  0  1  |

    When applied to a homogeneous point [x, y, z, 1]^T:
        - Translates by (tx, ty, tz)
    When applied to a direction [x, y, z, 0]^T:
        - No effect (directions are translation-invariant)

    Args:
        tx, ty, tz: Translation components

    Returns:
        4×4 numpy translation matrix
    """
    # TODO: Implement translation matrix (Nazar)
    raise NotImplementedError("translation_matrix: Nazar to implement")


def homogeneous_transform(
    rotation: NDArray[np.floating],
    translation: Vector3 | tuple[float, float, float]
) -> NDArray[np.floating]:
    """
    Create 4×4 homogeneous transformation matrix from rotation and translation.

    Combines a 3×3 rotation matrix and translation vector into:
        | R11  R12  R13  tx |
        | R21  R22  R23  ty |
        | R31  R32  R33  tz |
        |  0    0    0   1  |

    This matrix transforms points as: p' = T · R · p
    which first rotates, then translates.

    Args:
        rotation: 3×3 rotation matrix
        translation: Translation as Vector3 or (tx, ty, tz) tuple

    Returns:
        4×4 numpy transformation matrix
    """
    # TODO: Implement homogeneous transform composition (Nazar)
    raise NotImplementedError("homogeneous_transform: Nazar to implement")


@dataclass
class Transform4x4:
    """
    High-level wrapper for 4×4 homogeneous transformation matrix.

    Provides convenient methods for:
        - Composing transforms (matrix multiplication)
        - Applying transforms to points and directions
        - Extracting rotation and translation components
        - Computing inverse transforms

    Each joint in the robot arm has an associated Transform4x4 that
    describes its local coordinate frame relative to its parent.

    Attributes:
        matrix: The underlying 4×4 numpy array
    """

    matrix: NDArray[np.floating]

    def __post_init__(self) -> None:
        """Validate matrix shape."""
        if self.matrix.shape != (4, 4):
            raise ValueError(f"Transform must be 4×4, got {self.matrix.shape}")

    @classmethod
    def identity(cls) -> "Transform4x4":
        """Create identity transform (no rotation, no translation)."""
        return cls(np.eye(4, dtype=np.float64))

    @classmethod
    def from_rotation_x(cls, theta: float) -> "Transform4x4":
        """Create transform with only X-axis rotation."""
        # TODO: Use rotation_x and embed in 4×4 form (Nazar)
        raise NotImplementedError("Transform4x4.from_rotation_x: Nazar to implement")

    @classmethod
    def from_rotation_y(cls, theta: float) -> "Transform4x4":
        """Create transform with only Y-axis rotation."""
        # TODO: Use rotation_y and embed in 4×4 form (Nazar)
        raise NotImplementedError("Transform4x4.from_rotation_y: Nazar to implement")

    @classmethod
    def from_rotation_z(cls, theta: float) -> "Transform4x4":
        """Create transform with only Z-axis rotation."""
        # TODO: Use rotation_z and embed in 4×4 form (Nazar)
        raise NotImplementedError("Transform4x4.from_rotation_z: Nazar to implement")

    @classmethod
    def from_translation(cls, tx: float, ty: float, tz: float) -> "Transform4x4":
        """Create pure translation transform."""
        # TODO: Use translation_matrix (Nazar)
        raise NotImplementedError("Transform4x4.from_translation: Nazar to implement")

    @classmethod
    def from_rotation_translation(
        cls,
        rotation: NDArray[np.floating],
        translation: Vector3
    ) -> "Transform4x4":
        """Create combined rotation + translation transform."""
        # TODO: Use homogeneous_transform (Nazar)
        raise NotImplementedError("Transform4x4.from_rotation_translation: Nazar to implement")

    def __matmul__(self, other: "Transform4x4") -> "Transform4x4":
        """
        Compose transforms: T_result = self @ other.

        Matrix multiplication is right-to-left:
            - 'other' transform is applied first
            - 'self' transform is applied second

        This is how we chain joint transforms:
            T_end = T_base @ T_joint1 @ T_joint2 @ ... @ T_effector
        """
        return Transform4x4(self.matrix @ other.matrix)

    def transform_point(self, point: Vector3) -> Vector3:
        """
        Apply transform to a 3D point.

        Converts to homogeneous coordinates, multiplies, then converts back.
        Points are affected by both rotation and translation.

        Args:
            point: 3D point to transform

        Returns:
            Transformed 3D point
        """
        # TODO: Implement point transformation (Nazar)
        raise NotImplementedError("Transform4x4.transform_point: Nazar to implement")

    def transform_direction(self, direction: Vector3) -> Vector3:
        """
        Apply transform to a 3D direction vector.

        Directions use w=0, so translation has no effect.
        Only the rotation component is applied.

        Args:
            direction: 3D direction to transform

        Returns:
            Transformed 3D direction
        """
        # TODO: Implement direction transformation (Nazar)
        raise NotImplementedError("Transform4x4.transform_direction: Nazar to implement")

    @property
    def rotation(self) -> NDArray[np.floating]:
        """Extract the 3×3 rotation component."""
        return self.matrix[:3, :3].copy()

    @property
    def translation(self) -> Vector3:
        """Extract the translation component as Vector3."""
        return Vector3(
            float(self.matrix[0, 3]),
            float(self.matrix[1, 3]),
            float(self.matrix[2, 3])
        )

    def inverse(self) -> "Transform4x4":
        """
        Compute the inverse transform.

        For a rigid transform (rotation + translation):
            T^(-1) = | R^T   -R^T·t |
                     |  0       1   |

        We use the property that R^(-1) = R^T for rotation matrices.

        Returns:
            Inverse transform such that T @ T.inverse() = Identity
        """
        # TODO: Implement efficient inverse using R^T property (Nazar)
        raise NotImplementedError("Transform4x4.inverse: Nazar to implement")
