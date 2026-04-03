"""
Test Suite for Robot Arm Simulation.

Comprehensive tests for:
    - Vector operations
    - Transformation matrices
    - Forward kinematics
    - Inverse kinematics
    - Projection pipeline

Author: Mykola Balyk

File Location: tests/test_core.py
"""

import math
import pytest
import numpy as np

from robot_arm.core.vectors import Vector3, Vector4
from robot_arm.core.transforms import (
    rotation_x, rotation_y, rotation_z,
    translation_matrix, homogeneous_transform, Transform4x4
)


class TestVector3:
    """Tests for Vector3 class."""

    def test_creation(self):
        """Test vector creation."""
        v = Vector3(1.0, 2.0, 3.0)
        assert v.x == 1.0
        assert v.y == 2.0
        assert v.z == 3.0

    def test_zero_vector(self):
        """Test zero vector factory."""
        v = Vector3.zero()
        assert v.x == 0.0
        assert v.y == 0.0
        assert v.z == 0.0

    def test_unit_vectors(self):
        """Test unit vector factories."""
        assert Vector3.unit_x() == Vector3(1.0, 0.0, 0.0)
        assert Vector3.unit_y() == Vector3(0.0, 1.0, 0.0)
        assert Vector3.unit_z() == Vector3(0.0, 0.0, 1.0)

    def test_addition(self):
        """Test vector addition."""
        v1 = Vector3(1.0, 2.0, 3.0)
        v2 = Vector3(4.0, 5.0, 6.0)
        result = v1 + v2
        assert result == Vector3(5.0, 7.0, 9.0)

    def test_subtraction(self):
        """Test vector subtraction."""
        v1 = Vector3(4.0, 5.0, 6.0)
        v2 = Vector3(1.0, 2.0, 3.0)
        result = v1 - v2
        assert result == Vector3(3.0, 3.0, 3.0)

    def test_scalar_multiplication(self):
        """Test scalar multiplication."""
        v = Vector3(1.0, 2.0, 3.0)
        assert v * 2.0 == Vector3(2.0, 4.0, 6.0)
        assert 2.0 * v == Vector3(2.0, 4.0, 6.0)

    def test_dot_product(self):
        """Test dot product computation."""
        v1 = Vector3(1.0, 2.0, 3.0)
        v2 = Vector3(4.0, 5.0, 6.0)
        assert v1.dot(v2) == 32.0  # 1*4 + 2*5 + 3*6

    def test_cross_product(self):
        """Test cross product computation."""
        # X × Y = Z
        assert Vector3.unit_x().cross(Vector3.unit_y()) == Vector3.unit_z()
        # Y × Z = X
        assert Vector3.unit_y().cross(Vector3.unit_z()) == Vector3.unit_x()
        # Z × X = Y
        assert Vector3.unit_z().cross(Vector3.unit_x()) == Vector3.unit_y()

    def test_magnitude(self):
        """Test magnitude computation."""
        v = Vector3(3.0, 4.0, 0.0)
        assert v.magnitude == 5.0

    def test_normalized(self):
        """Test vector normalization."""
        v = Vector3(3.0, 4.0, 0.0)
        n = v.normalized()
        assert abs(n.magnitude - 1.0) < 1e-10
        assert abs(n.x - 0.6) < 1e-10
        assert abs(n.y - 0.8) < 1e-10

    def test_normalize_zero_raises(self):
        """Test that normalizing zero vector raises."""
        with pytest.raises(ValueError):
            Vector3.zero().normalized()

    def test_homogeneous_conversion(self):
        """Test conversion to homogeneous coordinates."""
        v = Vector3(1.0, 2.0, 3.0)
        h = v.to_homogeneous()
        assert h.x == 1.0
        assert h.y == 2.0
        assert h.z == 3.0
        assert h.w == 1.0


class TestVector4:
    """Tests for Vector4 class."""

    def test_from_point(self):
        """Test creating homogeneous point."""
        p = Vector3(1.0, 2.0, 3.0)
        h = Vector4.from_point(p)
        assert h.w == 1.0

    def test_from_direction(self):
        """Test creating homogeneous direction."""
        d = Vector3(1.0, 0.0, 0.0)
        h = Vector4.from_direction(d)
        assert h.w == 0.0

    def test_to_vector3_with_divide(self):
        """Test perspective divide when converting back."""
        h = Vector4(2.0, 4.0, 6.0, 2.0)
        v = h.to_vector3()
        assert v == Vector3(1.0, 2.0, 3.0)


class TestRotationMatrices:
    """Tests for rotation matrix functions."""

    @pytest.mark.skip(reason="Waiting for Nazar's implementation")
    def test_rotation_x_90(self):
        """Test 90° rotation around X-axis."""
        R = rotation_x(math.pi / 2)

        # Y-axis should become Z-axis
        y = np.array([0, 1, 0])
        result = R @ y
        expected = np.array([0, 0, 1])
        np.testing.assert_array_almost_equal(result, expected)

    @pytest.mark.skip(reason="Waiting for Nazar's implementation")
    def test_rotation_y_90(self):
        """Test 90° rotation around Y-axis."""
        R = rotation_y(math.pi / 2)

        # Z-axis should become X-axis
        z = np.array([0, 0, 1])
        result = R @ z
        expected = np.array([1, 0, 0])
        np.testing.assert_array_almost_equal(result, expected)

    @pytest.mark.skip(reason="Waiting for Nazar's implementation")
    def test_rotation_z_90(self):
        """Test 90° rotation around Z-axis."""
        R = rotation_z(math.pi / 2)

        # X-axis should become Y-axis
        x = np.array([1, 0, 0])
        result = R @ x
        expected = np.array([0, 1, 0])
        np.testing.assert_array_almost_equal(result, expected)

    @pytest.mark.skip(reason="Waiting for Nazar's implementation")
    def test_rotation_orthogonality(self):
        """Test that rotation matrices are orthogonal."""
        for theta in [0, math.pi/4, math.pi/2, math.pi]:
            for rotation_fn in [rotation_x, rotation_y, rotation_z]:
                R = rotation_fn(theta)
                # R^T * R should be identity
                np.testing.assert_array_almost_equal(R.T @ R, np.eye(3))
                # det(R) should be 1
                assert abs(np.linalg.det(R) - 1.0) < 1e-10

    @pytest.mark.skip(reason="Waiting for Nazar's implementation")
    def test_rotation_inverse_is_transpose(self):
        """Test R^(-1) = R^T property."""
        R = rotation_x(0.5)
        R_inv = np.linalg.inv(R)
        np.testing.assert_array_almost_equal(R_inv, R.T)


class TestTranslationMatrix:
    """Tests for translation matrix function."""

    @pytest.mark.skip(reason="Waiting for Nazar's implementation")
    def test_translation_point(self):
        """Test translating a point."""
        T = translation_matrix(1, 2, 3)
        p = np.array([0, 0, 0, 1])  # Point at origin
        result = T @ p
        expected = np.array([1, 2, 3, 1])
        np.testing.assert_array_almost_equal(result, expected)

    @pytest.mark.skip(reason="Waiting for Nazar's implementation")
    def test_translation_direction_invariant(self):
        """Test that directions are not affected by translation."""
        T = translation_matrix(1, 2, 3)
        d = np.array([1, 0, 0, 0])  # Direction (w=0)
        result = T @ d
        expected = np.array([1, 0, 0, 0])  # Unchanged
        np.testing.assert_array_almost_equal(result, expected)


class TestTransform4x4:
    """Tests for Transform4x4 class."""

    def test_identity(self):
        """Test identity transform."""
        T = Transform4x4.identity()
        np.testing.assert_array_almost_equal(T.matrix, np.eye(4))

    @pytest.mark.skip(reason="Waiting for Nazar's implementation")
    def test_composition(self):
        """Test transform composition."""
        T1 = Transform4x4.from_translation(1, 0, 0)
        T2 = Transform4x4.from_translation(0, 1, 0)
        T_combined = T1 @ T2

        # Applying T_combined should translate by (1, 1, 0)
        p = Vector3(0, 0, 0)
        result = T_combined.transform_point(p)
        assert abs(result.x - 1.0) < 1e-10
        assert abs(result.y - 1.0) < 1e-10

    @pytest.mark.skip(reason="Waiting for Nazar's implementation")
    def test_inverse(self):
        """Test transform inverse."""
        T = Transform4x4.from_translation(1, 2, 3)
        T_inv = T.inverse()
        T_identity = T @ T_inv
        np.testing.assert_array_almost_equal(T_identity.matrix, np.eye(4))


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
