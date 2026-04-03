"""
Test Suite for Kinematics Module.

Tests for:
    - Forward kinematics
    - Inverse kinematics
    - Jacobian computation
    - Convergence properties

Author: Mykola Balyk

File Location: tests/test_kinematics.py
"""

import math
import pytest
import numpy as np

from robot_arm.core.vectors import Vector3
from robot_arm.kinematics.forward import RobotArm, Joint, JointType, compute_forward_kinematics
from robot_arm.kinematics.inverse import (
    DampedLeastSquaresSolver,
    IKStatus,
    compute_workspace_bounds,
    is_target_reachable,
)


class TestRobotArm:
    """Tests for RobotArm class."""

    def test_create_3dof_arm(self):
        """Test creating a 3-DOF arm."""
        arm = RobotArm.create_3dof_arm([2.0, 1.5, 1.0])
        assert arm.num_joints == 3
        assert arm.total_reach == 4.5

    def test_set_get_angles(self):
        """Test setting and getting joint angles."""
        arm = RobotArm.create_3dof_arm()
        arm.set_angles([0.5, -0.3, 0.8])
        angles = arm.get_angles()
        assert abs(angles[0] - 0.5) < 1e-10
        assert abs(angles[1] - (-0.3)) < 1e-10
        assert abs(angles[2] - 0.8) < 1e-10

    def test_angle_clamping(self):
        """Test that angles are clamped to limits."""
        arm = RobotArm.create_3dof_arm()
        arm.set_angles([10.0, -10.0, 0.0])  # Beyond limits
        angles = arm.get_angles()
        # Should be clamped to [-π, π]
        assert angles[0] <= math.pi
        assert angles[1] >= -math.pi

    def test_invalid_angles_count_raises(self):
        """Test that wrong number of angles raises."""
        arm = RobotArm.create_3dof_arm()
        with pytest.raises(ValueError):
            arm.set_angles([0.5, 0.5])  # Only 2 angles for 3 joints


class TestForwardKinematics:
    """Tests for forward kinematics computation."""

    @pytest.mark.skip(reason="Waiting for Nazar's implementation")
    def test_fk_zero_angles(self):
        """Test FK with all zero angles."""
        arm = RobotArm.create_3dof_arm([2.0, 1.5, 1.0])
        arm.set_angles([0.0, 0.0, 0.0])

        # With zero angles, arm should extend along initial axis
        end_pos = arm.get_end_effector_position()

        # Total reach along X-axis (for Z-axis rotation joints)
        assert abs(end_pos.x - 4.5) < 1e-6
        assert abs(end_pos.y) < 1e-6
        assert abs(end_pos.z) < 1e-6

    @pytest.mark.skip(reason="Waiting for Nazar's implementation")
    def test_fk_90_degree(self):
        """Test FK with 90° first joint."""
        arm = RobotArm.create_3dof_arm([2.0, 1.5, 1.0])
        arm.set_angles([math.pi / 2, 0.0, 0.0])

        end_pos = arm.get_end_effector_position()

        # Should now extend along Y-axis
        assert abs(end_pos.x) < 1e-6
        assert abs(end_pos.y - 4.5) < 1e-6

    @pytest.mark.skip(reason="Waiting for Nazar's implementation")
    def test_fk_folded(self):
        """Test FK with folded arm configuration."""
        arm = RobotArm.create_3dof_arm([2.0, 2.0, 2.0])
        arm.set_angles([0.0, math.pi, 0.0])

        # Second link should fold back
        end_pos = arm.get_end_effector_position()

        # With equal link lengths and 180° fold: 2 - 2 + 2 = 2
        assert abs(end_pos.x - 2.0) < 1e-6

    @pytest.mark.skip(reason="Waiting for Nazar's implementation")
    def test_joint_positions(self):
        """Test getting all joint positions."""
        arm = RobotArm.create_3dof_arm([2.0, 1.0, 0.5])
        arm.set_angles([0.0, 0.0, 0.0])

        positions = arm.get_joint_positions()

        assert len(positions) == 3
        # First joint at base + first link
        assert abs(positions[0].x - 2.0) < 1e-6
        # Second joint
        assert abs(positions[1].x - 3.0) < 1e-6
        # End effector
        assert abs(positions[2].x - 3.5) < 1e-6


class TestInverseKinematics:
    """Tests for inverse kinematics solver."""

    @pytest.mark.skip(reason="Waiting for Nazar's implementation")
    def test_ik_reachable_target(self):
        """Test IK for a reachable target."""
        arm = RobotArm.create_3dof_arm([2.0, 1.5, 1.0])
        solver = DampedLeastSquaresSolver(
            tolerance=0.01,
            max_iterations=100,
        )

        target = Vector3(3.0, 2.0, 0.0)
        result = solver.solve(arm, target)

        assert result.status == IKStatus.SUCCESS
        assert result.final_error < 0.01

    @pytest.mark.skip(reason="Waiting for Nazar's implementation")
    def test_ik_unreachable_target(self):
        """Test IK for unreachable target."""
        arm = RobotArm.create_3dof_arm([2.0, 1.5, 1.0])  # Total reach = 4.5
        solver = DampedLeastSquaresSolver(max_iterations=50)

        target = Vector3(10.0, 0.0, 0.0)  # Beyond reach
        result = solver.solve(arm, target)

        # Should not succeed, but should not crash
        assert result.status in [IKStatus.MAX_ITERATIONS, IKStatus.UNREACHABLE]

    @pytest.mark.skip(reason="Waiting for Nazar's implementation")
    def test_ik_error_decreases(self):
        """Test that error decreases monotonically (usually)."""
        arm = RobotArm.create_3dof_arm()
        solver = DampedLeastSquaresSolver(max_iterations=50)

        target = Vector3(3.0, 1.0, 0.0)
        result = solver.solve(arm, target)

        # Check error generally decreases (allow small fluctuations)
        if len(result.error_history) > 5:
            # Final error should be less than initial
            assert result.error_history[-1] < result.error_history[0]

    @pytest.mark.skip(reason="Waiting for Nazar's implementation")
    def test_ik_multiple_solutions(self):
        """Test that IK finds a valid solution from different starts."""
        arm = RobotArm.create_3dof_arm()
        solver = DampedLeastSquaresSolver(tolerance=0.05)
        target = Vector3(2.0, 2.0, 0.0)

        # Try from different initial configurations
        initial_configs = [
            [0.0, 0.0, 0.0],
            [0.5, 0.5, 0.5],
            [-0.5, 0.5, -0.5],
        ]

        for initial in initial_configs:
            result = solver.solve(arm, target, initial_angles=initial)
            # All should eventually reach the target (or get close)
            assert result.final_error < 0.5  # Reasonably close


class TestWorkspaceAnalysis:
    """Tests for workspace analysis functions."""

    def test_workspace_bounds(self):
        """Test workspace bounds computation."""
        arm = RobotArm.create_3dof_arm([2.0, 1.5, 1.0])
        min_reach, max_reach = compute_workspace_bounds(arm)

        assert max_reach == 4.5  # Sum of all links
        # Min reach with these lengths should be 0 (arm can fold completely)
        assert min_reach >= 0

    def test_target_reachability(self):
        """Test target reachability check."""
        arm = RobotArm.create_3dof_arm([2.0, 1.5, 1.0])

        # Inside workspace
        assert is_target_reachable(arm, Vector3(3.0, 1.0, 0.0))

        # Outside workspace
        assert not is_target_reachable(arm, Vector3(10.0, 0.0, 0.0))


class TestJacobianComputation:
    """Tests for Jacobian computation."""

    @pytest.mark.skip(reason="Waiting for Nazar's implementation")
    def test_jacobian_shape(self):
        """Test Jacobian has correct shape."""
        arm = RobotArm.create_3dof_arm()
        solver = DampedLeastSquaresSolver()

        J = solver._compute_jacobian(arm)

        # Should be 3×n (3 spatial dimensions, n joints)
        assert J.shape == (3, arm.num_joints)

    @pytest.mark.skip(reason="Waiting for Nazar's implementation")
    def test_jacobian_numerical_verification(self):
        """Verify Jacobian against numerical differentiation."""
        arm = RobotArm.create_3dof_arm()
        arm.set_angles([0.3, 0.5, 0.2])
        solver = DampedLeastSquaresSolver()

        J_analytical = solver._compute_jacobian(arm)

        # Numerical Jacobian
        eps = 1e-6
        J_numerical = np.zeros((3, arm.num_joints))

        for j in range(arm.num_joints):
            angles = arm.get_angles()

            angles_plus = angles.copy()
            angles_plus[j] += eps
            arm.set_angles(angles_plus)
            p_plus = arm.get_end_effector_position()

            angles_minus = angles.copy()
            angles_minus[j] -= eps
            arm.set_angles(angles_minus)
            p_minus = arm.get_end_effector_position()

            J_numerical[:, j] = [
                (p_plus.x - p_minus.x) / (2 * eps),
                (p_plus.y - p_minus.y) / (2 * eps),
                (p_plus.z - p_minus.z) / (2 * eps),
            ]

            arm.set_angles(angles)  # Restore

        np.testing.assert_array_almost_equal(J_analytical, J_numerical, decimal=4)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
