"""
Inverse Kinematics Solver using Jacobian-based Methods.

Solves the IK problem: given a target end-effector position, find joint
angles that achieve it. Uses Damped Least Squares (DLS) method with
SVD-based pseudoinverse for numerical stability.

Mathematical Foundation:
    The Jacobian J relates infinitesimal joint angle changes to end-effector
    velocity:
        Δp ≈ J · Δθ

    To find Δθ given desired Δp, we need J^(-1). Since J is typically not
    square (more joints than 3 spatial dimensions), we use the pseudoinverse:
        Δθ = J⁺ · Δp

    The Moore-Penrose pseudoinverse via SVD:
        J = U · Σ · V^T
        J⁺ = V · Σ⁺ · U^T

    For Damped Least Squares, we modify singular values:
        σᵢ⁺ = σᵢ / (σᵢ² + λ²)

    This prevents numerical blow-up when σᵢ → 0 (singularities).

Algorithm (iterative):
    1. Compute current end-effector position via FK
    2. Compute error: Δp = p_target - p_current
    3. Compute Jacobian J at current configuration
    4. Compute pseudoinverse J⁺ via SVD
    5. Update angles: θ ← θ + J⁺ · Δp
    6. Repeat until ||Δp|| < ε

Author: Nazar Pasichnyk

File Location: src/robot_arm/kinematics/inverse.py
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Tuple

import numpy as np
from numpy.typing import NDArray

from robot_arm.core.vectors import Vector3
from robot_arm.core.transforms import Transform4x4
from robot_arm.kinematics.forward import RobotArm


class IKStatus(Enum):
    """Status of IK solver result."""
    SUCCESS = auto()           # Converged within tolerance
    MAX_ITERATIONS = auto()    # Reached iteration limit
    UNREACHABLE = auto()       # Target outside workspace
    SINGULARITY = auto()       # Hit singular configuration


@dataclass
class IKResult:
    """
    Result of an inverse kinematics solve.

    Attributes:
        angles: Solution joint angles (may be partial solution if not converged)
        status: Solver status (success, max iterations, etc.)
        iterations: Number of iterations performed
        final_error: Distance from end-effector to target
        error_history: Error at each iteration (for debugging/visualization)
    """

    angles: List[float]
    status: IKStatus
    iterations: int
    final_error: float
    error_history: List[float] = field(default_factory=list)

    @property
    def success(self) -> bool:
        """Check if solver converged successfully."""
        return self.status == IKStatus.SUCCESS


class InverseKinematicsSolver(ABC):
    """
    Abstract base class for IK solvers.

    Different solving strategies can be implemented:
        - Jacobian Transpose (simple but slow)
        - Pseudoinverse (fast but unstable near singularities)
        - Damped Least Squares (robust, our chosen method)
        - CCD (heuristic, one joint at a time)
    """

    @abstractmethod
    def solve(
        self,
        arm: RobotArm,
        target: Vector3,
        initial_angles: List[float] | None = None,
    ) -> IKResult:
        """
        Solve IK to reach the target position.

        Args:
            arm: Robot arm model
            target: Desired end-effector position
            initial_angles: Starting configuration (uses current if None)

        Returns:
            IKResult with solution angles and convergence info
        """
        pass


@dataclass
class DampedLeastSquaresSolver(InverseKinematicsSolver):
    """
    Damped Least Squares (DLS) IK solver with SVD.

    This is our primary solver, chosen because it:
        - Demonstrates key linear algebra concepts (Jacobian, SVD, pseudoinverse)
        - Handles singularities gracefully via damping
        - Converges reliably for reachable targets

    Attributes:
        damping: Damping factor λ (larger = more stable but slower)
        tolerance: Convergence threshold for position error
        max_iterations: Maximum iterations before giving up
        step_scale: Scale factor for angle updates (< 1 for stability)
        min_singular_value: Threshold for detecting singularities
    """

    damping: float = 0.5
    tolerance: float = 1e-3
    max_iterations: int = 100
    step_scale: float = 1.0
    min_singular_value: float = 1e-4

    def solve(
        self,
        arm: RobotArm,
        target: Vector3,
        initial_angles: List[float] | None = None,
    ) -> IKResult:
        """
        Solve IK using Damped Least Squares.

        Implementation follows the algorithm in the interim report:
            1. Forward kinematics to get current position
            2. Compute position error
            3. Build Jacobian matrix
            4. Compute damped pseudoinverse via SVD
            5. Update joint angles
            6. Repeat until convergence

        Args:
            arm: Robot arm model
            target: Target end-effector position
            initial_angles: Starting angles (uses arm's current angles if None)

        Returns:
            IKResult with solution and convergence information
        """
        # Initialize angles
        if initial_angles is not None:
            angles = list(initial_angles)
        else:
            angles = arm.get_angles()
        
        error_history: List[float] = []
        
        for iteration in range(self.max_iterations):
            arm.set_angles(angles)
            current_pos = arm.get_end_effector_position()
            error_vec = target - current_pos
            error = error_vec.magnitude
            error_history.append(error)
            
            if error < self.tolerance:
                return IKResult(
                    angles=angles,
                    status=IKStatus.SUCCESS,
                    iterations=iteration + 1,
                    final_error=error,
                    error_history=error_history
                )
            
            J = self._compute_jacobian(arm)
            delta_theta = self._compute_step(J, error_vec.to_array())
            angles = [a + d for a, d in zip(angles, delta_theta)]
        
        # Max iterations reached
        arm.set_angles(angles)
        final_pos = arm.get_end_effector_position()
        final_error = (target - final_pos).magnitude
        
        return IKResult(
            angles=angles,
            status=IKStatus.MAX_ITERATIONS,
            iterations=self.max_iterations,
            final_error=final_error,
            error_history=error_history
        )

    def _compute_jacobian(self, arm: RobotArm) -> NDArray[np.floating]:
        """
        Compute the Jacobian matrix at the current configuration.

        The Jacobian is a 3×n matrix where n is the number of joints.
        Each column j represents how the end-effector position changes
        with respect to joint angle θⱼ:

            Jⱼ = axisⱼ × (p_end - pⱼ)

        where:
            - axisⱼ is the rotation axis of joint j (in world frame)
            - pⱼ is the position of joint j
            - p_end is the end-effector position

        Args:
            arm: Robot arm at current configuration

        Returns:
            3×n Jacobian matrix
        """
        n = arm.num_joints
        J = np.zeros((3, n), dtype=np.float64)
        
        p_end = arm.get_end_effector_position()
        
        # Build accumulated transform to get joint positions and world-frame axes
        accumulated = Transform4x4.from_translation(
            arm.base_position.x, arm.base_position.y, arm.base_position.z
        )
        
        for j, joint in enumerate(arm.joints):
            # Get joint axis in world frame
            local_axis = joint.axis
            world_axis = accumulated.transform_direction(local_axis)
            
            # Get position of this joint (before translation)
            p_j = accumulated.translation
            
            # Compute column: axis × (p_end - p_j)
            r = p_end - p_j
            cross = world_axis.cross(r)
            J[:, j] = cross.to_array()
            
            # Update accumulated transform
            accumulated = accumulated @ joint.get_local_transform()
        
        return J

    def _compute_step(
        self,
        jacobian: NDArray[np.floating],
        error: NDArray[np.floating]
    ) -> NDArray[np.floating]:
        """
        Compute angle update using damped pseudoinverse.

        Uses SVD for numerical stability:
            J = U · Σ · Vᵀ
            J⁺ = V · Σ⁺ · Uᵀ

        where Σ⁺ applies damped inverse to singular values:
            σᵢ⁺ = σᵢ / (σᵢ² + λ²)

        Args:
            jacobian: 3×n Jacobian matrix
            error: 3D position error vector

        Returns:
            n-dimensional angle update vector
        """
        U, S, Vt = np.linalg.svd(jacobian, full_matrices=False)
        
        # Compute damped singular values
        lambda_sq = self.damping ** 2
        S_damped = S / (S ** 2 + lambda_sq)
        
        # Compute damped pseudoinverse: J⁺ = V · Σ⁺ · Uᵀ
        # Note: Vt from SVD is already V transposed, so V = Vt.T
        J_pinv = Vt.T @ np.diag(S_damped) @ U.T
        
        # Compute angle update
        delta_theta = J_pinv @ error * self.step_scale
        
        return delta_theta

    def _check_singularity(self, singular_values: NDArray[np.floating]) -> bool:
        """Check if near a singular configuration."""
        return np.any(singular_values < self.min_singular_value)


@dataclass
class JacobianTransposeSolver(InverseKinematicsSolver):
    """
    Simple Jacobian Transpose IK solver.

    Uses Δθ = α · Jᵀ · Δp instead of pseudoinverse.
    Simpler but slower convergence. Useful as a fallback.

    Attributes:
        step_size: Learning rate α
        tolerance: Convergence threshold
        max_iterations: Maximum iterations
    """

    step_size: float = 0.1
    tolerance: float = 1e-3
    max_iterations: int = 500

    def solve(
        self,
        arm: RobotArm,
        target: Vector3,
        initial_angles: List[float] | None = None,
    ) -> IKResult:
        """
        Solve IK using Jacobian Transpose method.

        Simpler than pseudoinverse but requires more iterations.
        """
        # TODO: Implement as alternative solver (Nazar - optional)
        raise NotImplementedError("JacobianTransposeSolver: optional implementation")


def compute_workspace_bounds(arm: RobotArm) -> Tuple[float, float]:
    """
    Estimate reachable workspace bounds.

    For a serial arm, the workspace is roughly a sphere:
        - Outer radius: sum of all link lengths
        - Inner radius: difference between longest and sum of others

    Args:
        arm: Robot arm model

    Returns:
        (min_reach, max_reach) tuple
    """
    total = arm.total_reach
    if arm.num_joints == 0:
        return (0.0, 0.0)

    max_link = max(j.link_length for j in arm.joints)
    others = total - max_link
    min_reach = max(0.0, max_link - others)

    return (min_reach, total)


def is_target_reachable(arm: RobotArm, target: Vector3) -> bool:
    """
    Quick check if target is within workspace.

    Args:
        arm: Robot arm model
        target: Target position

    Returns:
        True if target might be reachable (false positives possible)
    """
    distance = (target - arm.base_position).magnitude
    min_reach, max_reach = compute_workspace_bounds(arm)
    return min_reach <= distance <= max_reach
