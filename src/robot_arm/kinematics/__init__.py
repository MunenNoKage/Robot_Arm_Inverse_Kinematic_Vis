"""
Kinematics subpackage for robot arm simulation.

This module provides:
    - forward: Forward kinematics computation
    - inverse: Inverse kinematics solver (Jacobian-based)
"""

from robot_arm.kinematics.forward import (
    RobotArm,
    Joint,
    JointType,
    compute_forward_kinematics,
)
from robot_arm.kinematics.inverse import (
    InverseKinematicsSolver,
    IKResult,
    IKStatus,
    DampedLeastSquaresSolver,
)

__all__ = [
    "RobotArm",
    "Joint",
    "JointType",
    "compute_forward_kinematics",
    "InverseKinematicsSolver",
    "IKResult",
    "IKStatus",
    "DampedLeastSquaresSolver",
]
