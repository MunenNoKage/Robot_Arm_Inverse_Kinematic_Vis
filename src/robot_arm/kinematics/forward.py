"""
Forward Kinematics for Multi-Joint Robot Arm.

Computes the end-effector position and orientation from joint angles
using a chain of homogeneous transformation matrices.

Mathematical Model:
    For a robot arm with n joints, each joint i has:
        - θᵢ: joint angle (variable)
        - tᵢ: link length (fixed offset to next joint)
        - axisᵢ: rotation axis (typically Z for revolute joints)

    The transformation from joint i to joint i+1:
        Tᵢ(θᵢ) = Rᵢ(θᵢ) · Translate(tᵢ)

    The end-effector pose in world coordinates:
        T_end = T₁(θ₁) · T₂(θ₂) · ... · Tₙ(θₙ)

    The position is extracted from the last column:
        p_end = T_end · [0, 0, 0, 1]ᵀ

Author: Nazar Pasichnyk

File Location: src/robot_arm/kinematics/forward.py
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Sequence

import numpy as np
from numpy.typing import NDArray

from robot_arm.core.vectors import Vector3
from robot_arm.core.transforms import Transform4x4, rotation_x, rotation_y, rotation_z


class JointType(Enum):
    """
    Type of robot joint.

    Revolute joints rotate around an axis.
    Prismatic joints translate along an axis (not implemented in this project).
    """
    REVOLUTE_X = auto()  # Rotates around X-axis
    REVOLUTE_Y = auto()  # Rotates around Y-axis
    REVOLUTE_Z = auto()  # Rotates around Z-axis (most common for planar arms)


@dataclass
class Joint:
    """
    Represents a single joint in the robot arm.

    A joint connects two links and allows relative motion between them.
    For revolute joints, the joint angle θ controls rotation around the
    joint's axis.

    Attributes:
        name: Human-readable identifier
        joint_type: Type of joint (determines rotation axis)
        link_length: Distance to the next joint along the link
        angle: Current joint angle in radians
        min_angle: Minimum allowed angle (joint limit)
        max_angle: Maximum allowed angle (joint limit)

    Example:
        >>> shoulder = Joint("shoulder", JointType.REVOLUTE_Z, link_length=2.0)
        >>> shoulder.angle = math.pi / 4  # 45 degrees
    """

    name: str
    joint_type: JointType
    link_length: float
    angle: float = 0.0
    min_angle: float = -math.pi
    max_angle: float = math.pi

    @property
    def axis(self) -> Vector3:
        """Get the rotation axis based on joint type."""
        axis_map = {
            JointType.REVOLUTE_X: Vector3.unit_x(),
            JointType.REVOLUTE_Y: Vector3.unit_y(),
            JointType.REVOLUTE_Z: Vector3.unit_z(),
        }
        return axis_map[self.joint_type]

    def clamp_angle(self, angle: float) -> float:
        """Clamp angle to joint limits."""
        return max(self.min_angle, min(self.max_angle, angle))

    def get_local_transform(self) -> Transform4x4:
        """
        Compute the local transformation for this joint.

        The transform consists of:
            1. Rotation around the joint axis by current angle
            2. Translation along the link to the next joint

        Returns:
            4×4 transformation matrix
        """
        # TODO: Implement local transform computation (Nazar)
        # Steps:
        # 1. Create rotation matrix based on joint_type and angle
        # 2. Create translation along X/Y/Z based on link geometry
        # 3. Combine into homogeneous transform
        raise NotImplementedError("Joint.get_local_transform: Nazar to implement")


@dataclass
class RobotArm:
    """
    Multi-joint robot arm model.

    The robot arm consists of a chain of joints connected by rigid links.
    The base is fixed at the origin, and the end-effector (gripper) position
    is computed via forward kinematics.

    Attributes:
        joints: List of joints from base to end-effector
        base_position: Position of the arm's base in world coordinates

    Example:
        >>> arm = RobotArm.create_3dof_arm()
        >>> arm.set_angles([0.5, -0.3, 0.8])
        >>> end_pos = arm.get_end_effector_position()
    """

    joints: List[Joint] = field(default_factory=list)
    base_position: Vector3 = field(default_factory=Vector3.zero)

    @classmethod
    def create_3dof_arm(
        cls,
        link_lengths: Sequence[float] = (2.0, 1.5, 1.0)
    ) -> "RobotArm":
        """
        Create a simple 3-DOF (degrees of freedom) planar arm.

        All joints rotate around the Z-axis, creating motion in the XY plane.
        This is the configuration used in our project visualization.

        Args:
            link_lengths: Lengths of each link (shoulder to elbow, etc.)

        Returns:
            Configured RobotArm instance
        """
        joints = [
            Joint("shoulder", JointType.REVOLUTE_Z, link_lengths[0]),
            Joint("elbow", JointType.REVOLUTE_Z, link_lengths[1]),
            Joint("wrist", JointType.REVOLUTE_Z, link_lengths[2]),
        ]
        return cls(joints=joints)

    @classmethod
    def create_6dof_arm(
        cls,
        link_lengths: Sequence[float] = (1.0, 2.0, 1.5, 0.5, 0.5, 0.3)
    ) -> "RobotArm":
        """
        Create a 6-DOF spatial arm (more complex configuration).

        Alternates between different rotation axes for full 3D motion.

        Args:
            link_lengths: Lengths of each link

        Returns:
            Configured RobotArm instance
        """
        joints = [
            Joint("base_yaw", JointType.REVOLUTE_Z, link_lengths[0]),
            Joint("shoulder_pitch", JointType.REVOLUTE_Y, link_lengths[1]),
            Joint("shoulder_roll", JointType.REVOLUTE_X, link_lengths[2]),
            Joint("elbow_pitch", JointType.REVOLUTE_Y, link_lengths[3]),
            Joint("wrist_roll", JointType.REVOLUTE_X, link_lengths[4]),
            Joint("wrist_pitch", JointType.REVOLUTE_Y, link_lengths[5]),
        ]
        return cls(joints=joints)

    @property
    def num_joints(self) -> int:
        """Number of joints (degrees of freedom)."""
        return len(self.joints)

    @property
    def total_reach(self) -> float:
        """Maximum possible reach (sum of all link lengths)."""
        return sum(joint.link_length for joint in self.joints)

    def get_angles(self) -> List[float]:
        """Get current angles for all joints."""
        return [joint.angle for joint in self.joints]

    def set_angles(self, angles: Sequence[float]) -> None:
        """
        Set angles for all joints (with clamping to limits).

        Args:
            angles: Sequence of angles, one per joint

        Raises:
            ValueError: If number of angles doesn't match number of joints
        """
        if len(angles) != self.num_joints:
            raise ValueError(
                f"Expected {self.num_joints} angles, got {len(angles)}"
            )
        for joint, angle in zip(self.joints, angles):
            joint.angle = joint.clamp_angle(angle)

    def compute_joint_transforms(self) -> List[Transform4x4]:
        """
        Compute cumulative transforms for each joint.

        Returns transforms from world origin to each joint position.
        Used for both visualization (drawing links) and Jacobian computation.

        Returns:
            List of transforms, one per joint, in world coordinates
        """
        # TODO: Implement cumulative transform computation (Nazar)
        # Start with base transform, then multiply each joint's local transform
        # Result: [T_base, T_base·T_1, T_base·T_1·T_2, ...]
        raise NotImplementedError("RobotArm.compute_joint_transforms: Nazar to implement")

    def get_joint_positions(self) -> List[Vector3]:
        """
        Get world positions of all joints.

        Returns:
            List of 3D positions for each joint, starting from base
        """
        # TODO: Implement using compute_joint_transforms (Nazar)
        raise NotImplementedError("RobotArm.get_joint_positions: Nazar to implement")

    def get_end_effector_position(self) -> Vector3:
        """
        Compute the current end-effector (gripper) position.

        This is the primary output of forward kinematics:
            p_end = T_total · [0, 0, 0, 1]ᵀ

        Returns:
            3D position of the end-effector in world coordinates
        """
        # TODO: Implement end-effector position (Nazar)
        raise NotImplementedError("RobotArm.get_end_effector_position: Nazar to implement")

    def get_end_effector_transform(self) -> Transform4x4:
        """
        Get full transform (position + orientation) of end-effector.

        Returns:
            4×4 transformation matrix for the end-effector frame
        """
        # TODO: Implement full transform chain (Nazar)
        raise NotImplementedError("RobotArm.get_end_effector_transform: Nazar to implement")


def compute_forward_kinematics(
    arm: RobotArm,
    angles: Sequence[float] | None = None
) -> Vector3:
    """
    Convenience function to compute FK for given angles.

    Args:
        arm: RobotArm instance
        angles: Optional angles to set (uses current if None)

    Returns:
        End-effector position
    """
    if angles is not None:
        arm.set_angles(angles)
    return arm.get_end_effector_position()
