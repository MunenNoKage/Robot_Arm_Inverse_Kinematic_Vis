"""
Basic Example: Robot Arm Inverse Kinematics Demonstration.

This example shows how to:
1. Create a robot arm
2. Use forward kinematics to compute end-effector position
3. Use inverse kinematics to reach a target
4. Run the visualization

Run with: python example_basic.py

Authors: Roman Prokhorov, Nazar Pasichnyk, Mykola Balyk
"""

import math
import sys

from robot_arm.core.vectors import Vector3
from robot_arm.kinematics.forward import RobotArm
from robot_arm.kinematics.inverse import DampedLeastSquaresSolver, is_target_reachable


def demo_forward_kinematics():
    """Demonstrate forward kinematics computation."""
    print("=" * 60)
    print("Forward Kinematics Demo")
    print("=" * 60)

    # Create a 3-DOF robot arm
    arm = RobotArm.create_3dof_arm(link_lengths=[2.0, 1.5, 1.0])
    print(f"Created arm with {arm.num_joints} joints")
    print(f"Total reach: {arm.total_reach} units")

    # Set some joint angles
    angles = [math.radians(45), math.radians(-30), math.radians(20)]
    print(f"\nSetting angles to: {[math.degrees(a) for a in angles]}°")
    arm.set_angles(angles)

    try:
        # Compute end-effector position
        end_pos = arm.get_end_effector_position()
        print(f"End-effector position: ({end_pos.x:.3f}, {end_pos.y:.3f}, {end_pos.z:.3f})")

        # Get all joint positions
        joint_positions = arm.get_joint_positions()
        print("\nJoint positions:")
        for i, pos in enumerate(joint_positions):
            print(f"  Joint {i+1}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
    except NotImplementedError as e:
        print(f"\n[Not yet implemented: {e}]")
        print("Waiting for Nazar's forward kinematics implementation...")


def demo_inverse_kinematics():
    """Demonstrate inverse kinematics solving."""
    print("\n" + "=" * 60)
    print("Inverse Kinematics Demo")
    print("=" * 60)

    arm = RobotArm.create_3dof_arm([2.0, 1.5, 1.0])
    solver = DampedLeastSquaresSolver(
        damping=0.5,
        tolerance=0.01,
        max_iterations=100,
    )

    # Target position
    target = Vector3(3.0, 2.0, 0.0)
    print(f"Target position: ({target.x}, {target.y}, {target.z})")

    # Check reachability
    if is_target_reachable(arm, target):
        print("Target is within workspace!")
    else:
        print("Warning: Target may be outside workspace")

    try:
        # Solve IK
        result = solver.solve(arm, target)

        print(f"\nSolver result:")
        print(f"  Status: {result.status.name}")
        print(f"  Iterations: {result.iterations}")
        print(f"  Final error: {result.final_error:.6f}")
        print(f"  Solution angles: {[math.degrees(a) for a in result.angles]}°")

        if result.success:
            print("\n✓ Target reached successfully!")
        else:
            print("\n✗ Could not reach target exactly")
    except NotImplementedError as e:
        print(f"\n[Not yet implemented: {e}]")
        print("Waiting for Nazar's inverse kinematics implementation...")


def demo_visualization():
    """Launch the visualization."""
    print("\n" + "=" * 60)
    print("Launching Visualization")
    print("=" * 60)

    try:
        from robot_arm.visualization.app import RobotArmSimulator

        print("Starting simulator...")
        print("Controls:")
        print("  Left click: Move target")
        print("  Right drag: Rotate camera")
        print("  Scroll: Zoom")
        print("  Space: Pause/Resume")
        print("  R: Reset")
        print("  ESC: Quit")
        print()

        sim = RobotArmSimulator()
        sim.run()
    except ImportError as e:
        print(f"Could not import visualization: {e}")
        print("Make sure pygame is installed: pip install pygame")
    except Exception as e:
        print(f"Visualization error: {e}")


def main():
    """Run all demos."""
    print("\n" + "#" * 60)
    print("# Robot Arm Inverse Kinematics - Basic Examples")
    print("# Linear Algebra Course Project")
    print("#" * 60 + "\n")

    demo_forward_kinematics()
    demo_inverse_kinematics()

    # Ask if user wants to launch visualization
    print("\n" + "-" * 60)
    response = input("Launch interactive visualization? (y/n): ").strip().lower()
    if response == 'y':
        demo_visualization()
    else:
        print("Visualization skipped.")

    print("\nDemo complete!")


if __name__ == "__main__":
    main()
