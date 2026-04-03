"""
Visualization subpackage for interactive robot arm simulation.

This module provides:
    - app: Main application with pygame-based visualization
    - renderer: Drawing routines for the robot arm
    - input_handler: Mouse/keyboard input processing
"""

from robot_arm.visualization.app import (
    RobotArmSimulator,
    main,
)

__all__ = [
    "RobotArmSimulator",
    "main",
]
