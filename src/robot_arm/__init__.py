"""
Robot Arm Simulation Package

A Linear Algebra course project demonstrating how matrix transformations,
rotation groups, and projection geometry power real-time robotic simulation.

Authors:
    - Roman Prokhorov
    - Nazar Pasichnyk
    - Mykola Balyk

Course: Linear Algebra, March 2026
"""

__version__ = "0.1.0"
__authors__ = ["Roman Prokhorov", "Nazar Pasichnyk", "Mykola Balyk"]

from robot_arm.core import transforms, vectors
from robot_arm.kinematics import forward, inverse
from robot_arm.rendering import projection, camera
from robot_arm.visualization import app

__all__ = [
    "transforms",
    "vectors",
    "forward",
    "inverse",
    "projection",
    "camera",
    "app",
]
