"""
Main Robot Arm Simulator Application.

This module integrates all components into an interactive visualization:
    - Robot arm model with forward kinematics
    - Inverse kinematics solver
    - 3D rendering with perspective projection
    - Real-time user interaction

The application uses Pygame for rendering and input handling.

Author: Roman Prokhorov

File Location: src/robot_arm/visualization/app.py
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Tuple, Optional

import numpy as np

# Pygame import with error handling
try:
    import pygame
    from pygame import Surface, Color
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False

from robot_arm.core.vectors import Vector3
from robot_arm.kinematics.forward import RobotArm, JointType
from robot_arm.kinematics.inverse import DampedLeastSquaresSolver, IKResult, IKStatus
from robot_arm.rendering.projection import perspective_matrix, project_point, ProjectionConfig
from robot_arm.rendering.camera import OrbitCamera


class AppState(Enum):
    """Application state machine."""
    RUNNING = auto()
    PAUSED = auto()
    QUIT = auto()


@dataclass
class Colors:
    """Color palette for visualization."""
    BACKGROUND: Tuple[int, int, int] = (20, 20, 30)
    GRID: Tuple[int, int, int] = (50, 50, 60)
    ARM_LINK: Tuple[int, int, int] = (100, 150, 255)
    ARM_JOINT: Tuple[int, int, int] = (255, 200, 100)
    TARGET: Tuple[int, int, int] = (255, 100, 100)
    TARGET_REACHED: Tuple[int, int, int] = (100, 255, 100)
    END_EFFECTOR: Tuple[int, int, int] = (100, 255, 200)
    TEXT: Tuple[int, int, int] = (200, 200, 200)
    WORKSPACE: Tuple[int, int, int] = (40, 40, 50)
    AXIS_X: Tuple[int, int, int] = (255, 80, 80)
    AXIS_Y: Tuple[int, int, int] = (80, 255, 80)
    AXIS_Z: Tuple[int, int, int] = (80, 80, 255)


@dataclass
class SimulationConfig:
    """Configuration for the simulation."""
    # Window settings
    window_width: int = 1280
    window_height: int = 720
    window_title: str = "Robot Arm Inverse Kinematics Visualization"
    fps: int = 60

    # Robot arm settings
    link_lengths: Tuple[float, ...] = (2.0, 1.5, 1.0)

    # IK solver settings
    ik_iterations_per_frame: int = 10
    ik_damping: float = 0.5
    ik_tolerance: float = 0.01

    # Rendering settings
    joint_radius: int = 8
    link_width: int = 4
    target_radius: int = 12
    show_grid: bool = True
    show_workspace: bool = True
    show_axes: bool = True

    # Camera settings
    camera_distance: float = 15.0
    camera_elevation: float = 0.4
    camera_azimuth: float = 0.3


@dataclass
class InputState:
    """Current state of user input."""
    mouse_pos: Tuple[int, int] = (0, 0)
    mouse_pressed: Tuple[bool, bool, bool] = (False, False, False)
    mouse_delta: Tuple[int, int] = (0, 0)

    # Camera control
    rotating_camera: bool = False
    panning_camera: bool = False

    # Target control
    dragging_target: bool = False
    target_depth: float = 0.0


class RobotArmSimulator:
    """
    Main application class for robot arm visualization.

    Integrates:
        - RobotArm: The kinematic model
        - DampedLeastSquaresSolver: IK solver
        - OrbitCamera: Camera for 3D view
        - Pygame: Rendering and input

    Usage:
        >>> sim = RobotArmSimulator()
        >>> sim.run()
    """

    def __init__(self, config: SimulationConfig | None = None):
        """
        Initialize the simulator.

        Args:
            config: Simulation configuration (uses defaults if None)
        """
        if not PYGAME_AVAILABLE:
            raise RuntimeError(
                "Pygame is required for visualization. "
                "Install with: pip install pygame"
            )

        self.config = config or SimulationConfig()
        self.state = AppState.RUNNING
        self.colors = Colors()
        self.input = InputState()

        # Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode(
            (self.config.window_width, self.config.window_height)
        )
        pygame.display.set_caption(self.config.window_title)
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)

        # Initialize robot arm
        self.arm = RobotArm.create_3dof_arm(self.config.link_lengths)

        # Initialize IK solver
        self.ik_solver = DampedLeastSquaresSolver(
            damping=self.config.ik_damping,
            tolerance=self.config.ik_tolerance,
            max_iterations=self.config.ik_iterations_per_frame,
        )

        # Target position (in world coordinates)
        self.target = Vector3(3.0, 2.0, 0.0)
        self.target_reached = False

        # Camera
        self.camera = OrbitCamera(
            target=Vector3.zero(),
            distance=self.config.camera_distance,
            elevation=self.config.camera_elevation,
            azimuth=self.config.camera_azimuth,
        )

        # Projection configuration
        self.projection_config = ProjectionConfig(
            screen_width=self.config.window_width,
            screen_height=self.config.window_height,
        )

        # IK result for display
        self.last_ik_result: Optional[IKResult] = None

    def run(self) -> None:
        """
        Main application loop.

        Runs until user quits:
            1. Handle input events
            2. Update simulation (IK solve)
            3. Render frame
            4. Cap framerate
        """
        while self.state != AppState.QUIT:
            dt = self.clock.tick(self.config.fps) / 1000.0

            self._handle_events()

            if self.state == AppState.RUNNING:
                self._update(dt)

            self._render()

            pygame.display.flip()

        pygame.quit()

    def _handle_events(self) -> None:
        """Process pygame events."""
        # Reset delta for this frame
        self.input.mouse_delta = (0, 0)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.state = AppState.QUIT

            elif event.type == pygame.KEYDOWN:
                self._handle_keydown(event.key)

            elif event.type == pygame.MOUSEBUTTONDOWN:
                self._handle_mouse_down(event.button, event.pos)

            elif event.type == pygame.MOUSEBUTTONUP:
                self._handle_mouse_up(event.button, event.pos)

            elif event.type == pygame.MOUSEMOTION:
                self._handle_mouse_motion(event.pos, event.rel)

            elif event.type == pygame.MOUSEWHEEL:
                self._handle_mouse_wheel(event.y)

    def _handle_keydown(self, key: int) -> None:
        """Handle keyboard input."""
        if key == pygame.K_ESCAPE:
            self.state = AppState.QUIT
        elif key == pygame.K_SPACE:
            # Toggle pause
            if self.state == AppState.RUNNING:
                self.state = AppState.PAUSED
            else:
                self.state = AppState.RUNNING
        elif key == pygame.K_r:
            # Reset arm and target
            self.arm.set_angles([0.0] * self.arm.num_joints)
            self.target = Vector3(3.0, 2.0, 0.0)
        elif key == pygame.K_c:
            # Reset camera
            self.camera.reset()
        elif key == pygame.K_g:
            # Toggle grid
            self.config.show_grid = not self.config.show_grid
        elif key == pygame.K_w:
            # Toggle workspace visualization
            self.config.show_workspace = not self.config.show_workspace

    def _handle_mouse_down(self, button: int, pos: Tuple[int, int]) -> None:
        """Handle mouse button press."""
        if button == 1:  # Left click
            # Start dragging target
            self.input.dragging_target = True
            self._update_target_from_mouse(pos)
        elif button == 3:  # Right click
            # Start rotating camera
            self.input.rotating_camera = True
        elif button == 2:  # Middle click
            # Start panning camera
            self.input.panning_camera = True

    def _handle_mouse_up(self, button: int, pos: Tuple[int, int]) -> None:
        """Handle mouse button release."""
        if button == 1:
            self.input.dragging_target = False
        elif button == 3:
            self.input.rotating_camera = False
        elif button == 2:
            self.input.panning_camera = False

    def _handle_mouse_motion(
        self,
        pos: Tuple[int, int],
        rel: Tuple[int, int]
    ) -> None:
        """Handle mouse movement."""
        self.input.mouse_pos = pos
        self.input.mouse_delta = rel

        if self.input.dragging_target:
            self._update_target_from_mouse(pos)

        if self.input.rotating_camera:
            # Rotate camera based on mouse delta
            sensitivity = 0.005
            self.camera.rotate(
                -rel[0] * sensitivity,
                -rel[1] * sensitivity
            )

        if self.input.panning_camera:
            # Pan camera
            sensitivity = 0.01
            self.camera.pan(rel[0] * sensitivity, -rel[1] * sensitivity)

    def _handle_mouse_wheel(self, y: int) -> None:
        """Handle mouse wheel for zoom."""
        zoom_speed = 1.0
        self.camera.zoom(y * zoom_speed)

    def _update_target_from_mouse(self, pos: Tuple[int, int]) -> None:
        """
        Convert mouse position to 3D target position.

        This is an inverse projection problem: we need to map 2D screen
        coordinates back to 3D world coordinates. We keep the target
        on a plane at fixed depth.
        """
        # TODO: Implement mouse-to-3D coordinate conversion (Roman)
        #
        # Approach:
        # 1. Get inverse view and projection matrices
        # 2. Convert screen pos to NDC: x_ndc = 2*x/width - 1, y_ndc = 1 - 2*y/height
        # 3. Create ray from camera through the NDC point
        # 4. Intersect ray with target plane (e.g., Z=0 plane)
        #
        # For now, use a simplified 2D mapping:
        width = self.config.window_width
        height = self.config.window_height

        # Simple mapping: screen center is (0,0), edges are workspace bounds
        max_reach = self.arm.total_reach
        x = (pos[0] - width / 2) / (width / 2) * max_reach
        y = -(pos[1] - height / 2) / (height / 2) * max_reach

        self.target = Vector3(x, y, 0.0)

    def _update(self, dt: float) -> None:
        """
        Update simulation state.

        Runs IK solver to move arm toward target.

        Args:
            dt: Time delta since last frame
        """
        # Run IK solver
        self.last_ik_result = self.ik_solver.solve(
            self.arm,
            self.target,
            initial_angles=self.arm.get_angles(),
        )

        # Check if target is reached
        if self.last_ik_result:
            self.target_reached = self.last_ik_result.success

    def _render(self) -> None:
        """Render the current frame."""
        # Clear screen
        self.screen.fill(self.colors.BACKGROUND)

        # Get view and projection matrices
        view_matrix = self.camera.get_view_matrix()
        proj_matrix = self.projection_config.get_perspective_matrix()

        # Draw grid
        if self.config.show_grid:
            self._draw_grid(view_matrix, proj_matrix)

        # Draw coordinate axes
        if self.config.show_axes:
            self._draw_axes(view_matrix, proj_matrix)

        # Draw workspace boundary
        if self.config.show_workspace:
            self._draw_workspace(view_matrix, proj_matrix)

        # Draw robot arm
        self._draw_arm(view_matrix, proj_matrix)

        # Draw target
        self._draw_target(view_matrix, proj_matrix)

        # Draw UI overlay
        self._draw_ui()

    def _project_point(
        self,
        point: Vector3,
        view_matrix: np.ndarray,
        proj_matrix: np.ndarray
    ) -> Optional[Tuple[int, int]]:
        """
        Project 3D point to screen coordinates.

        Returns None if point is behind camera.
        """
        # TODO: Use projection module when implemented
        # For now, use simplified projection
        try:
            # Transform to view space
            p = np.array([point.x, point.y, point.z, 1.0])
            p_view = view_matrix @ p

            # Skip if behind camera
            if p_view[2] > -0.1:
                return None

            # Project
            p_clip = proj_matrix @ p_view

            # Perspective divide
            if abs(p_clip[3]) < 1e-6:
                return None

            x_ndc = p_clip[0] / p_clip[3]
            y_ndc = p_clip[1] / p_clip[3]

            # To screen coordinates
            x_screen = int((x_ndc + 1) * self.config.window_width / 2)
            y_screen = int((1 - y_ndc) * self.config.window_height / 2)

            return (x_screen, y_screen)
        except Exception:
            return None

    def _draw_grid(
        self,
        view_matrix: np.ndarray,
        proj_matrix: np.ndarray
    ) -> None:
        """Draw ground grid for spatial reference."""
        grid_size = 10
        grid_spacing = 1.0

        for i in range(-grid_size, grid_size + 1):
            # Lines parallel to X
            start = Vector3(-grid_size * grid_spacing, 0, i * grid_spacing)
            end = Vector3(grid_size * grid_spacing, 0, i * grid_spacing)
            self._draw_line_3d(start, end, self.colors.GRID, view_matrix, proj_matrix)

            # Lines parallel to Z
            start = Vector3(i * grid_spacing, 0, -grid_size * grid_spacing)
            end = Vector3(i * grid_spacing, 0, grid_size * grid_spacing)
            self._draw_line_3d(start, end, self.colors.GRID, view_matrix, proj_matrix)

    def _draw_axes(
        self,
        view_matrix: np.ndarray,
        proj_matrix: np.ndarray
    ) -> None:
        """Draw coordinate axes at origin."""
        axis_length = 2.0
        origin = Vector3.zero()

        # X axis (red)
        self._draw_line_3d(
            origin,
            Vector3(axis_length, 0, 0),
            self.colors.AXIS_X,
            view_matrix,
            proj_matrix,
            width=2
        )

        # Y axis (green)
        self._draw_line_3d(
            origin,
            Vector3(0, axis_length, 0),
            self.colors.AXIS_Y,
            view_matrix,
            proj_matrix,
            width=2
        )

        # Z axis (blue)
        self._draw_line_3d(
            origin,
            Vector3(0, 0, axis_length),
            self.colors.AXIS_Z,
            view_matrix,
            proj_matrix,
            width=2
        )

    def _draw_workspace(
        self,
        view_matrix: np.ndarray,
        proj_matrix: np.ndarray
    ) -> None:
        """Draw circular workspace boundary."""
        max_reach = self.arm.total_reach
        segments = 64

        for i in range(segments):
            angle1 = 2 * math.pi * i / segments
            angle2 = 2 * math.pi * (i + 1) / segments

            p1 = Vector3(
                max_reach * math.cos(angle1),
                max_reach * math.sin(angle1),
                0
            )
            p2 = Vector3(
                max_reach * math.cos(angle2),
                max_reach * math.sin(angle2),
                0
            )

            self._draw_line_3d(
                p1, p2,
                self.colors.WORKSPACE,
                view_matrix,
                proj_matrix
            )

    def _draw_arm(
        self,
        view_matrix: np.ndarray,
        proj_matrix: np.ndarray
    ) -> None:
        """Draw the robot arm."""
        # Get joint positions
        try:
            positions = self.arm.get_joint_positions()
        except NotImplementedError:
            # FK not implemented yet - draw placeholder
            self._draw_placeholder_arm()
            return

        # Add base position at start
        all_positions = [self.arm.base_position] + positions

        # Draw links
        for i in range(len(all_positions) - 1):
            self._draw_line_3d(
                all_positions[i],
                all_positions[i + 1],
                self.colors.ARM_LINK,
                view_matrix,
                proj_matrix,
                width=self.config.link_width
            )

        # Draw joints
        for i, pos in enumerate(all_positions):
            screen_pos = self._project_point(pos, view_matrix, proj_matrix)
            if screen_pos:
                color = (
                    self.colors.END_EFFECTOR if i == len(all_positions) - 1
                    else self.colors.ARM_JOINT
                )
                radius = (
                    self.config.joint_radius + 4 if i == len(all_positions) - 1
                    else self.config.joint_radius
                )
                pygame.draw.circle(self.screen, color, screen_pos, radius)

    def _draw_placeholder_arm(self) -> None:
        """Draw placeholder when FK not implemented."""
        center = (self.config.window_width // 2, self.config.window_height // 2)

        # Draw text
        text = self.font.render(
            "FK not implemented - arm visualization placeholder",
            True,
            self.colors.TEXT
        )
        self.screen.blit(text, (center[0] - text.get_width() // 2, center[1]))

    def _draw_target(
        self,
        view_matrix: np.ndarray,
        proj_matrix: np.ndarray
    ) -> None:
        """Draw the target position."""
        screen_pos = self._project_point(self.target, view_matrix, proj_matrix)

        if screen_pos:
            color = (
                self.colors.TARGET_REACHED if self.target_reached
                else self.colors.TARGET
            )

            # Draw crosshair
            size = self.config.target_radius
            pygame.draw.line(
                self.screen, color,
                (screen_pos[0] - size, screen_pos[1]),
                (screen_pos[0] + size, screen_pos[1]),
                2
            )
            pygame.draw.line(
                self.screen, color,
                (screen_pos[0], screen_pos[1] - size),
                (screen_pos[0], screen_pos[1] + size),
                2
            )
            pygame.draw.circle(self.screen, color, screen_pos, size // 2, 2)

    def _draw_line_3d(
        self,
        start: Vector3,
        end: Vector3,
        color: Tuple[int, int, int],
        view_matrix: np.ndarray,
        proj_matrix: np.ndarray,
        width: int = 1
    ) -> None:
        """Draw a 3D line segment."""
        screen_start = self._project_point(start, view_matrix, proj_matrix)
        screen_end = self._project_point(end, view_matrix, proj_matrix)

        if screen_start and screen_end:
            pygame.draw.line(self.screen, color, screen_start, screen_end, width)

    def _draw_ui(self) -> None:
        """Draw user interface overlay."""
        y_offset = 10
        line_height = 25

        # Title
        self._draw_text(
            "Robot Arm Inverse Kinematics",
            (10, y_offset),
            self.colors.TEXT
        )
        y_offset += line_height + 10

        # Status
        status = "PAUSED" if self.state == AppState.PAUSED else "RUNNING"
        self._draw_text(f"Status: {status}", (10, y_offset), self.colors.TEXT)
        y_offset += line_height

        # Target position
        self._draw_text(
            f"Target: ({self.target.x:.2f}, {self.target.y:.2f}, {self.target.z:.2f})",
            (10, y_offset),
            self.colors.TEXT
        )
        y_offset += line_height

        # IK result
        if self.last_ik_result:
            ik_status = "Reached" if self.last_ik_result.success else "Solving..."
            self._draw_text(
                f"IK: {ik_status} (error: {self.last_ik_result.final_error:.4f})",
                (10, y_offset),
                self.colors.TEXT
            )
            y_offset += line_height

        # Joint angles
        angles = self.arm.get_angles()
        angles_str = ", ".join(f"{math.degrees(a):.1f}°" for a in angles)
        self._draw_text(f"Angles: [{angles_str}]", (10, y_offset), self.colors.TEXT)
        y_offset += line_height + 10

        # Controls help
        controls = [
            "Controls:",
            "  Left click: Move target",
            "  Right drag: Rotate camera",
            "  Scroll: Zoom",
            "  Space: Pause/Resume",
            "  R: Reset",
            "  C: Reset camera",
            "  G: Toggle grid",
            "  ESC: Quit",
        ]

        for line in controls:
            self._draw_text(line, (10, y_offset), self.colors.TEXT)
            y_offset += line_height - 5

    def _draw_text(
        self,
        text: str,
        pos: Tuple[int, int],
        color: Tuple[int, int, int]
    ) -> None:
        """Draw text on screen."""
        surface = self.font.render(text, True, color)
        self.screen.blit(surface, pos)


def main() -> None:
    """Entry point for the robot arm simulator."""
    try:
        sim = RobotArmSimulator()
        sim.run()
    except RuntimeError as e:
        print(f"Error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nSimulation terminated by user.")
        sys.exit(0)


if __name__ == "__main__":
    main()
