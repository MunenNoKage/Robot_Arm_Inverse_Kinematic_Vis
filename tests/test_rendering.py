"""
Test Suite for Rendering Module.

Tests for:
    - Projection matrices
    - Camera view matrix
    - Full rendering pipeline

Author: Mykola Balyk

File Location: tests/test_rendering.py
"""

import math
import pytest
import numpy as np

from robot_arm.core.vectors import Vector3
from robot_arm.rendering.projection import (
    perspective_matrix,
    orthographic_matrix,
    viewport_transform,
    project_point,
    ProjectionConfig,
)
from robot_arm.rendering.camera import Camera, OrbitCamera, look_at


class TestPerspectiveProjection:
    """Tests for perspective projection matrix."""

    @pytest.mark.skip(reason="Waiting for Mykola's implementation")
    def test_perspective_matrix_shape(self):
        """Test projection matrix has correct shape."""
        P = perspective_matrix(math.radians(60), 16/9, 0.1, 100)
        assert P.shape == (4, 4)

    @pytest.mark.skip(reason="Waiting for Mykola's implementation")
    def test_perspective_origin_unchanged(self):
        """Test that origin projects to center."""
        P = perspective_matrix(math.radians(60), 1.0, 0.1, 100)
        V = np.eye(4)  # Identity view matrix

        # Point at origin, slightly in front of camera
        p = np.array([0, 0, -1, 1])

        p_clip = P @ p
        p_ndc = p_clip[:3] / p_clip[3]

        # Should be at center (0, 0)
        assert abs(p_ndc[0]) < 1e-6
        assert abs(p_ndc[1]) < 1e-6

    @pytest.mark.skip(reason="Waiting for Mykola's implementation")
    def test_perspective_foreshortening(self):
        """Test that distant objects appear smaller."""
        P = perspective_matrix(math.radians(60), 1.0, 0.1, 100)

        # Same x,y offset at different depths
        p_near = np.array([1, 0, -2, 1])
        p_far = np.array([1, 0, -10, 1])

        p_near_clip = P @ p_near
        p_far_clip = P @ p_far

        x_near = p_near_clip[0] / p_near_clip[3]
        x_far = p_far_clip[0] / p_far_clip[3]

        # Far point should appear closer to center
        assert abs(x_far) < abs(x_near)


class TestOrthographicProjection:
    """Tests for orthographic projection matrix."""

    @pytest.mark.skip(reason="Waiting for Mykola's implementation")
    def test_orthographic_matrix_shape(self):
        """Test orthographic matrix shape."""
        O = orthographic_matrix(-5, 5, -5, 5, 0.1, 100)
        assert O.shape == (4, 4)

    @pytest.mark.skip(reason="Waiting for Mykola's implementation")
    def test_orthographic_no_foreshortening(self):
        """Test that depth doesn't affect x,y projection."""
        O = orthographic_matrix(-5, 5, -5, 5, 0.1, 100)

        p_near = np.array([1, 1, -2, 1])
        p_far = np.array([1, 1, -50, 1])

        p_near_proj = O @ p_near
        p_far_proj = O @ p_far

        # x,y should be same regardless of depth
        np.testing.assert_almost_equal(p_near_proj[0], p_far_proj[0])
        np.testing.assert_almost_equal(p_near_proj[1], p_far_proj[1])


class TestViewportTransform:
    """Tests for viewport transformation."""

    @pytest.mark.skip(reason="Waiting for Mykola's implementation")
    def test_viewport_center(self):
        """Test that NDC (0,0) maps to screen center."""
        V = viewport_transform(1280, 720)

        p_ndc = np.array([0, 0, 0, 1])
        p_screen = V @ p_ndc

        assert abs(p_screen[0] - 640) < 1e-6
        assert abs(p_screen[1] - 360) < 1e-6

    @pytest.mark.skip(reason="Waiting for Mykola's implementation")
    def test_viewport_corners(self):
        """Test that NDC corners map to screen corners."""
        V = viewport_transform(800, 600)

        # Top-left NDC (-1, 1) should map to screen (0, 0)
        p_tl = np.array([-1, 1, 0, 1])
        p_tl_screen = V @ p_tl
        assert abs(p_tl_screen[0]) < 1e-6
        assert abs(p_tl_screen[1]) < 1e-6

        # Bottom-right NDC (1, -1) should map to (800, 600)
        p_br = np.array([1, -1, 0, 1])
        p_br_screen = V @ p_br
        assert abs(p_br_screen[0] - 800) < 1e-6
        assert abs(p_br_screen[1] - 600) < 1e-6


class TestCamera:
    """Tests for Camera class."""

    @pytest.mark.skip(reason="Waiting for Mykola's implementation")
    def test_view_matrix_shape(self):
        """Test view matrix has correct shape."""
        cam = Camera(
            position=Vector3(0, 0, 10),
            target=Vector3.zero(),
        )
        V = cam.get_view_matrix()
        assert V.shape == (4, 4)

    @pytest.mark.skip(reason="Waiting for Mykola's implementation")
    def test_view_matrix_transforms_origin(self):
        """Test that origin transforms to expected position."""
        cam = Camera(
            position=Vector3(0, 0, 10),
            target=Vector3.zero(),
        )
        V = cam.get_view_matrix()

        # Origin in world space
        p_world = np.array([0, 0, 0, 1])
        p_view = V @ p_world

        # In view space, origin should be at (0, 0, -10)
        # because camera is at Z=10 looking at origin
        assert abs(p_view[0]) < 1e-6
        assert abs(p_view[1]) < 1e-6
        assert abs(p_view[2] - (-10)) < 1e-6

    @pytest.mark.skip(reason="Waiting for Mykola's implementation")
    def test_camera_forward_direction(self):
        """Test camera forward direction computation."""
        cam = Camera(
            position=Vector3(0, 0, 10),
            target=Vector3.zero(),
        )
        forward = cam.get_forward()

        # Should point toward -Z
        assert abs(forward.x) < 1e-6
        assert abs(forward.y) < 1e-6
        assert forward.z < 0  # Pointing toward origin


class TestOrbitCamera:
    """Tests for OrbitCamera class."""

    @pytest.mark.skip(reason="Waiting for Mykola's implementation")
    def test_orbit_initial_position(self):
        """Test orbit camera position calculation."""
        cam = OrbitCamera(
            target=Vector3.zero(),
            distance=10.0,
            azimuth=0.0,
            elevation=0.0,
        )
        pos = cam.get_position()

        # With zero elevation and azimuth, should be along +Z
        assert abs(pos.x) < 1e-6
        assert abs(pos.y) < 1e-6
        assert abs(pos.z - 10) < 1e-6

    @pytest.mark.skip(reason="Waiting for Mykola's implementation")
    def test_orbit_rotation(self):
        """Test orbit camera rotation."""
        cam = OrbitCamera(
            target=Vector3.zero(),
            distance=10.0,
        )

        initial_pos = cam.get_position()
        cam.rotate(math.pi / 2, 0)  # 90° horizontal rotation
        rotated_pos = cam.get_position()

        # Position should have changed
        assert abs(initial_pos.x - rotated_pos.x) > 0.1 or \
               abs(initial_pos.z - rotated_pos.z) > 0.1

    @pytest.mark.skip(reason="Waiting for Mykola's implementation")
    def test_orbit_zoom(self):
        """Test orbit camera zoom."""
        cam = OrbitCamera(target=Vector3.zero(), distance=10.0)

        cam.zoom(2.0)  # Zoom in
        assert cam.distance == 8.0

        cam.zoom(-3.0)  # Zoom out
        assert cam.distance == 11.0

    def test_orbit_zoom_limits(self):
        """Test that zoom respects limits."""
        cam = OrbitCamera(
            target=Vector3.zero(),
            distance=10.0,
            min_distance=5.0,
            max_distance=20.0,
        )

        cam.zoom(100.0)  # Try to zoom in too much
        assert cam.distance >= 5.0

        cam.zoom(-100.0)  # Try to zoom out too much
        assert cam.distance <= 20.0


class TestProjectPoint:
    """Tests for full projection pipeline."""

    @pytest.mark.skip(reason="Waiting for Mykola's implementation")
    def test_project_point_in_view(self):
        """Test projecting a point in front of camera."""
        cam = Camera(Vector3(0, 0, 10), Vector3.zero())
        V = cam.get_view_matrix()
        P = perspective_matrix(math.radians(60), 1.0, 0.1, 100)

        # Point at origin
        x, y, z = project_point(Vector3.zero(), V, P)

        # Should be roughly at center of NDC
        assert -1 <= x <= 1
        assert -1 <= y <= 1

    @pytest.mark.skip(reason="Waiting for Mykola's implementation")
    def test_project_multiple_points(self):
        """Test projecting multiple points maintains relative positions."""
        cam = Camera(Vector3(0, 0, 10), Vector3.zero())
        V = cam.get_view_matrix()
        P = perspective_matrix(math.radians(60), 1.0, 0.1, 100)

        # Point to the left should project left
        x_left, _, _ = project_point(Vector3(-1, 0, 0), V, P)
        x_right, _, _ = project_point(Vector3(1, 0, 0), V, P)

        assert x_left < x_right


class TestProjectionConfig:
    """Tests for ProjectionConfig class."""

    def test_default_config(self):
        """Test default configuration values."""
        config = ProjectionConfig()
        assert config.screen_width == 1280
        assert config.screen_height == 720
        assert abs(config.aspect_ratio - 16/9) < 0.01

    def test_update_aspect_ratio(self):
        """Test aspect ratio update."""
        config = ProjectionConfig()
        config.update_aspect_ratio(800, 600)

        assert config.screen_width == 800
        assert config.screen_height == 600
        assert abs(config.aspect_ratio - 800/600) < 1e-6


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
