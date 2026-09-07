"""Unit tests for the pure pinhole-camera bearing/elevation/range math."""

import math

import pytest

from px4_offboard.vision_marker import PinholeCamera, bearing_elevation_deg, range_m


def test_focal_length_matches_pinhole_formula():
    # 90 degree hfov: focal length in pixels = (width/2) / tan(45deg) = width/2.
    cam = PinholeCamera(width_px=640, height_px=480, horizontal_fov_rad=math.radians(90))
    assert cam.focal_length_px == pytest.approx(320.0)


def test_centered_pixel_has_zero_bearing_and_elevation():
    cam = PinholeCamera(width_px=640, height_px=480, horizontal_fov_rad=math.radians(90))
    bearing, elevation = bearing_elevation_deg(cam, pixel_x=320.0, pixel_y=240.0)
    assert bearing == pytest.approx(0.0, abs=1e-9)
    assert elevation == pytest.approx(0.0, abs=1e-9)


def test_pixel_right_of_center_is_positive_bearing():
    cam = PinholeCamera(width_px=640, height_px=480, horizontal_fov_rad=math.radians(90))
    bearing, _ = bearing_elevation_deg(cam, pixel_x=640.0, pixel_y=240.0)
    # At the right edge with a 90 deg hfov, bearing should be 45 degrees.
    assert bearing == pytest.approx(45.0)


def test_pixel_above_center_is_positive_elevation():
    cam = PinholeCamera(width_px=640, height_px=480, horizontal_fov_rad=math.radians(90))
    _, elevation = bearing_elevation_deg(cam, pixel_x=320.0, pixel_y=0.0)
    assert elevation > 0.0


def test_pixel_below_center_is_negative_elevation():
    cam = PinholeCamera(width_px=640, height_px=480, horizontal_fov_rad=math.radians(90))
    _, elevation = bearing_elevation_deg(cam, pixel_x=320.0, pixel_y=480.0)
    assert elevation < 0.0


def test_range_matches_similar_triangles():
    cam = PinholeCamera(width_px=640, height_px=480, horizontal_fov_rad=math.radians(90))
    # focal_length_px = 320. A 0.3 m marker spanning 32 px should be 10x
    # farther than one spanning 320 px (which sits exactly at 0.3 m).
    close = range_m(cam, marker_size_m=0.3, apparent_size_px=320.0)
    far = range_m(cam, marker_size_m=0.3, apparent_size_px=32.0)
    assert close == pytest.approx(0.3)
    assert far == pytest.approx(3.0)


def test_range_rejects_non_positive_inputs():
    cam = PinholeCamera(width_px=640, height_px=480, horizontal_fov_rad=math.radians(90))
    with pytest.raises(ValueError):
        range_m(cam, marker_size_m=0.3, apparent_size_px=0.0)
    with pytest.raises(ValueError):
        range_m(cam, marker_size_m=0.0, apparent_size_px=100.0)
