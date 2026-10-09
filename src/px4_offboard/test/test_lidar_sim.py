"""Synthetic 2-D LiDAR: exact ranges against boxes and circles."""
import math

import numpy as np
import pytest

from px4_offboard.lidar_sim import Box, Circle, scan_angles, simulate_scan


def test_scan_angles_match_the_x500_lidar():
    a = scan_angles()
    assert len(a) == 720
    assert a[0] == pytest.approx(-2.356195) and a[-1] == pytest.approx(2.356195)


def test_range_to_a_box_straight_ahead_and_to_the_side():
    wall = Box(north=10.0, east=0.0, size_north=1.0, size_east=20.0)   # face at north = 9.5
    r = simulate_scan([wall], (0.0, 0.0, 0.0), np.array([0.0]), max_range=30.0)
    assert r[0] == pytest.approx(9.5)
    # Facing east (yaw 90 deg), the beam at -90 deg (left) points north at the same wall.
    r = simulate_scan([wall], (0.0, 0.0, math.pi / 2), np.array([-math.pi / 2]), max_range=30.0)
    assert r[0] == pytest.approx(9.5)


def test_positive_beam_angles_are_to_the_right_of_the_heading():
    post = Circle(north=0.0, east=5.0, radius=0.5)  # due east of a north-facing drone
    r = simulate_scan([post], (0.0, 0.0, 0.0), np.array([math.pi / 2, -math.pi / 2]), max_range=30.0)
    assert r[0] == pytest.approx(4.5) and math.isinf(r[1])


def test_nearest_hit_wins_and_out_of_range_is_no_return():
    near, far = Circle(5.0, 0.0, 0.5), Box(20.0, 0.0, 2.0, 10.0)
    r = simulate_scan([far, near], (0.0, 0.0, 0.0), np.array([0.0]), max_range=30.0)
    assert r[0] == pytest.approx(4.5)
    r = simulate_scan([far], (0.0, 0.0, 0.0), np.array([0.0]), max_range=10.0)
    assert math.isinf(r[0])


def test_noise_is_seeded_and_bounded():
    wall = Box(10.0, 0.0, 1.0, 20.0)
    a = simulate_scan([wall], (0, 0, 0), np.zeros(200), 30.0, noise_std=0.02, rng=np.random.default_rng(1))
    b = simulate_scan([wall], (0, 0, 0), np.zeros(200), 30.0, noise_std=0.02, rng=np.random.default_rng(1))
    assert np.array_equal(a, b)
    assert abs(a.mean() - 9.5) < 0.01 and 0.01 < a.std() < 0.03


def test_a_drone_inside_a_box_sees_nothing_of_it():
    # Only outward faces count: a pose inside a footprint is a planning error, not a hit at 0.
    r = simulate_scan([Box(0.0, 0.0, 4.0, 4.0)], (0, 0, 0), np.array([0.0]), 30.0)
    assert math.isinf(r[0])
