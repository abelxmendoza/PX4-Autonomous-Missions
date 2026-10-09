"""Detection -> ground position for the downward camera, and per-target aggregation."""
from __future__ import annotations

import math
import random

import numpy as np
import pytest

from px4_offboard.search_geolocate import (
    SEARCH_CAMERA,
    TargetTracker,
    geolocate,
    project_ground_point,
    quat_from_euler,
)

LEVEL_NORTH = quat_from_euler(0.0, 0.0, 0.0)


def test_quat_from_euler_matches_the_rotation_the_ekf_uses():
    from px4_offboard.ekf_fusion import quat_to_rotation_matrix

    r = quat_to_rotation_matrix(quat_from_euler(0.0, 0.0, math.pi / 2))
    # Yaw 90 deg: body forward (FRD x) points east (NED y).
    assert np.allclose(r @ [1, 0, 0], [0, 1, 0], atol=1e-9)


def test_image_centre_from_a_level_drone_is_the_point_directly_below():
    n, e = geolocate(SEARCH_CAMERA, (10.0, 5.0, -8.0), LEVEL_NORTH, SEARCH_CAMERA.cx, SEARCH_CAMERA.cy)
    assert (n, e) == pytest.approx((10.0, 5.0), abs=1e-9)


def test_image_axes_follow_the_mount_up_is_forward_right_is_right():
    # The camera is pitched +90 deg about body y, so image up = drone forward and
    # image right = drone right. Facing north: north is up in the image, east is right.
    pos = (0.0, 0.0, -8.0)
    u_n, v_n = project_ground_point(SEARCH_CAMERA, pos, LEVEL_NORTH, (2.0, 0.0))
    u_e, v_e = project_ground_point(SEARCH_CAMERA, pos, LEVEL_NORTH, (0.0, 2.0))
    assert v_n < SEARCH_CAMERA.cy and u_n == pytest.approx(SEARCH_CAMERA.cx)
    assert u_e > SEARCH_CAMERA.cx and v_e == pytest.approx(SEARCH_CAMERA.cy)
    # Facing east, east is up in the image.
    u, v = project_ground_point(SEARCH_CAMERA, pos, quat_from_euler(0.0, 0.0, math.pi / 2), (0.0, 2.0))
    assert v < SEARCH_CAMERA.cy and u == pytest.approx(SEARCH_CAMERA.cx)


def test_geolocate_inverts_the_projection_for_tilted_drones_at_any_heading():
    rng = random.Random(3)
    for _ in range(300):
        pos = (rng.uniform(0, 60), rng.uniform(-30, 30), -rng.uniform(5, 12))
        q = quat_from_euler(math.radians(rng.uniform(-12, 12)), math.radians(rng.uniform(-12, 12)),
                            rng.uniform(-math.pi, math.pi))
        target = (pos[0] + rng.uniform(-4, 4), pos[1] + rng.uniform(-4, 4))
        pixel = project_ground_point(SEARCH_CAMERA, pos, q, target)
        assert pixel is not None
        assert geolocate(SEARCH_CAMERA, pos, q, *pixel) == pytest.approx(target, abs=1e-6)


def test_a_pixel_whose_ray_never_reaches_the_ground_gives_no_position():
    # Rolled 100 deg the camera looks above the horizon: no ground intersection.
    q = quat_from_euler(math.radians(100), 0.0, 0.0)
    assert geolocate(SEARCH_CAMERA, (0.0, 0.0, -8.0), q, SEARCH_CAMERA.cx, SEARCH_CAMERA.cy) is None
    # And a drone at or below the ground cannot see it from above.
    assert geolocate(SEARCH_CAMERA, (0.0, 0.0, 0.5), LEVEL_NORTH, SEARCH_CAMERA.cx, SEARCH_CAMERA.cy) is None


def test_tracker_confirms_a_target_once_after_enough_consistent_detections():
    t = TargetTracker(min_detections=3)
    assert t.add(7, 10.0, 5.0, 1.0) == []
    assert t.add(7, 10.2, 5.1, 1.1) == []
    assert t.add(7, 9.9, 4.9, 1.2) == [7]   # confirmed on the third
    assert t.add(7, 10.1, 5.0, 1.3) == []   # reported once only
    est = t.estimates()[7]
    assert (est.north, est.east) == pytest.approx((10.05, 4.95), abs=0.06)
    assert est.detections == 4 and est.first_seen_s == 1.0


def test_tracker_estimate_is_robust_to_a_single_wild_detection():
    t = TargetTracker(min_detections=3)
    for k, (n, e) in enumerate([(10.0, 5.0), (10.1, 5.0), (40.0, -20.0), (9.9, 5.1), (10.0, 4.9)]):
        t.add(3, n, e, float(k))
    est = t.estimates()[3]
    assert math.dist((est.north, est.east), (10.0, 5.0)) < 0.15  # median, not mean
    assert est.spread_m > 20  # but the outlier is visible in the spread, not hidden


def test_unconfirmed_targets_are_not_reported():
    t = TargetTracker(min_detections=3)
    t.add(1, 0.0, 0.0, 0.0)
    assert t.confirmed() == {}
