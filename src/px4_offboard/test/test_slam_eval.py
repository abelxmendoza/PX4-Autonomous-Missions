"""Scoring helpers for SLAM runs (ground truth is used for scoring only)."""
import math

import pytest

from px4_offboard.slam_eval import TRUTH_BOXES, clearance_to_truth, world_of_px4_frames


def test_route_clearance_is_the_closest_approach_to_a_real_surface():
    n, e, sn, se = TRUTH_BOXES[0]                     # course box 1
    face = e + se / 2                                 # its east face
    route = [(n - 3.0, face + 1.5), (n + 3.0, face + 1.5)]
    assert clearance_to_truth(route) == pytest.approx(1.5, abs=0.05)
    through = [(n, e - 6.0), (n, e + 6.0)]
    assert clearance_to_truth(through) == pytest.approx(0.0, abs=1e-9)


def test_px4_frame_mapping_with_gps_is_one_position_fit_and_without_follows_each_pose():
    truth = [(float(k), 0.0, 0.0) for k in range(20)]
    biased = [(n, e, yaw + math.radians(3.0)) for n, e, yaw in truth]   # heading error only
    gps = world_of_px4_frames(biased, truth, gps=True)
    assert all(w == pytest.approx((0.0, 0.0, 0.0), abs=1e-9) for w in gps)
    denied = world_of_px4_frames(biased, truth, gps=False)
    assert math.degrees(denied[5][2]) == pytest.approx(-3.0)
