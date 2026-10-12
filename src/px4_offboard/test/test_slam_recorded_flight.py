"""Regression: replay a real PX4/Gazebo LiDAR flight through SLAM, with no simulator.

The fixture (test/data/gazebo_lidar_obstacle_course.npz, 156 KB) is 410 real Gazebo GPU-lidar
scans from tools/slam_flight.py --raw: a 108 s loop of the obstacle course at 3 m, with PX4's
pose estimate, attitude and height for each scan, and Gazebo's pose (scoring only). The
odometry is degraded exactly as the live demo does it (3% scale, 0.5 deg/s heading drift).

Recorded baseline (2026-10-12): 409/410 scans matched; path error after one rigid frame fit
5.3 cm mean, 0.94 m max; 91.8% of map cells within 0.5 m of a real surface; 5/5 course boxes
mapped; dead reckoning ends 13.8 m off. Map points sent to PX4 through MapFrameTracker land
0.14 m (GPS-denied) / 0.07 m (GPS) from where they really are, against 4.67 m / 1.63 m if the
map's coordinates were sent as they are. The thresholds below sit roughly 2x outside that, so a
change that makes SLAM measurably worse on real sensor data fails here, in CI.
"""
import math
from pathlib import Path

import pytest

from px4_offboard.slam_eval import load_recorded_flight, replay
from px4_offboard.slam_runtime import DriftingOdometry

FIXTURE = Path(__file__).parent / "data" / "gazebo_lidar_obstacle_course.npz"


@pytest.fixture(scope="module")
def result():
    return replay(load_recorded_flight(FIXTURE), DriftingOdometry(1.03, 0.5))


@pytest.fixture(scope="module")
def with_gps():
    return replay(load_recorded_flight(FIXTURE))


def test_the_recording_is_the_flight_described():
    f = load_recorded_flight(FIXTURE)
    assert len(f["t"]) == 410 and f["ranges"].shape == (410, 720)
    assert 100.0 < f["t"][-1] < 115.0
    assert abs(f["angles"][0]) == pytest.approx(math.radians(135), abs=1e-4)


def test_injected_drift_is_large_enough_to_matter(result):
    assert result["odometry_final_m"] > 10.0


def test_slam_tracks_the_real_flight_despite_the_drift(result):
    e = result["errors"]
    assert result["matched"] >= 400
    assert e["ate_mean_m"] < 0.12, e
    assert e["ate_max_m"] < 1.8, e


def test_the_map_lies_on_real_surfaces_and_covers_every_course_box(result):
    m = result["map"]
    assert m["within_0_5m_after_frame_alignment"] > 0.85, m
    assert m["course_boxes_mapped"] == "5/5", m


def test_the_map_frame_offset_stays_what_it_was_recorded_as(result):
    # The frame is rotated by PX4's heading error at the first scan (about -4.4 deg in this
    # recording). A change here means the anchoring changed, not the matcher.
    assert result["errors"]["frame_yaw_deg"] == pytest.approx(-4.4, abs=1.5)


def test_gps_denied_map_points_reach_px4_where_they_really_are(result):
    reg = result["registration"]
    assert reg["untransformed_mean_m"] > 2.0, reg          # without tracking it would miss...
    assert reg["tracked_mean_m"] < 0.3, reg                # ...with it, routes land on target
    assert reg["tracked_max_m"] < 1.2, reg


def test_with_gps_the_tracker_removes_the_map_frame_rotation(with_gps):
    reg = with_gps["registration"]
    assert reg["tracked_mean_m"] < 0.2, reg
    assert reg["tracked_max_m"] < 1.5, reg
    assert reg["tracked_mean_m"] < 0.25 * reg["untransformed_mean_m"], reg
    # The rotation the tracker learned from PX4's data alone matches the one ground truth shows.
    learned = math.degrees(with_gps["tracker"].transform()[2])
    assert learned == pytest.approx(with_gps["errors"]["frame_yaw_deg"], abs=0.5)
