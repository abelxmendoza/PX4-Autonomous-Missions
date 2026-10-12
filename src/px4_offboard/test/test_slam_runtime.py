"""Time alignment, frame conversion and drift injection for online SLAM."""
import math

import pytest

from px4_offboard.slam_runtime import DriftingOdometry, PoseBuffer, euler_from_quaternion, gazebo_pose_to_ned


def _q_yaw(yaw):
    return math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)


def test_gazebo_enu_pose_becomes_ned():
    # Gazebo x=east, y=north; a model facing +x (east) has NED heading +90 deg.
    n, e, yaw = gazebo_pose_to_ned(3.0, 7.0, *_q_yaw(0.0))
    assert (n, e) == (7.0, 3.0) and yaw == pytest.approx(math.pi / 2)
    # Facing +y (north) in ENU is heading 0 in NED.
    assert gazebo_pose_to_ned(0, 0, *_q_yaw(math.pi / 2))[2] == pytest.approx(0.0)


def test_euler_from_quaternion_round_trips_roll_pitch_yaw():
    r, p, y = 0.1, -0.2, 2.5
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2),
                              math.cos(y / 2), math.sin(y / 2))
    q = (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
         cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)
    assert euler_from_quaternion(*q) == pytest.approx((r, p, y))


def test_pose_buffer_interpolates_including_across_the_yaw_wrap():
    b = PoseBuffer(max_gap_s=2.0)
    b.add(1.0, (0.0, 0.0, math.radians(179)))
    b.add(2.0, (2.0, 4.0, math.radians(-179)))
    n, e, yaw = b.at(1.5)
    assert (n, e) == pytest.approx((1.0, 2.0))
    assert abs(abs(yaw) - math.pi) < 1e-9          # 180 deg, not 0
    assert b.at(2.0) == (2.0, 4.0, math.radians(-179))
    assert b.at(0.5) is None and b.at(2.5) is None  # never extrapolates


def test_pose_buffer_refuses_to_bridge_a_gap_and_drops_old_poses():
    b = PoseBuffer(max_age_s=5.0, max_gap_s=0.25)
    b.add(0.0, (0.0, 0.0, 0.0))
    b.add(1.0, (1.0, 0.0, 0.0))
    assert b.at(0.5) is None
    b.add(10.0, (5.0, 0.0, 0.0))
    b.add(9.0, (9.0, 9.0, 0.0))                     # out of order: ignored
    assert b.at(1.0) is None and b.at(10.0) == (5.0, 0.0, 0.0)


def test_drifting_odometry_scales_steps_and_accumulates_heading_bias():
    d = DriftingOdometry(scale=1.1, yaw_drift_deg_per_s=1.0)
    assert d.update(0.0, (0.0, 0.0, 0.0)) == (0.0, 0.0, 0.0)
    n, e, yaw = d.update(1.0, (10.0, 0.0, 0.0))
    assert (n, e) == pytest.approx((11.0, 0.0)) and yaw == pytest.approx(math.radians(1.0))
    n, e, yaw = d.update(2.0, (20.0, 0.0, 0.0))     # the next step leaves along the biased heading
    assert yaw == pytest.approx(math.radians(2.0))
    assert e == pytest.approx(11.0 * math.sin(math.radians(1.0)), rel=1e-6)
    assert DriftingOdometry().update(0.0, (1.0, 2.0, 0.3)) == (1.0, 2.0, 0.3)


def test_align_se2_recovers_a_rigid_offset_and_ignores_none():
    import numpy as np

    from px4_offboard.slam2d import compose
    from px4_offboard.slam_runtime import align_se2
    rng = np.random.default_rng(0)
    pts = rng.uniform(-20, 40, size=(50, 2))
    offset = (1.5, -2.0, math.radians(4.7))
    moved = np.array([compose(offset, (n, e, 0.0))[:2] for n, e in pts])
    dn, de, dyaw = align_se2(pts, moved)
    assert (dn, de, dyaw) == pytest.approx(offset, abs=1e-9)
    assert align_se2(pts, pts) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)


def _rot(offset, pose):
    from px4_offboard.slam2d import compose
    return compose(offset, pose)


def test_gps_mode_recovers_the_map_frame_rotation_from_positions_not_from_a_biased_heading():
    import numpy as np

    from px4_offboard.slam_runtime import MapFrameTracker
    # Truth path: an L-shaped survey. SLAM's map is the world rotated 4.4 deg about a point
    # near the start (its first pose); PX4 (GPS) has positions to 5 cm and a heading 2 deg off.
    rng = np.random.default_rng(1)
    path = [(n, 0.0, 0.0) for n in np.linspace(0, 30, 61)] + [(30.0, e, math.pi / 2) for e in np.linspace(0, 20, 41)]
    world_to_map = (0.3, -0.2, math.radians(4.4))
    tracker = MapFrameTracker(mode="gps")
    for k, p in enumerate(path):
        map_pose = _rot(world_to_map, p)
        px4 = (p[0] + rng.normal(0, 0.05), p[1] + rng.normal(0, 0.05), p[2] + math.radians(-2.0))
        tracker.update(k * 0.5, map_pose, px4)
    # map -> px4 should undo world_to_map, i.e. a -4.4 deg rotation, NOT -4.4 + (-2) deg.
    dn, de, dyaw = tracker.transform()
    assert math.degrees(dyaw) == pytest.approx(-4.4, abs=0.3)
    x_map = _rot(world_to_map, (40.0, 10.0, 0.0))
    n, e = tracker.to_px4(x_map[:2])
    assert math.dist((n, e), (40.0, 10.0)) < 0.25


def test_gps_mode_uses_the_pose_pair_until_the_path_has_spread_out():
    from px4_offboard.slam_runtime import MapFrameTracker
    tracker = MapFrameTracker(mode="gps", min_spread_m=5.0)
    assert tracker.transform() is None
    tracker.update(0.0, (1.0, 0.0, 0.1), (11.0, 5.0, 0.3))
    dn, de, dyaw = tracker.transform()
    assert dyaw == pytest.approx(0.2)
    assert tracker.to_px4((1.0, 0.0)) == pytest.approx((11.0, 5.0))


def test_odometry_mode_follows_a_drifting_px4_frame():
    from px4_offboard.slam_runtime import DriftingOdometry, MapFrameTracker
    # GPS-denied: PX4's own frame drifts. A map point must be sent to wherever PX4's *current*
    # frame puts it, so the transform has to move with the drift.
    drift = DriftingOdometry(scale=1.05, yaw_drift_deg_per_s=1.0)
    tracker = MapFrameTracker(mode="odometry", smoothing_s=0.0)
    for k in range(80):
        t = k * 0.5
        truth = (0.5 * k, 0.0, 0.0)
        tracker.update(t, truth, drift.update(t, truth))   # SLAM exact: map == world
    target_world = (45.0, 5.0)
    # Where PX4's drifting frame says that world point is, right now:
    from px4_offboard.slam2d import compose, relative
    world_to_px4 = compose(drift.pose, relative((0.5 * 79, 0.0, 0.0), (0.0, 0.0, 0.0)))
    expected = compose(world_to_px4, (*target_world, 0.0))[:2]
    assert tracker.to_px4(target_world) == pytest.approx(expected, abs=1e-6)
    assert math.dist(expected, target_world) > 2.0       # the drift is real...
