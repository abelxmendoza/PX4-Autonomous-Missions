"""2-D LiDAR SLAM: occupancy mapping, scan-to-map matching, and correcting odometry drift."""
import math

import numpy as np
import pytest

from px4_offboard.lidar_sim import Box, Circle, scan_angles, simulate_scan
from px4_offboard.slam2d import OccupancyGrid, Slam2D, compose, relative

ANGLES = scan_angles()
# A small synthetic site: two buildings, a wall, three posts (NED metres).
WORLD = [Box(20.0, -10.0, 8.0, 10.0), Box(35.0, 12.0, 10.0, 8.0), Box(28.0, 0.0, 12.0, 0.6),
         Circle(12.0, 6.0, 0.6), Circle(42.0, -14.0, 0.6), Circle(10.0, -18.0, 0.6)]
BOUNDS = dict(north_min=-5.0, north_max=55.0, east_min=-30.0, east_max=30.0)


def _grid():
    return OccupancyGrid(resolution=0.25, **BOUNDS)


def test_pose_composition_round_trips():
    a, b = (3.0, -2.0, 0.7), (1.5, 0.5, -0.2)
    assert relative(a, compose(a, b)) == pytest.approx(b)


def test_one_scan_marks_the_wall_occupied_the_way_to_it_free_and_behind_it_unknown():
    g = _grid()
    wall = [Box(10.0, 0.0, 1.0, 20.0)]
    scan = simulate_scan(wall, (0, 0, 0), ANGLES)
    g.update((0.0, 0.0, 0.0), ANGLES, scan)
    assert g.probability_at(9.5, 0.0) > 0.7       # the wall face: one hit is strong evidence
    assert g.probability_at(5.0, 0.0) < 0.5       # seen through: leaning free after one pass...
    for _ in range(2):
        g.update((0.0, 0.0, 0.0), ANGLES, scan)
    assert g.probability_at(5.0, 0.0) < 0.3       # ...and confidently free once confirmed
    assert g.probability_at(9.5, 0.0) > 0.9
    assert g.probability_at(12.0, 0.0) == pytest.approx(0.5)  # behind the wall: never observed


def test_scan_matcher_recovers_a_known_pose_error():
    g = _grid()
    true_pose = (15.0, 2.0, 0.3)
    for p in [(10.0, 0.0, 0.0), (15.0, 2.0, 0.3), (18.0, 4.0, 0.6)]:
        g.update(p, ANGLES, simulate_scan(WORLD, p, ANGLES))
    scan = simulate_scan(WORLD, true_pose, ANGLES)
    guess = (true_pose[0] + 0.4, true_pose[1] - 0.3, true_pose[2] + math.radians(1.5))
    found, score = g.match(guess, ANGLES, scan)
    assert math.dist(found[:2], true_pose[:2]) < 0.1
    assert abs(found[2] - true_pose[2]) < math.radians(0.4)
    assert score > 0.5


def _lawnmower(t):
    """A 70 s survey path through the site; returns (north, east, yaw)."""
    legs = [(5, -20), (5, 20), (15, 20), (15, -20), (25, -24), (45, -24), (48, 20)]
    seg = 10.0  # seconds per leg
    i = min(int(t // seg), len(legs) - 2)
    k = min(1.0, (t - i * seg) / seg)
    (n0, e0), (n1, e1) = legs[i], legs[i + 1]
    return n0 + (n1 - n0) * k, e0 + (e1 - e0) * k, math.atan2(e1 - e0, n1 - n0)


def _run(drift: bool, seed: int = 4):
    """Feed SLAM odometry with injected drift; return (slam errors, dead-reckoning errors, slam)."""
    rng = np.random.default_rng(seed)
    slam = Slam2D(OccupancyGrid(resolution=0.25, **BOUNDS))
    prev_true = _lawnmower(0.0)
    odom = prev_true
    slam_err, dr_err = [], []
    for k in range(1, 141):  # 10 Hz scans... at 0.5 s steps (70 s)
        t = k * 0.5
        true = _lawnmower(t)
        step = relative(prev_true, true)
        if drift:  # 4% scale error and 0.6 deg/s heading drift: dead reckoning walks away
            step = (step[0] * 1.04, step[1] * 1.04, step[2] + math.radians(0.3))
        step = (step[0] + rng.normal(0, 0.01), step[1] + rng.normal(0, 0.01), step[2] + rng.normal(0, 0.002))
        odom = compose(odom, step)
        prev_true = true
        scan = simulate_scan(WORLD, true, ANGLES, noise_std=0.02, rng=rng)
        pose = slam.step(odom, ANGLES, scan)
        slam_err.append(math.dist(pose[:2], true[:2]))
        dr_err.append(math.dist(odom[:2], true[:2]))
    return slam_err, dr_err, slam


def test_slam_holds_position_where_dead_reckoning_drifts_away():
    # The injected heading drift (0.6 deg/s) is far worse than a real EKF's; it is there to
    # make dead reckoning walk ~20 m off. Without loop closure SLAM still accumulates a little
    # error while mapping new ground: over seeds 0-9 its worst error is 0.21-0.83 m.
    slam_err, dr_err, slam = _run(drift=True)
    assert dr_err[-1] > 10.0, f"injected drift too small to prove anything ({dr_err[-1]:.2f} m)"
    assert max(slam_err) < 1.0, max(slam_err)
    assert max(slam_err) < 0.05 * dr_err[-1]
    assert slam_err[-1] < 1.0
    assert slam.stats["matched"] > 100


def test_the_map_puts_occupied_cells_on_real_surfaces():
    _, _, slam = _run(drift=True)
    occ = slam.grid.occupied_points(threshold=0.7)
    assert len(occ) > 300

    def dist_to_world(n, e):
        d = math.inf
        for s in WORLD:
            if isinstance(s, Box):
                dn = max(abs(n - s.north) - s.size_north / 2, 0.0)
                de = max(abs(e - s.east) - s.size_east / 2, 0.0)
                d = min(d, math.hypot(dn, de))
            else:
                d = min(d, abs(math.hypot(n - s.north, e - s.east) - s.radius))
        return d

    precision = np.mean([dist_to_world(n, e) < 0.5 for n, e in occ])
    assert precision > 0.95, precision


def test_without_a_map_yet_slam_trusts_odometry():
    slam = Slam2D(OccupancyGrid(resolution=0.25, **BOUNDS))
    empty = np.full(len(ANGLES), np.inf)
    assert slam.step((1.0, 2.0, 0.3), ANGLES, empty) == pytest.approx((1.0, 2.0, 0.3))
    assert slam.step((1.5, 2.0, 0.3), ANGLES, empty) == pytest.approx((1.5, 2.0, 0.3))
    assert slam.stats["matched"] == 0


def test_runs_are_reproducible():
    a, _, _ = _run(drift=True, seed=9)
    b, _, _ = _run(drift=True, seed=9)
    assert a == b


def test_level_scan_keeps_a_level_scan_and_drops_ground_hits_when_tilted():
    from px4_offboard.slam2d import level_scan
    ranges = simulate_scan(WORLD, (15.0, 2.0, 0.3), ANGLES)
    b, r = level_scan(ANGLES, ranges, 0.0, 0.0, height_m=3.0)
    assert np.allclose(b, ANGLES) and np.array_equal(np.isinf(r), np.isinf(ranges))
    assert np.allclose(r[np.isfinite(r)], ranges[np.isfinite(ranges)])
    # Pitched 10 deg nose-down: a beam at bearing a meets the ground at 3 / (sin 10 deg cos a).
    pitch = math.radians(-10.0)
    ahead = np.abs(ANGLES) < math.radians(60)
    ground = np.full(len(ANGLES), np.inf)
    ground[ahead] = 3.0 / (math.sin(math.radians(10.0)) * np.cos(ANGLES[ahead]))
    b, r = level_scan(ANGLES, ground, 0.0, pitch, height_m=3.0)
    assert np.isnan(r[ahead]).all()                       # ground, not a wall: ignored
    assert np.isinf(r[~ahead]).all()                      # beams that hit nothing stay no-return
    # A wall 5 m ahead seen through the same pitch is kept, at its horizontal distance.
    b, r = level_scan(np.array([0.0]), np.array([5.0 / math.cos(pitch)]), 0.0, pitch, height_m=3.0)
    assert r[0] == pytest.approx(5.0) and b[0] == pytest.approx(0.0)
    # Rolled right: right-hand beams dip; the bearing of a beam at +90 deg stays +90 deg.
    b, r = level_scan(np.array([math.pi / 2]), np.array([4.0]), math.radians(8), 0.0, height_m=3.0)
    assert b[0] == pytest.approx(math.pi / 2) and r[0] == pytest.approx(4.0 * math.cos(math.radians(8)))
    # Returns from the drone's own frame are ignored.
    _, r = level_scan(np.array([0.0]), np.array([0.3]), 0.0, 0.0, height_m=3.0)
    assert np.isnan(r[0])


def test_ignored_beams_change_nothing_and_no_return_beams_clear_only_so_far():
    g = OccupancyGrid(resolution=0.25, no_return_clear_m=10.0, **BOUNDS)
    ranges = np.full(len(ANGLES), np.inf)
    ranges[: len(ANGLES) // 2] = np.nan
    g.update((0.0, 0.0, 0.0), ANGLES, ranges)
    assert g.probability_at(5.0, 5.0) < 0.5       # right half (positive angles): cleared
    assert g.probability_at(5.0, -5.0) == pytest.approx(0.5)   # left half was NaN: untouched
    assert g.probability_at(15.0, 0.5) == pytest.approx(0.5)   # beyond the clear limit


def test_an_initial_pose_fixes_the_map_frame_and_odometry_is_used_relative_to_it():
    anchor = (1.0, 2.0, 0.25)
    slam = Slam2D(OccupancyGrid(resolution=0.25, **BOUNDS), initial_pose=anchor)
    empty = np.full(len(ANGLES), np.inf)
    assert slam.step((1.1, 2.1, 0.30), ANGLES, empty) == pytest.approx(anchor)
    # The next odometry step (1 m forward in odometry's frame) is applied from the anchor.
    nxt = compose((1.1, 2.1, 0.30), (1.0, 0.0, 0.0))
    assert slam.step(nxt, ANGLES, empty) == pytest.approx(compose(anchor, (1.0, 0.0, 0.0)))
