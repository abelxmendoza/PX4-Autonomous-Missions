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
