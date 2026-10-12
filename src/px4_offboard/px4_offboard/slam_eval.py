"""Scoring SLAM against the obstacle-course ground truth, and replaying recorded flights.

Shared by tools/slam_flight.py (live runs) and the regression test that replays a real
Gazebo LiDAR flight from test/data/. Ground truth here is for SCORING only: nothing in the
SLAM path reads it.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from .mission_logic import DEFAULT_OBSTACLE_COURSE
from .slam2d import OccupancyGrid, Slam2D, compose, level_scan, relative
from .slam_runtime import DriftingOdometry, MapFrameTracker, align_se2

LIDAR_FORWARD_M = 0.12  # lidar_2d_v2 sits 12 cm ahead of the x500's base_link
GRID = dict(resolution=0.25, north_min=-12.0, north_max=62.0, east_min=-32.0, east_max=32.0)
NO_RETURN_CLEAR_M = 12.0
# What a horizontal scan at ~3.3 m can hit in worlds/obstacle_world.sdf: the five course
# boxes, plus (from landscape_landmarks) four tree crowns and the shelter roof.
# Boxes are (north, east, size_north, size_east); trees are (north, east, radius).
TRUTH_BOXES = [(o.north, o.east, o.size_north, o.size_east) for o in DEFAULT_OBSTACLE_COURSE] + [
    (-2.0, 22.0, 5.5, 7.5)]
TRUTH_TREES = [(8.0, -28.0, 1.8), (31.0, -25.0, 2.1), (18.0, 27.0, 1.9), (43.0, 30.0, 2.2)]


def _box_dist(n: float, e: float, box) -> float:
    bn, be, sn, se = box
    return math.hypot(max(abs(n - bn) - sn / 2, 0.0), max(abs(e - be) - se / 2, 0.0))


def _tree_dist(n: float, e: float, tree) -> float:
    return abs(math.hypot(n - tree[0], e - tree[1]) - tree[2])


def map_precision(grid: OccupancyGrid, alignment=(0.0, 0.0, 0.0)) -> dict:
    """Occupied map cells vs the real surfaces in the world, as built and after the run's
    frame alignment."""
    pts = grid.occupied_points(0.7)
    shapes = [(_box_dist, b) for b in TRUTH_BOXES] + [(_tree_dist, t) for t in TRUTH_TREES]

    def on_surface(points):
        d = np.array([min(f(n, e, s) for f, s in shapes) for n, e in points]) if points else np.array([])
        return round(float((d <= 0.5).mean()), 3) if len(d) else None

    aligned = [compose(alignment, (n, e, 0.0))[:2] for n, e in pts]
    course = sum(1 for b in TRUTH_BOXES[:len(DEFAULT_OBSTACLE_COURSE)]
                 if any(_box_dist(n, e, b) <= 0.5 for n, e in aligned))
    return {"occupied_cells": len(pts),
            "within_0_5m_of_a_real_surface": on_surface(pts),
            "within_0_5m_after_frame_alignment": on_surface(aligned),
            "course_boxes_mapped": f"{course}/{len(DEFAULT_OBSTACLE_COURSE)}"}


def trajectory_errors(estimate, truth) -> dict:
    """Raw error, and error after one rigid fit of the estimate's frame onto truth (ATE)."""
    est, gt = np.asarray(estimate, dtype=float), np.asarray(truth, dtype=float)
    raw = np.hypot(est[:, 0] - gt[:, 0], est[:, 1] - gt[:, 1])
    al = align_se2(est, gt)
    moved = np.array([compose(al, (n, e, 0.0))[:2] for n, e in est[:, :2]])
    ate = np.hypot(moved[:, 0] - gt[:, 0], moved[:, 1] - gt[:, 1])
    return {"raw_mean_m": float(raw.mean()), "raw_max_m": float(raw.max()),
            "ate_mean_m": float(ate.mean()), "ate_max_m": float(ate.max()),
            "frame_yaw_deg": math.degrees(al[2]), "alignment": al}


def load_recorded_flight(path: str | Path) -> dict:
    """A flight recorded by tools/slam_flight.py --raw and packed for tests: base-link poses
    (PX4 estimate and Gazebo truth), (height, roll, pitch), one shared row of beam angles
    (NED, right positive) and ranges in whole centimetres (65535 = no return)."""
    d = np.load(path)
    ranges = d["ranges_cm"].astype(float) / 100.0
    ranges[d["ranges_cm"] == 65535] = np.inf
    return {"t": d["t"], "angles": d["angles"], "ranges": ranges, "px4": d["px4"],
            "attitude": d["attitude"], "truth": d["truth"]}


def registration_errors(transforms, world_of_px4, alignment, checkpoints, warmup: int = 20) -> dict:
    """How far a map point lands from where it really is once converted into PX4's frame and
    flown: for each world checkpoint w, x = alignment^-1(w) is where the map has it; at scan k
    it is sent to PX4 as T_k(x), which in the world is W_k(T_k(x)). Also reports the same
    with no transform (map coordinates sent to PX4 as they are). ``alignment`` (map->world)
    and W_k (PX4 frame->world at scan k) come from ground truth: scoring only. With GPS, W is
    one fit of PX4's position track onto the true track; GPS-denied, PX4's frame drifts and
    W_k is truth_k o odometry_k^-1 at each scan."""
    inv_al = relative(alignment, (0.0, 0.0, 0.0))
    xs = [compose(inv_al, (n, e, 0.0)) for n, e in checkpoints]
    with_t, without = [], []
    for k in range(warmup, len(transforms)):
        if transforms[k] is None:
            continue
        for (wn, we), x in zip(checkpoints, xs):
            got = compose(world_of_px4[k], compose(transforms[k], x))
            raw = compose(world_of_px4[k], x)
            with_t.append(math.hypot(got[0] - wn, got[1] - we))
            without.append(math.hypot(raw[0] - wn, raw[1] - we))
    return {"tracked_mean_m": float(np.mean(with_t)), "tracked_max_m": float(np.max(with_t)),
            "untransformed_mean_m": float(np.mean(without)), "untransformed_max_m": float(np.max(without))}


def world_of_px4_frames(odometry, truth, gps: bool) -> list:
    """PX4 frame -> world, per scan. With GPS, a position setpoint lands where PX4's position
    estimate says, whatever its heading error: one fit of PX4's position track onto the
    true track. GPS-denied, PX4's frame drifts: truth_k o odometry_k^-1 at each scan."""
    if gps:
        return [align_se2(odometry, truth)] * len(odometry)
    return [compose(g, relative(o, (0.0, 0.0, 0.0))) for o, g in zip(odometry, truth)]


def clearance_to_truth(route, step_m: float = 0.1) -> float:
    """Closest approach (metres) of a world-frame polyline to any real surface."""
    shapes = [(_box_dist, b) for b in TRUTH_BOXES] + [(_tree_dist, t) for t in TRUTH_TREES]
    best = math.inf
    for (n0, e0), (n1, e1) in zip(route, route[1:]):
        k = max(1, int(math.ceil(math.hypot(n1 - n0, e1 - e0) / step_m)))
        for i in range(k + 1):
            n, e = n0 + (n1 - n0) * i / k, e0 + (e1 - e0) * i / k
            best = min(best, min(f(n, e, x) for f, x in shapes))
    return best


# Registration checkpoints: the centres of the five course boxes (world frame).
CHECKPOINTS = [(o.north, o.east) for o in DEFAULT_OBSTACLE_COURSE]


def replay(flight: dict, drift: DriftingOdometry | None = None) -> dict:
    """Run SLAM over a recorded flight exactly as slam_flight.py does live; score it.
    The map->PX4 frame tracker runs in gps mode with PX4's estimate as odometry, and in
    odometry mode when the odometry is degraded (GPS-denied)."""
    lever = (LIDAR_FORWARD_M, 0.0, 0.0)
    slam = Slam2D(OccupancyGrid(**GRID, no_return_clear_m=NO_RETURN_CLEAR_M))
    tracker = MapFrameTracker(mode="odometry" if drift is not None else "gps")
    transforms = []
    est, gt, odo = [], [], []
    for k in range(len(flight["t"])):
        height, roll, pitch = flight["attitude"][k]
        bearings, flat = level_scan(flight["angles"], flight["ranges"][k], roll, pitch, height)
        odom = compose(tuple(flight["px4"][k]), lever)
        if drift is not None:
            odom = drift.update(float(flight["t"][k]), odom)
        est.append(slam.step(odom, bearings, flat))
        gt.append(compose(tuple(flight["truth"][k]), lever))
        odo.append(odom)
        tracker.update(float(flight["t"][k]), est[-1], odom)
        transforms.append(tracker.transform())
    world_of_px4 = world_of_px4_frames(odo, gt, gps=drift is None)
    errors = trajectory_errors(est, gt)
    odometry = trajectory_errors(odo, gt)
    return {"slam": slam, "errors": errors, "odometry_final_m": float(np.hypot(*(np.array(odo[-1][:2]) - gt[-1][:2]))),
            "odometry_raw_mean_m": odometry["raw_mean_m"],
            "map": map_precision(slam.grid, errors["alignment"]), "matched": slam.stats["matched"],
            "scans": len(est), "tracker": tracker,
            "registration": registration_errors(transforms, world_of_px4, errors["alignment"], CHECKPOINTS)}
