#!/usr/bin/env python3
"""Fly the obstacle course with the 2-D LiDAR drone and build a SLAM map and costmap online.

    GZ_IP=127.0.0.1 python tools/slam_flight.py [--altitude 3] [--speed 2] [--trace out.json]

PX4 SITL must already be running obstacle_world with gz_x500_lidar_2d
(scripts/run_slam_demo.sh starts everything). The route is a fixed loop around and through
the course, flown with MAVSDK. While it flies, every real Gazebo scan (gz-transport) goes
through two SLAM instances:

  A  odometry = PX4's own estimate. In SITL with GPS this is near-perfect, so A is the
     baseline: SLAM must not make a good estimate worse.
  B  odometry = PX4's estimate degraded on purpose (--drift-scale, --drift-deg-per-s), the
     way dead reckoning drifts without GPS. B shows what scan matching buys.

B's costmap and map are drawn in the Gazebo window (GUI-only markers), with its
dead-reckoning trail in red and its SLAM trail in green. Gazebo's ground-truth pose is
recorded ONLY to score the run; neither SLAM instance reads it. After landing, a route is
planned on B's costmap with graded costs and with binary inflation, for comparison.
"""
from __future__ import annotations

import os

# Must precede every protobuf import (MAVSDK's included); see search_demo_flight.py.
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
os.environ.setdefault("GZ_IP", "127.0.0.1")

import argparse  # noqa: E402
import asyncio  # noqa: E402
import collections  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import sys  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
from pathlib import Path  # noqa: E402

import numpy as np  # noqa: E402
from mavsdk import System  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "px4_offboard"))
from px4_offboard.costmap2d import Costmap, plan_on_costmap  # noqa: E402
from px4_offboard.costmap_markers import GazeboMapDisplay  # noqa: E402
from px4_offboard.gz_trail import build_trail_marker  # noqa: E402
from px4_offboard.mission_logic import DEFAULT_OBSTACLE_COURSE  # noqa: E402
from px4_offboard.slam2d import OccupancyGrid, Slam2D, compose, level_scan  # noqa: E402
from px4_offboard.slam_runtime import (DriftingOdometry, PoseBuffer, align_se2,  # noqa: E402
                                       euler_from_quaternion, gazebo_pose_to_ned)

WORLD = "obstacle_world"
MODEL = "x500_lidar_2d_0"
SCAN_TOPIC = f"/world/{WORLD}/model/{MODEL}/link/link/sensor/lidar_2d_v2/scan"
POSE_TOPIC = f"/world/{WORLD}/dynamic_pose/info"
LIDAR_FORWARD_M = 0.12      # lidar_2d_v2 sits 12 cm ahead of the x500's base_link
LIDAR_ABOVE_BASE_M = 0.31   # 0.26 mount + 0.055 sensor
# PX4's odometry position leads Gazebo's pose by 0.107 s x velocity (measured on both axes),
# while its attitude is on time (heading error shows no correlation with yaw rate). So only
# position samples are re-stamped by this much before aligning with scans; shifting attitude
# too was tried and put the tilt correction 0.1 s off in turns. Each report re-measures both
# ("clock_check").
PX4_POSITION_LEAD_S = 0.107
EARTH_M_PER_DEG = 111_320.0
# (north, east) loop: down the west side, across the far end, back up the east side, then
# through the middle and home. Every leg keeps >= 3.5 m from every course obstacle.
ROUTE = [(4.0, 0.0), (4.0, -14.0), (31.0, -14.0), (46.0, -9.0), (46.0, 9.0), (31.0, 15.0),
         (4.0, 15.0), (4.0, 0.0), (30.0, 0.0), (4.0, 0.0)]
GRID = dict(resolution=0.25, north_min=-12.0, north_max=62.0, east_min=-32.0, east_max=32.0)
PLAN_START, PLAN_GOAL = (2.0, 0.0), (46.0, 0.0)
# What a horizontal scan at ~3.3 m can hit in obstacle_world.sdf, for SCORING the map only:
# the five course boxes, plus (from landscape_landmarks) four tree crowns and the shelter
# roof. Boxes are (north, east, size_north, size_east); trees are (north, east, radius).
TRUTH_BOXES = [(o.north, o.east, o.size_north, o.size_east) for o in DEFAULT_OBSTACLE_COURSE] + [
    (-2.0, 22.0, 5.5, 7.5)]
TRUTH_TREES = [(8.0, -28.0, 1.8), (31.0, -25.0, 2.1), (18.0, 27.0, 1.9), (43.0, 30.0, 2.2)]


def offset(lat: float, lon: float, north: float, east: float) -> tuple[float, float]:
    return (lat + north / EARTH_M_PER_DEG,
            lon + east / (EARTH_M_PER_DEG * math.cos(math.radians(lat))))


class Inputs:
    """Latest data from Gazebo and PX4, filled by transport threads and the MAVSDK loop."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        # Recent scans, (sim time, angles NED, ranges). A scan is used once both pose streams
        # have a sample after it: the buffers interpolate but never extrapolate.
        self.scans: collections.deque = collections.deque(maxlen=20)
        self.truth = PoseBuffer(max_age_s=20.0, max_gap_s=0.5)
        self.position = PoseBuffer(max_age_s=20.0, max_gap_s=0.5)   # (north, east, 0)
        self.heading = PoseBuffer(max_age_s=20.0, max_gap_s=0.5)    # (0, 0, yaw)
        # (height, roll, pitch): PoseBuffer wraps the third element as an angle, so it must be
        # one. Storing height there turned 3.3 m into 3.3 - 2*pi = -2.98 m.
        self.attitude = PoseBuffer(max_age_s=20.0, max_gap_s=0.5)
        self.px4_time = 0.0
        self.scans_seen = 0

    def on_scan(self, msg) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        # Gazebo angles are counter-clockwise (left) positive; slam2d's are right positive.
        angles = -(msg.angle_min + msg.angle_step * np.arange(msg.count))
        ranges = np.array(msg.ranges, dtype=float)
        ranges[ranges >= msg.range_max] = np.inf
        with self.lock:
            self.scans.append((t, angles, ranges))
            self.scans_seen += 1

    def on_pose(self, msg) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        for p in msg.pose:
            if p.name == MODEL:
                o = p.orientation
                pose = gazebo_pose_to_ned(p.position.x, p.position.y, o.w, o.x, o.y, o.z)
                with self.lock:
                    self.truth.add(t, pose)
                return


class SlamRun:
    def __init__(self, name: str, drift: DriftingOdometry | None, anchor) -> None:
        self.name = name
        self.drift = drift
        self.slam = Slam2D(OccupancyGrid(**GRID, no_return_clear_m=12.0), initial_pose=anchor)
        self.samples: list[list[float]] = []   # t, truth(3), odom(3), slam(3)
        self.first_seen: dict[int, float] = {}

    def step(self, t: float, px4_sensor_pose, angles, ranges, truth_sensor_pose) -> None:
        odom = self.drift.update(t, px4_sensor_pose) if self.drift else px4_sensor_pose
        pose = self.slam.step(odom, angles, ranges)
        self.samples.append([round(t, 3), *(round(v, 4) for v in truth_sensor_pose),
                             *(round(v, 4) for v in odom), *(round(v, 4) for v in pose)])

    def note_map(self, t: float) -> None:
        g = self.slam.grid
        for idx in np.flatnonzero(g.probabilities().reshape(-1) > 0.7):
            self.first_seen.setdefault(int(idx), round(t, 2))

    def alignment(self):
        a = np.array(self.samples)
        return align_se2(a[:, 7:9], a[:, 1:3])

    def errors(self) -> dict:
        a = np.array(self.samples)
        if len(a) == 0:
            return {}
        slam = np.hypot(a[:, 7] - a[:, 1], a[:, 8] - a[:, 2])
        odom = np.hypot(a[:, 4] - a[:, 1], a[:, 5] - a[:, 2])
        yaw = np.degrees(np.abs((a[:, 9] - a[:, 3] + np.pi) % (2 * np.pi) - np.pi))
        al = self.alignment()
        aligned = np.array([compose(al, (n, e, 0.0))[:2] for n, e in a[:, 7:9]])
        ate = np.hypot(aligned[:, 0] - a[:, 1], aligned[:, 1] - a[:, 2])
        return {"scans": len(a), "matched": self.slam.stats["matched"],
                "slam_mean_m": round(float(slam.mean()), 3), "slam_max_m": round(float(slam.max()), 3),
                "slam_final_m": round(float(slam[-1]), 3), "slam_yaw_max_deg": round(float(yaw.max()), 2),
                "slam_ate_aligned_mean_m": round(float(ate.mean()), 3),
                "slam_ate_aligned_max_m": round(float(ate.max()), 3),
                "map_frame_offset": {"yaw_deg": round(math.degrees(al[2]), 2),
                                     "north_m": round(al[0], 3), "east_m": round(al[1], 3)},
                "odometry_mean_m": round(float(odom.mean()), 3), "odometry_final_m": round(float(odom[-1]), 3)}


def _box_dist(n: float, e: float, box) -> float:
    bn, be, sn, se = box
    return math.hypot(max(abs(n - bn) - sn / 2, 0.0), max(abs(e - be) - se / 2, 0.0))


def _tree_dist(n: float, e: float, tree) -> float:
    return abs(math.hypot(n - tree[0], e - tree[1]) - tree[2])


def map_precision(grid: OccupancyGrid, alignment=(0.0, 0.0, 0.0)) -> dict:
    """Occupied map cells vs the real surfaces in the world (ground truth, scoring only),
    as built and after the run's frame alignment."""
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


def clock_check(run: SlamRun) -> dict:
    """Residual lag between PX4's (offset-corrected) odometry and Gazebo time: PX4 position
    minus truth regressed on velocity. Near zero when PX4_CLOCK_OFFSET_S is right."""
    a = np.array(run.samples)
    if len(a) < 10:
        return {}
    out = {"px4_position_lead_corrected_s": PX4_POSITION_LEAD_S}
    for axis, name in ((1, "north"), (2, "east")):
        v = np.gradient(a[:, axis], a[:, 0])
        lag = np.linalg.lstsq(np.column_stack([v, np.ones_like(v)]), a[:, axis + 3] - a[:, axis], rcond=None)[0][0]
        out[f"residual_lag_{name}_s"] = round(float(lag), 3)
    rate = np.gradient(np.unwrap(a[:, 3]), a[:, 0])
    dyaw = (a[:, 6] - a[:, 3] + np.pi) % (2 * np.pi) - np.pi
    lag, bias = np.linalg.lstsq(np.column_stack([rate, np.ones_like(rate)]), dyaw, rcond=None)[0]
    out["residual_lag_heading_s"] = round(float(lag), 3)
    out["px4_heading_bias_deg"] = round(math.degrees(float(bias)), 2)
    return out


def slam_worker(inputs: Inputs, runs: list[SlamRun], active: threading.Event, stop: threading.Event,
                display: GazeboMapDisplay | None, rate_hz: float, stats: dict, raw: list | None) -> None:
    last_t, last_draw = -1.0, 0.0
    drawer: threading.Thread | None = None
    while not stop.is_set():
        time.sleep(1.0 / rate_hz)
        with inputs.lock:
            if not active.is_set():
                continue
            ready = None
            for scan in reversed(inputs.scans):
                if scan[0] <= last_t:
                    break
                pos, hdg = inputs.position.at(scan[0]), inputs.heading.at(scan[0])
                aligned = (None if pos is None or hdg is None else (pos[0], pos[1], hdg[2]),
                           inputs.attitude.at(scan[0]), inputs.truth.at(scan[0]))
                if None not in aligned:
                    ready = (scan, aligned)
                    break
        if ready is None:
            stats["waits"] += 1
            continue
        (t, angles, ranges), (px4, att, truth) = ready
        last_t = t
        height, roll, pitch = att
        if raw is not None:
            raw.append((t, np.asarray(angles, np.float32), np.asarray(ranges, np.float32), px4, att, truth))
        bearings, flat = level_scan(angles, ranges, roll, pitch, height)
        lever = (LIDAR_FORWARD_M, 0.0, 0.0)
        for run in runs:
            run.step(t, compose(px4, lever), bearings, flat, compose(truth, lever))
        stats["processed"] += 1
        if time.monotonic() - last_draw > 2.0:
            last_draw = time.monotonic()
            shown = runs[-1]
            shown.note_map(t)
            if display is not None and (drawer is None or not drawer.is_alive()):
                # Snapshot, then draw on another thread: a slow GUI must never stall SLAM.
                costmap = Costmap(**GRID, robot_radius_m=0.6, inflation_radius_m=2.5)
                costmap.set_occupancy(shown.slam.grid)
                costmap.costs()
                points = shown.slam.grid.occupied_points(0.7)
                trail = np.array(shown.samples)[::5]
                drawer = threading.Thread(target=draw, args=(display, costmap, points, trail, height, stats),
                                          daemon=True)
                drawer.start()


def draw(display: GazeboMapDisplay, costmap: Costmap, points, trail: np.ndarray, height: float,
         stats: dict) -> None:
    try:
        display.publish(costmap, points, height)
        for mid, cols, rgb in ((1, (4, 5), (0.9, 0.2, 0.2)), (2, (7, 8), (0.2, 0.9, 0.4))):
            m = build_trail_marker([(n, e, -height) for n, e in trail[:, cols]], marker_id=mid, rgb=rgb,
                                   ns="slam_trails")
            if m is not None:
                display._send(m)
    except Exception as exc:  # display is best effort
        stats["display_errors"] = f"{exc}"


async def fly(args) -> None:
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    inputs = Inputs()
    node = Node()
    node.subscribe(LaserScan, SCAN_TOPIC, inputs.on_scan)
    node.subscribe(Pose_V, POSE_TOPIC, inputs.on_pose)
    display = None
    if os.environ.get("HEADLESS"):
        print("(HEADLESS: no map display)", flush=True)
    else:
        try:
            display = GazeboMapDisplay()
            # The Gazebo window can take a while after PX4 is up; keep asking for a minute.
            deadline = time.monotonic() + 60.0
            while not display.available():
                if time.monotonic() > deadline:
                    print("(no Gazebo window answering /marker: running without the map display)", flush=True)
                    display = None
                    break
            if display is not None:
                print("Gazebo window found: drawing costmap, map and trails", flush=True)
        except Exception as exc:
            print(f"(no Gazebo display: {exc})", flush=True)
            display = None
    runs: list[SlamRun] = []
    active, stop_worker = threading.Event(), threading.Event()
    stats = {"processed": 0, "waits": 0}
    raw: list | None = [] if args.raw else None
    worker = threading.Thread(target=slam_worker, daemon=True,
                              args=(inputs, runs, active, stop_worker, display, args.rate, stats, raw))
    worker.start()

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    print("waiting for PX4 ...", flush=True)
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            break
    home = await anext(drone.telemetry.home())
    try:
        await drone.telemetry.set_rate_odometry(50.0)
    except Exception as exc:
        print(f"(odometry rate request refused: {exc})", flush=True)
    stop = asyncio.Event()

    async def follow():
        async for od in drone.telemetry.odometry():
            if stop.is_set():
                break
            t = od.time_usec * 1e-6
            q = od.q
            roll, pitch, yaw = euler_from_quaternion(q.w, q.x, q.y, q.z)
            p = od.position_body
            with inputs.lock:
                inputs.position.add(t + PX4_POSITION_LEAD_S, (p.x_m, p.y_m, 0.0))
                inputs.heading.add(t, (0.0, 0.0, yaw))
                inputs.attitude.add(t, (-p.z_m + LIDAR_ABOVE_BASE_M, roll, pitch))
                inputs.px4_time = t

    follower = asyncio.create_task(follow())
    await drone.action.set_takeoff_altitude(args.altitude)
    await drone.action.arm()
    await drone.action.takeoff()
    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m > args.altitude - 0.3:
            break
    await drone.action.set_current_speed(args.speed)
    # Fix the map frame from 2 s of hover: one heading sample carries ~1.5 deg of EKF noise,
    # and SLAM can never rotate its frame back afterwards.
    headings, positions = [], []
    hover_end = time.monotonic() + 2.0
    while time.monotonic() < hover_end:
        with inputs.lock:
            if inputs.position._p and inputs.heading._p:
                positions.append((inputs.position._p[-1][0], inputs.position._p[-1][1], inputs.heading._p[-1][2]))
        await asyncio.sleep(0.05)
    headings = [p[2] for p in positions]
    mean_yaw = math.atan2(sum(math.sin(h) for h in headings), sum(math.cos(h) for h in headings))
    anchor = compose((positions[-1][0], positions[-1][1], mean_yaw), (LIDAR_FORWARD_M, 0.0, 0.0))
    runs += [SlamRun("px4_odometry", None, anchor),
             SlamRun("drifting_odometry", DriftingOdometry(args.drift_scale, args.drift_deg_per_s), anchor)]
    with inputs.lock:
        print(f"clock check: last scan {inputs.scans[-1][0] if inputs.scans else float('nan'):.2f} s, "
              f"PX4 odometry {inputs.px4_time:.2f} s (sim time; must agree)", flush=True)
    active.set()
    t_start = time.monotonic()

    target_amsl = home.absolute_altitude_m + args.altitude
    prev = (0.0, 0.0)
    for k, (n, e) in enumerate(ROUTE, start=1):
        lat, lon = offset(home.latitude_deg, home.longitude_deg, n, e)
        yaw = math.degrees(math.atan2(e - prev[1], n - prev[0])) if (n, e) != prev else 0.0
        prev = (n, e)
        print(f"leg {k}/{len(ROUTE)}: to north {n:+.0f} m, east {e:+.0f} m  "
              f"(scans used {stats['processed']}, {runs[-1].slam.stats['matched']} matched)", flush=True)
        await drone.action.goto_location(lat, lon, target_amsl, yaw)
        async for pos in drone.telemetry.position():
            dn = (pos.latitude_deg - lat) * EARTH_M_PER_DEG
            de = (pos.longitude_deg - lon) * EARTH_M_PER_DEG * math.cos(math.radians(lat))
            if math.hypot(dn, de) < 1.0:
                break

    active.clear()
    flight_s = time.monotonic() - t_start
    print("route done, landing", flush=True)
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break
    print("landed", flush=True)
    stop.set()
    stop_worker.set()
    worker.join(timeout=5.0)
    stats["received"] = inputs.scans_seen
    await asyncio.wait_for(follower, timeout=10.0)
    finish(args, runs, stats, flight_s, display)
    if raw:
        args.raw.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(args.raw, t=np.array([r[0] for r in raw]), angles=np.stack([r[1] for r in raw]),
                            ranges=np.stack([r[2] for r in raw]), px4=np.array([r[3] for r in raw]),
                            attitude=np.array([r[4] for r in raw]), truth=np.array([r[5] for r in raw]))
        print(f"raw scans: {args.raw} ({len(raw)})", flush=True)


def finish(args, runs: list[SlamRun], stats: dict, flight_s: float, display) -> None:
    shown = runs[-1]
    shown.note_map(shown.samples[-1][0] if shown.samples else 0.0)
    cm = Costmap(**GRID, robot_radius_m=0.6, inflation_radius_m=2.5)
    cm.set_occupancy(shown.slam.grid)
    if display is not None:
        try:
            display.publish(cm, shown.slam.grid.occupied_points(0.7), args.altitude + LIDAR_ABOVE_BASE_M)
        except Exception:
            pass
    plans = {}
    for name, weight in (("graded", 3.0), ("binary", 0.0)):
        try:
            plans[name] = [[round(n, 2), round(e, 2)] for n, e in
                           plan_on_costmap(cm, PLAN_START, PLAN_GOAL, cost_weight=weight, allow_unknown=False)]
        except (RuntimeError, ValueError) as exc:
            plans[name] = f"no route: {exc}"
    report = {
        "world": WORLD, "vehicle": "x500_lidar_2d (PX4 SITL, Gazebo Harmonic)",
        "sensor": "lidar_2d_v2: 720 beams over 270 deg, 30 m, 20 Hz (real Gazebo GPU-lidar scans)",
        "altitude_m": args.altitude, "speed_mps": args.speed, "flight_s": round(flight_s, 1),
        "slam_rate_hz": args.rate, "scans_received": stats.get("received"), "scans_processed": stats["processed"],
        "ground_truth": "Gazebo model pose, used only for these scores; SLAM never reads it",
        "runs": {r.name: r.errors() for r in runs},
        "drift_injected": {"scale": args.drift_scale, "yaw_deg_per_s": args.drift_deg_per_s},
        "map": {r.name: map_precision(r.slam.grid, r.alignment()) for r in runs},
        "clock_check": clock_check(runs[0]),
        "plans": {"start": PLAN_START, "goal": PLAN_GOAL, **{k: (v if isinstance(v, str) else len(v))
                                                             for k, v in plans.items()}},
    }
    print(json.dumps(report, indent=2), flush=True)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n")
    if args.trace:
        g = shown.slam.grid
        trace = {
            "report": report,
            "grid": {"resolution": g.res, "north_min": g.north_min, "east_min": g.east_min,
                     "rows": g.rows, "cols": g.cols},
            "columns": ["t", "truth_n", "truth_e", "truth_yaw", "odom_n", "odom_e", "odom_yaw",
                        "slam_n", "slam_e", "slam_yaw"],
            "samples": {r.name: r.samples for r in runs},
            "map_cells": sorted([idx, t] for idx, t in shown.first_seen.items()),
            "costmap": cm.to_dict(),
            "plans": plans,
            "obstacles_truth": [{"north": o.north, "east": o.east, "size_north": o.size_north,
                                 "size_east": o.size_east, "height": o.height} for o in DEFAULT_OBSTACLE_COURSE],
            "route": ROUTE,
        }
        args.trace.parent.mkdir(parents=True, exist_ok=True)
        args.trace.write_text(json.dumps(trace, separators=(",", ":")) + "\n")
        print(f"trace: {args.trace}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--altitude", type=float, default=3.0)
    parser.add_argument("--speed", type=float, default=2.0)
    parser.add_argument("--rate", type=float, default=5.0, help="SLAM updates per second")
    parser.add_argument("--drift-scale", type=float, default=1.03)
    parser.add_argument("--drift-deg-per-s", type=float, default=0.5)
    parser.add_argument("--report", type=Path, default=Path("/tmp/slam_demo/slam_report.json"))
    parser.add_argument("--trace", type=Path, help="also save the run for the browser replay")
    parser.add_argument("--raw", type=Path, help="also save every processed scan with its poses (.npz)")
    args = parser.parse_args()
    asyncio.run(fly(args))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
