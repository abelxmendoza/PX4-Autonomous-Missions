"""Glue for running slam2d online against PX4 SITL + Gazebo: time alignment and frames.

Everything here is pure (no Gazebo, no MAVSDK) so it can be tested; tools/slam_flight.py
does the I/O. Poses are (north, east, yaw) in NED metres/radians.
"""

from __future__ import annotations

import bisect
import math

from .slam2d import Pose, compose, relative


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def euler_from_quaternion(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """(roll, pitch, yaw) of a body-to-world quaternion (ZYX convention)."""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def gazebo_pose_to_ned(x: float, y: float, qw: float, qx: float, qy: float, qz: float) -> Pose:
    """A Gazebo model pose (ENU world, FLU body) as an NED (north, east, yaw) pose."""
    yaw_enu = euler_from_quaternion(qw, qx, qy, qz)[2]
    return (y, x, _wrap(math.pi / 2 - yaw_enu))


class PoseBuffer:
    """Timestamped poses; ``at(t)`` interpolates between the two that bracket t."""

    def __init__(self, max_age_s: float = 10.0, max_gap_s: float = 0.25) -> None:
        self.max_age_s, self.max_gap_s = max_age_s, max_gap_s
        self._t: list[float] = []
        self._p: list[Pose] = []

    def add(self, t: float, pose: Pose) -> None:
        if self._t and t <= self._t[-1]:
            return  # out of order or duplicate: keep the stream monotonic
        self._t.append(t)
        self._p.append(pose)
        cut = bisect.bisect_left(self._t, t - self.max_age_s)
        if cut:
            del self._t[:cut], self._p[:cut]

    def at(self, t: float) -> Pose | None:
        k = bisect.bisect_left(self._t, t)
        if k < len(self._t) and self._t[k] == t:
            return self._p[k]
        if k == 0 or k == len(self._t) or self._t[k] - self._t[k - 1] > self.max_gap_s:
            return None
        t0, t1 = self._t[k - 1], self._t[k]
        a, b = self._p[k - 1], self._p[k]
        f = (t - t0) / (t1 - t0)
        return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, _wrap(a[2] + _wrap(b[2] - a[2]) * f))


class DriftingOdometry:
    """Degrade a good pose stream into dead reckoning that drifts, the way odometry does
    without GPS: every increment is scaled and the heading picks up a steady bias. Used to
    show SLAM correcting drift on real simulated scans (PX4's own SITL estimate, with GPS,
    barely drifts at all)."""

    def __init__(self, scale: float = 1.0, yaw_drift_deg_per_s: float = 0.0) -> None:
        self.scale = scale
        self.yaw_rate = math.radians(yaw_drift_deg_per_s)
        self._last: tuple[float, Pose] | None = None
        self.pose: Pose | None = None

    def update(self, t: float, good: Pose) -> Pose:
        if self._last is None:
            self.pose = good
        else:
            t0, prev = self._last
            step = relative(prev, good)
            step = (step[0] * self.scale, step[1] * self.scale, step[2] + self.yaw_rate * (t - t0))
            self.pose = compose(self.pose, step)
        self._last = (t, good)
        return self.pose


def align_se2(source, target) -> tuple[float, float, float]:
    """Rigid 2-D transform (rotation, then translation) that best maps ``source`` points onto
    ``target`` in the least-squares sense (Horn/Umeyama, no scale). Returns (dn, de, dyaw);
    apply with compose((dn, de, dyaw), (n, e, 0)).

    SLAM's map frame is wherever its first pose put it, so a SLAM trajectory can be exact in
    shape and still be offset as a whole from ground truth. Trajectory error is reported
    both raw and after this alignment (the absolute trajectory error of the TUM RGB-D
    benchmark), with the frame offset itself reported separately.
    """
    import numpy as np

    s = np.asarray(source, dtype=float)[:, :2]
    g = np.asarray(target, dtype=float)[:, :2]
    sc, gc = s.mean(axis=0), g.mean(axis=0)
    h = (s - sc).T @ (g - gc)
    yaw = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    c, sn = math.cos(yaw), math.sin(yaw)
    rotated = np.array([c * sc[0] - sn * sc[1], sn * sc[0] + c * sc[1]])
    dn, de = gc - rotated
    return float(dn), float(de), yaw


def _inverse(p: Pose) -> Pose:
    return relative(p, (0.0, 0.0, 0.0))


class MapFrameTracker:
    """Keeps the transform from SLAM's map frame to the frame PX4 flies in (ROS's map->odom).

    SLAM's map frame is fixed by its first pose; PX4 flies in its own local frame. A route
    planned on the map must be converted before it can be sent as setpoints, and the
    conversion is learned from the pose pairs SLAM and PX4 report for the same scan:

    gps       PX4's positions are good (GPS) but its heading carries a few degrees of bias, so
              the rotation is fitted to the POSITION tracks (``align_se2``), which the heading
              bias cannot touch. Until the track has spread ``min_spread_m``, the latest pose
              pair is used instead.
    odometry  GPS-denied: PX4's frame itself drifts, and a setpoint must be where PX4's frame
              currently puts the target. The transform is the latest pose pair
              (px4 o map^-1), optionally smoothed over ``smoothing_s``.
    """

    def __init__(self, mode: str = "gps", min_spread_m: float = 5.0, smoothing_s: float = 1.0,
                 sample_every_s: float = 0.25) -> None:
        if mode not in ("gps", "odometry"):
            raise ValueError("mode must be 'gps' or 'odometry'")
        self.mode, self.min_spread_m, self.smoothing_s = mode, min_spread_m, smoothing_s
        self.sample_every_s = sample_every_s
        self._map_xy: list[tuple[float, float]] = []
        self._px4_xy: list[tuple[float, float]] = []
        self._last_sample_t: float | None = None
        self._pair: Pose | None = None        # latest px4 o map^-1
        self._smoothed: Pose | None = None
        self._last_t: float | None = None
        self._fit: Pose | None = None

    def update(self, t: float, map_pose: Pose, px4_pose: Pose) -> None:
        pair = compose(px4_pose, _inverse(map_pose))
        self._pair = pair
        if self._smoothed is None or self.smoothing_s <= 0 or self._last_t is None:
            self._smoothed = pair
        else:
            k = min(1.0, max(0.0, t - self._last_t) / self.smoothing_s)
            s = self._smoothed
            self._smoothed = (s[0] + (pair[0] - s[0]) * k, s[1] + (pair[1] - s[1]) * k,
                              _wrap(s[2] + _wrap(pair[2] - s[2]) * k))
        self._last_t = t
        if self.mode == "gps" and (self._last_sample_t is None or t - self._last_sample_t >= self.sample_every_s):
            self._last_sample_t = t
            self._map_xy.append((map_pose[0], map_pose[1]))
            self._px4_xy.append((px4_pose[0], px4_pose[1]))
            self._fit = None

    def spread_m(self) -> float:
        if len(self._map_xy) < 2:
            return 0.0
        import numpy as np

        xy = np.asarray(self._map_xy)
        return float(np.hypot(*(xy - xy.mean(axis=0)).T).max())

    def transform(self) -> Pose | None:
        """map -> PX4 frame as (dn, de, dyaw), or None before the first update."""
        if self._pair is None:
            return None
        if self.mode == "gps" and self.spread_m() >= self.min_spread_m:
            if self._fit is None:
                self._fit = align_se2(self._map_xy, self._px4_xy)
            return self._fit
        return self._smoothed

    def to_px4(self, point_map: tuple[float, float]) -> tuple[float, float]:
        n, e, _ = compose(self.transform(), (point_map[0], point_map[1], 0.0))
        return n, e

    def to_map(self, point_px4: tuple[float, float]) -> tuple[float, float]:
        n, e, _ = compose(_inverse(self.transform()), (point_px4[0], point_px4[1], 0.0))
        return n, e
