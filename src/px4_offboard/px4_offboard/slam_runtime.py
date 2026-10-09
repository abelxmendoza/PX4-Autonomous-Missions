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
