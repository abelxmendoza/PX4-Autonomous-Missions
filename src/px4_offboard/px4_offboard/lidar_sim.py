"""Synthetic horizontal 2-D LiDAR over axis-aligned boxes and circles (NED, metres).

For testing the SLAM and costmap code against a known world, before a Gazebo world
exists. Beam angles are relative to the vehicle heading, positive to the right (the NED
convention: yaw grows from north toward east). A beam that hits nothing within range
returns +inf (no return), as a real LiDAR reports an out-of-range reading.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Union

import numpy as np

# The x500 LiDAR in models/lidar_2d_v2: 720 rays over 270 degrees, 0.1-30 m.
X500_MIN_ANGLE = -2.356195
X500_MAX_ANGLE = 2.356195
X500_SAMPLES = 720
X500_MAX_RANGE = 30.0


@dataclass(frozen=True)
class Box:
    north: float
    east: float
    size_north: float
    size_east: float


@dataclass(frozen=True)
class Circle:
    north: float
    east: float
    radius: float


Shape = Union[Box, Circle]


def scan_angles(samples: int = X500_SAMPLES, lo: float = X500_MIN_ANGLE, hi: float = X500_MAX_ANGLE) -> np.ndarray:
    return np.linspace(lo, hi, samples)


def _ray_box(on: float, oe: float, dn: float, de: float, b: Box) -> float:
    t_near, t_far = -math.inf, math.inf
    for o, d, lo, hi in ((on, dn, b.north - b.size_north / 2, b.north + b.size_north / 2),
                         (oe, de, b.east - b.size_east / 2, b.east + b.size_east / 2)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return math.inf
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        t_near, t_far = max(t_near, t1), min(t_far, t2)
    if t_near > t_far or t_near <= 0.0:  # missed, or starting inside the box
        return math.inf
    return t_near


def _ray_circle(on: float, oe: float, dn: float, de: float, c: Circle) -> float:
    fn, fe = on - c.north, oe - c.east
    b = fn * dn + fe * de
    disc = b * b - (fn * fn + fe * fe - c.radius ** 2)
    if disc < 0:
        return math.inf
    t = -b - math.sqrt(disc)
    return t if t > 0.0 else math.inf


def simulate_scan(
    shapes: Sequence[Shape],
    pose: Sequence[float],
    angles: np.ndarray,
    max_range: float = X500_MAX_RANGE,
    noise_std: float = 0.0,
    rng: np.random.Generator | None = None,
) -> np.ndarray:
    """Ranges for each beam from pose (north, east, yaw_rad)."""
    n0, e0, yaw = pose
    ranges = np.full(len(angles), math.inf)
    for i, a in enumerate(angles):
        dn, de = math.cos(yaw + a), math.sin(yaw + a)
        best = math.inf
        for s in shapes:
            t = _ray_box(n0, e0, dn, de, s) if isinstance(s, Box) else _ray_circle(n0, e0, dn, de, s)
            best = min(best, t)
        if best <= max_range:
            ranges[i] = best
    if noise_std > 0.0:
        rng = rng or np.random.default_rng()
        hit = np.isfinite(ranges)
        ranges[hit] += rng.normal(0.0, noise_std, hit.sum())
    return ranges
