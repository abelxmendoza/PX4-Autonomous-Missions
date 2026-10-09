"""Where on the ground is the marker the downward camera just saw?

Pure geometry plus a small per-target aggregator; no ROS, no Gazebo. Inputs are what a
real vehicle has: its own position/attitude estimate (PX4's, NED / FRD) and a pixel.
It never uses simulator ground truth.

Camera mount (models/x500_search_cam): pitched +90 deg about body y, so the optical axis
points down, image up = drone forward and image right = drone right. A pixel (u, v)
therefore looks along, in body FRD,

    d_body = ( -(v - cy) / f,  (u - cx) / f,  1 )

which is rotated into NED with the vehicle attitude and intersected with the ground plane
(down = ground_down, 0 for the flat search field). Assumptions: flat ground, camera at the
body origin (it is 5 cm below it), no lens distortion (Gazebo renders an ideal pinhole).
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from .ekf_fusion import quat_to_rotation_matrix
from .vision_marker import PinholeCamera

# Must match models/x500_search_cam/model.sdf (test_search_drone_model checks the SDF side).
SEARCH_CAMERA = PinholeCamera(width_px=640, height_px=480, horizontal_fov_rad=1.74)


def quat_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """(w, x, y, z) for body FRD -> NED, ZYX (yaw, pitch, roll) order, PX4's convention."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def _ray_ned(camera: PinholeCamera, q: Sequence[float], u: float, v: float) -> np.ndarray:
    f = camera.focal_length_px
    d_body = np.array([-(v - camera.cy) / f, (u - camera.cx) / f, 1.0])
    return quat_to_rotation_matrix(np.asarray(q, dtype=float)) @ d_body


def geolocate(
    camera: PinholeCamera,
    position_ned: Sequence[float],
    q: Sequence[float],
    u: float,
    v: float,
    ground_down: float = 0.0,
) -> tuple[float, float] | None:
    """Ground (north, east) seen at pixel (u, v), or None if that ray never reaches the
    ground in front of the camera (looking above the horizon, or camera not above ground)."""
    p = np.asarray(position_ned, dtype=float)
    height = ground_down - p[2]  # metres above the ground plane
    if height <= 0.0:
        return None
    ray = _ray_ned(camera, q, u, v)
    if ray[2] <= 1e-9:
        return None
    t = height / ray[2]
    hit = p + t * ray
    return float(hit[0]), float(hit[1])


def project_ground_point(
    camera: PinholeCamera,
    position_ned: Sequence[float],
    q: Sequence[float],
    ground_ne: Sequence[float],
    ground_down: float = 0.0,
) -> tuple[float, float] | None:
    """Inverse of geolocate: the pixel where a ground point appears (None if behind)."""
    p = np.asarray(position_ned, dtype=float)
    world = np.array([ground_ne[0], ground_ne[1], ground_down]) - p
    body = quat_to_rotation_matrix(np.asarray(q, dtype=float)).T @ world
    if body[2] <= 1e-9:
        return None
    f = camera.focal_length_px
    return camera.cx + f * body[1] / body[2], camera.cy - f * body[0] / body[2]


@dataclass
class TargetEstimate:
    marker_id: int
    north: float
    east: float
    detections: int
    spread_m: float  # largest distance of any single detection from the estimate
    first_seen_s: float


@dataclass
class TargetTracker:
    """Collects geolocated detections per marker id; a target is *confirmed* (reported
    once) after ``min_detections``. The estimate is the per-axis median, so one wild
    detection does not move it; the spread keeps such outliers visible."""

    min_detections: int = 3
    _points: dict[int, list[tuple[float, float, float]]] = field(default_factory=dict)
    _reported: set[int] = field(default_factory=set)

    def add(self, marker_id: int, north: float, east: float, t_s: float) -> list[int]:
        self._points.setdefault(marker_id, []).append((north, east, t_s))
        if marker_id not in self._reported and len(self._points[marker_id]) >= self.min_detections:
            self._reported.add(marker_id)
            return [marker_id]
        return []

    def estimates(self) -> dict[int, TargetEstimate]:
        out = {}
        for mid, pts in self._points.items():
            n = statistics.median(p[0] for p in pts)
            e = statistics.median(p[1] for p in pts)
            spread = max(math.dist((n, e), p[:2]) for p in pts)
            out[mid] = TargetEstimate(mid, n, e, len(pts), spread, min(p[2] for p in pts))
        return out

    def confirmed(self) -> dict[int, TargetEstimate]:
        return {mid: est for mid, est in self.estimates().items() if mid in self._reported}
