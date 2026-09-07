"""Pure pinhole-camera math for estimating bearing/elevation/range to a
detected fiducial marker. No OpenCV/rclpy imports, so the geometry itself is
unit-testable without a camera or a running node — mirrors
frame_transforms.py and camera_frame.py.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PinholeCamera:
    width_px: int
    height_px: int
    horizontal_fov_rad: float

    @property
    def focal_length_px(self) -> float:
        return (self.width_px / 2.0) / math.tan(self.horizontal_fov_rad / 2.0)

    @property
    def cx(self) -> float:
        return self.width_px / 2.0

    @property
    def cy(self) -> float:
        return self.height_px / 2.0


def bearing_elevation_deg(camera: PinholeCamera, pixel_x: float, pixel_y: float) -> tuple[float, float]:
    """Angle off the camera boresight to a pixel: (+bearing = right, +elevation = up)."""
    f = camera.focal_length_px
    bearing = math.degrees(math.atan2(pixel_x - camera.cx, f))
    elevation = math.degrees(math.atan2(camera.cy - pixel_y, f))
    return bearing, elevation


def range_m(camera: PinholeCamera, marker_size_m: float, apparent_size_px: float) -> float:
    """Distance to a marker of known physical size from its apparent pixel size.

    Pinhole similar-triangles: range = (real size / apparent size) * focal length.
    """
    if marker_size_m <= 0.0:
        raise ValueError(f"marker_size_m must be positive, got {marker_size_m}")
    if apparent_size_px <= 0.0:
        raise ValueError(f"apparent_size_px must be positive, got {apparent_size_px}")
    return marker_size_m * camera.focal_length_px / apparent_size_px
