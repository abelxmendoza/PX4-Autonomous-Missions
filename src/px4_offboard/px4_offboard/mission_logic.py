"""Pure mission geometry used by the ROS 2 offboard controller.

This module intentionally has no ROS or PX4 dependencies.  Keeping the
calculations deterministic makes them usable from unit tests, simulations,
and a future C++ implementation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence


Vector3 = Sequence[float]


@dataclass(frozen=True)
class Obstacle:
    """Axis-aligned obstacle expressed in ENU horizontally."""

    east: float
    north: float
    size_east: float
    size_north: float
    height: float


@dataclass(frozen=True)
class Fence:
    """NED keep-in bounds with a positive maximum altitude."""

    north_min: float
    north_max: float
    east_min: float
    east_max: float
    altitude_max: float

    def __post_init__(self) -> None:
        if self.north_min > self.north_max:
            raise ValueError("north_min must not exceed north_max")
        if self.east_min > self.east_max:
            raise ValueError("east_min must not exceed east_max")
        if self.altitude_max < 0.0:
            raise ValueError("altitude_max must be non-negative")

    def contains(self, position: Vector3, altitude_tolerance: float = 0.05) -> bool:
        north, east, down = position
        altitude = -down
        return (
            self.north_min <= north <= self.north_max
            and self.east_min <= east <= self.east_max
            and altitude <= self.altitude_max + altitude_tolerance
        )

    def clamp(self, target: Vector3, margin: float = 0.0) -> tuple[list[float], bool]:
        """Clamp a NED target inside the fence and report whether it changed."""
        margin = max(0.0, margin)
        north_low, north_high = self.north_min + margin, self.north_max - margin
        east_low, east_high = self.east_min + margin, self.east_max - margin

        # A margin wider than the fence is invalid; retain the physical bounds.
        if north_low >= north_high:
            north_low, north_high = self.north_min, self.north_max
        if east_low >= east_high:
            east_low, east_high = self.east_min, self.east_max

        original = list(target)
        clamped = list(original)
        clamped[0] = min(max(clamped[0], north_low), north_high)
        clamped[1] = min(max(clamped[1], east_low), east_high)
        clamped[2] = max(clamped[2], -self.altitude_max)
        return clamped, clamped != original


def distance_3d(first: Vector3, second: Vector3) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def circle_target(
    elapsed_s: float, radius_m: float, period_s: float, altitude_m: float
) -> list[float]:
    if period_s <= 0.0:
        raise ValueError("period_s must be positive")
    angle = (2.0 * math.pi / period_s) * elapsed_s
    return [
        radius_m * math.cos(angle),
        radius_m * math.sin(angle),
        -altitude_m,
    ]


def yaw_toward(position: Vector3, target: Vector3, deadband_m: float = 0.1) -> float:
    delta_north = target[0] - position[0]
    delta_east = target[1] - position[1]
    if abs(delta_north) < deadband_m and abs(delta_east) < deadband_m:
        return float("nan")
    return math.atan2(delta_east, delta_north)


def detect_obstacle(
    position: Vector3,
    target: Vector3,
    obstacles: Iterable[Obstacle],
    detection_margin_m: float,
    front_angle_deg: float,
    side_angle_deg: float,
    detect_when_above: bool = False,
) -> str | None:
    """Classify the nearest relevant obstacle relative to direction of travel."""
    delta_north = target[0] - position[0]
    delta_east = target[1] - position[1]
    if abs(delta_north) < 1e-3 and abs(delta_east) < 1e-3:
        return None

    travel_yaw = math.degrees(math.atan2(delta_east, delta_north))
    altitude_agl = -position[2]
    best: tuple[float, str] | None = None

    for obstacle in obstacles:
        if not detect_when_above and altitude_agl > obstacle.height + 0.5:
            continue

        half_north = obstacle.size_north / 2.0
        half_east = obstacle.size_east / 2.0
        nearest_north = min(
            max(position[0], obstacle.north - half_north),
            obstacle.north + half_north,
        )
        nearest_east = min(
            max(position[1], obstacle.east - half_east),
            obstacle.east + half_east,
        )
        distance = math.hypot(
            nearest_north - position[0], nearest_east - position[1]
        )
        bearing_north = obstacle.north - position[0]
        bearing_east = obstacle.east - position[1]
        center_distance = math.hypot(bearing_north, bearing_east)
        trigger_distance = max(half_north, half_east) + detection_margin_m
        if distance > trigger_distance and center_distance > trigger_distance:
            continue

        bearing = math.degrees(math.atan2(bearing_east, bearing_north))
        relative_angle = (bearing - travel_yaw + 180.0) % 360.0 - 180.0
        if abs(relative_angle) < front_angle_deg:
            label = "front"
        elif 0.0 < relative_angle < side_angle_deg:
            label = "left"
        elif -side_angle_deg < relative_angle < 0.0:
            label = "right"
        else:
            continue

        if best is None or distance < best[0]:
            best = (distance, label)

    return best[1] if best else None


def blocking_height(
    position: Vector3,
    target: Vector3,
    obstacles: Iterable[Obstacle],
    detection_margin_m: float,
    side_angle_deg: float,
) -> float:
    """Return the tallest obstacle near the current travel corridor."""
    delta_north = target[0] - position[0]
    delta_east = target[1] - position[1]
    travel_yaw = (
        math.degrees(math.atan2(delta_east, delta_north))
        if abs(delta_north) + abs(delta_east) > 1e-3
        else 0.0
    )
    tallest = 0.0
    for obstacle in obstacles:
        bearing_north = obstacle.north - position[0]
        bearing_east = obstacle.east - position[1]
        distance = math.hypot(bearing_north, bearing_east)
        if distance > max(obstacle.size_north, obstacle.size_east) + detection_margin_m + 1.0:
            continue
        bearing = math.degrees(math.atan2(bearing_east, bearing_north))
        relative_angle = (bearing - travel_yaw + 180.0) % 360.0 - 180.0
        if abs(relative_angle) < side_angle_deg:
            tallest = max(tallest, obstacle.height)
    return tallest
