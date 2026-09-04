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


def obstacle_clearance(position: Vector3, obstacle: Obstacle) -> float:
    """Shortest distance from a NED point to an obstacle's solid AABB.

    Returns zero when the point lies inside the obstacle volume. The obstacle
    rests on the ground and extends upward to ``height``.
    """
    north, east, down = position
    altitude = -down
    delta_north = max(abs(north - obstacle.north) - obstacle.size_north / 2.0, 0.0)
    delta_east = max(abs(east - obstacle.east) - obstacle.size_east / 2.0, 0.0)
    if altitude < 0.0:
        delta_altitude = -altitude
    elif altitude > obstacle.height:
        delta_altitude = altitude - obstacle.height
    else:
        delta_altitude = 0.0
    return math.sqrt(
        delta_north * delta_north
        + delta_east * delta_east
        + delta_altitude * delta_altitude
    )


def sensor_bypass_target(
    position: Vector3,
    target: Vector3,
    obstacle_sector: str,
    forward_m: float,
    lateral_m: float,
    left_range_m: float = -1.0,
    right_range_m: float = -1.0,
) -> list[float]:
    """Create a committed local bypass without using mapped geometry."""
    delta_n = target[0] - position[0]
    delta_e = target[1] - position[1]
    norm = math.hypot(delta_n, delta_e)
    if norm < 1e-6:
        forward_n, forward_e = 1.0, 0.0
    else:
        forward_n, forward_e = delta_n / norm, delta_e / norm
    # Right-hand unit vector in the horizontal NED plane.
    right_n, right_e = -forward_e, forward_n
    if obstacle_sector == "left":
        side_sign = 1.0
    elif obstacle_sector == "right":
        side_sign = -1.0
    else:
        left_clear = math.inf if left_range_m < 0.0 else left_range_m
        right_clear = math.inf if right_range_m < 0.0 else right_range_m
        side_sign = -1.0 if left_clear >= right_clear else 1.0
    return [
        position[0] + forward_n * forward_m + right_n * side_sign * lateral_m,
        position[1] + forward_e * forward_m + right_e * side_sign * lateral_m,
        target[2],
    ]


def sensor_bypass_plan(
    position: Vector3,
    target: Vector3,
    obstacle_sector: str,
    forward_m: float,
    lateral_m: float,
    left_range_m: float = -1.0,
    right_range_m: float = -1.0,
) -> tuple[list[float], list[float]]:
    """Return lateral-escape then forward-clearance targets for blind AABB-free flight."""
    lateral = sensor_bypass_target(
        position,
        target,
        obstacle_sector,
        0.0,
        lateral_m,
        left_range_m,
        right_range_m,
    )
    combined = sensor_bypass_target(
        position,
        target,
        obstacle_sector,
        forward_m,
        lateral_m,
        left_range_m,
        right_range_m,
    )
    return lateral, combined


def sensor_hit_within_segment(
    position: Vector3,
    target: Vector3,
    obstacle_sector: str | None,
    front_range_m: float,
    left_range_m: float,
    right_range_m: float,
    endpoint_margin_m: float = 0.5,
) -> bool:
    """True when a sector return can obstruct the current horizontal leg."""
    if obstacle_sector is None:
        return False
    ranges = {
        "front": front_range_m,
        "left": left_range_m,
        "right": right_range_m,
    }
    hit_range = ranges.get(obstacle_sector, -1.0)
    if hit_range < 0.0:
        return False
    leg_length = math.hypot(target[0] - position[0], target[1] - position[1])
    return hit_range <= leg_length + endpoint_margin_m


def segment_endpoint_passed(
    origin: Vector3,
    endpoint: Vector3,
    position: Vector3,
    tolerance_m: float = 0.0,
) -> bool:
    """True after position crosses the plane normal to a horizontal leg."""
    leg_n = endpoint[0] - origin[0]
    leg_e = endpoint[1] - origin[1]
    length = math.hypot(leg_n, leg_e)
    if length < 1e-6:
        return True
    progress = (
        (position[0] - origin[0]) * leg_n
        + (position[1] - origin[1]) * leg_e
    ) / length
    return progress >= length - tolerance_m


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
