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


# Obstacle AABBs (east_m, north_m, size_e, size_n, height_m) matching the
# physical boxes placed in worlds/obstacle_world.sdf — the single source of
# truth for both the single-vehicle course planner and the swarm coordinator,
# so a two-vehicle survey through the same world avoids the same obstacles.
DEFAULT_OBSTACLE_COURSE = (
    Obstacle(-6.0, 10.0, 3.0, 3.0, 11.5),  # OB1 red — taller than fence ceiling, climb not an option
    Obstacle(10.0, 10.0, 3.0, 3.0, 6.0),   # OB2 orange — off to the side, rarely on the flight path
    Obstacle(-8.0, 24.0, 5.0, 3.0, 4.0),   # OB3 green — widened, tighter corridor with OB4
    Obstacle(6.0, 24.0, 3.0, 2.0, 5.0),    # OB4 blue — widened, tighter corridor with OB3
    Obstacle(0.0, 38.0, 5.0, 3.0, 11.5),   # OB5 purple — taller than fence ceiling, climb not an option
)


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


def point_to_aabb_distance_2d(
    north: float,
    east: float,
    obstacle: Obstacle,
    margin_m: float = 0.0,
) -> float:
    """Euclidean distance from a point to an expanded axis-aligned box (2D)."""
    half_n = obstacle.size_north / 2.0 + margin_m
    half_e = obstacle.size_east / 2.0 + margin_m
    dn = max(abs(north - obstacle.north) - half_n, 0.0)
    de = max(abs(east - obstacle.east) - half_e, 0.0)
    return math.hypot(dn, de)


def segment_hits_expanded_aabb(
    start: Vector3,
    end: Vector3,
    obstacle: Obstacle,
    margin_m: float,
) -> tuple[bool, float, float, float]:
    """Liang-Barsky style slab test of a horizontal segment vs expanded AABB.

    Returns (hits, closest_distance_from_start, closest_n, closest_e).
    """
    half_n = obstacle.size_north / 2.0 + margin_m
    half_e = obstacle.size_east / 2.0 + margin_m
    n0, e0 = start[0], start[1]
    n1, e1 = end[0], end[1]
    dn, de = n1 - n0, e1 - e0

    t0, t1 = 0.0, 1.0
    for dist, p0, minimum, maximum in (
        (dn, n0, obstacle.north - half_n, obstacle.north + half_n),
        (de, e0, obstacle.east - half_e, obstacle.east + half_e),
    ):
        if abs(dist) < 1e-12:
            if p0 < minimum or p0 > maximum:
                # Parallel to this axis and outside the slab — the segment
                # can never enter the box on this axis, so it cannot hit it
                # at all (e.g. a straight path running alongside an obstacle,
                # always N metres clear). Must return here: falling through
                # to the "segment overlaps" code below would compute a
                # bogus "hit" from whatever t0/t1 the other axis alone left
                # behind, ignoring that this axis rules the box out entirely.
                dist_start = point_to_aabb_distance_2d(n0, e0, obstacle, margin_m)
                return False, dist_start, n0, e0
            continue
        inv = 1.0 / dist
        ta = (minimum - p0) * inv
        tb = (maximum - p0) * inv
        if ta > tb:
            ta, tb = tb, ta
        t0 = max(t0, ta)
        t1 = min(t1, tb)
        if t0 > t1:
            # No overlap with slab interval — use start-to-AABB distance only.
            dist_start = point_to_aabb_distance_2d(n0, e0, obstacle, margin_m)
            return False, dist_start, n0, e0

    # Segment overlaps expanded AABB in parameter [t0, t1].
    t_hit = max(0.0, min(1.0, t0))
    hit_n = n0 + dn * t_hit
    hit_e = e0 + de * t_hit
    # Clamp representation of closest surface point for bearing.
    clamp_n = min(max(hit_n, obstacle.north - half_n), obstacle.north + half_n)
    clamp_e = min(max(hit_e, obstacle.east - half_e), obstacle.east + half_e)
    dist = math.hypot(clamp_n - n0, clamp_e - e0)
    return True, dist, clamp_n, clamp_e


def detect_obstacle(
    position: Vector3,
    target: Vector3,
    obstacles: Iterable[Obstacle],
    detection_margin_m: float,
    front_angle_deg: float,
    side_angle_deg: float,
    detect_when_above: bool = False,
) -> str | None:
    """Classify the nearest relevant obstacle relative to direction of travel.

    Uses segment ∩ expanded-AABB (computational geometry) so corridor sides
    of tall thin boxes are not missed by center-bearing heuristics alone.
    """
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

        hits, distance, near_n, near_e = segment_hits_expanded_aabb(
            position, target, obstacle, detection_margin_m
        )
        # Also accept obstacles near the vehicle even if the remaining
        # setpoint segment is short / already past the box.
        near_start = point_to_aabb_distance_2d(
            position[0], position[1], obstacle, detection_margin_m
        )
        if not hits and near_start > 0.05:
            continue
        if not hits:
            distance = near_start
            near_n = min(
                max(position[0], obstacle.north - obstacle.size_north / 2.0),
                obstacle.north + obstacle.size_north / 2.0,
            )
            near_e = min(
                max(position[1], obstacle.east - obstacle.size_east / 2.0),
                obstacle.east + obstacle.size_east / 2.0,
            )

        # Bearing for left/right/front: prefer vector to obstacle center when
        # the vehicle is already inside the expanded keep-out (closest-point
        # bearing collapses to ~0 and falsely looks like "front").
        if near_start <= 0.05 or distance < 0.15:
            bn = obstacle.north - position[0]
            be = obstacle.east - position[1]
        else:
            bn = near_n - position[0]
            be = near_e - position[1]
        bearing = math.degrees(math.atan2(be, bn))
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
    """Return the tallest obstacle whose expanded AABB meets the travel leg."""
    tallest = 0.0
    for obstacle in obstacles:
        hits, _, _, _ = segment_hits_expanded_aabb(
            position, target, obstacle, detection_margin_m + 1.0
        )
        near = point_to_aabb_distance_2d(
            position[0], position[1], obstacle, detection_margin_m + 1.0
        )
        if hits or near <= 0.05:
            tallest = max(tallest, obstacle.height)
    return tallest
