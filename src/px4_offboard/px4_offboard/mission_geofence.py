"""Mission-wide geofence for the integrated sortie.

The ROS offboard node keeps a separate NED box in ``mission_logic.Fence`` and
clamps setpoints (the geo-cage). This module does not clamp. It decides whether
the vehicle and a proposed setpoint are inside a configured boundary, and the
orchestrator rejects commands that are not.

Rectangles are built from the same north/east/altitude numbers as ``Fence``.
Geographic polygons are refused: this repository has no independently checked
local-to-geographic conversion for a keep-in boundary.

Prediction assumes constant velocity over ``prediction_horizon_s``. It is an
advisory. It is not a certified safety system and it does not command a
recovery maneuver by itself.

Altitude is compared only in the frame named by the configuration. A local
height is never treated as AGL, AMSL, or barometric altitude.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .altitude_reference import AltitudeFrame
from .mission_logic import Fence

INSIDE = "inside"
APPROACHING = "approaching_boundary"
OUTSIDE = "outside"
INSUFFICIENT = "insufficient_data"
INVALID = "invalid_configuration"

ADMIT = "admit"
HOLD_INSIDE = "hold_inside"
NAV_LAND = "nav_land"

# The airframe can rest slightly below the recovery floor before PX4's land
# detector reports landed. This lag is not a lower cruise floor.
RECOVERY_TOUCHDOWN_LAG_M = 1.0


@dataclass(frozen=True)
class GeofenceConfig:
    """Versioned keep-in. Distances and altitudes are metres. Time is seconds."""

    source: str
    polygon_ned: tuple[tuple[float, float], ...]
    altitude_frame: AltitudeFrame
    min_altitude_m: float
    max_altitude_m: float
    margin_m: float = 1.0
    approach_distance_m: float = 2.0
    uncertainty_sigma: float = 1.0
    prediction_horizon_s: float = 2.0
    recovery_polygon_ned: tuple[tuple[float, float], ...] | None = None
    recovery_min_altitude_m: float = 0.0
    version: int = 1
    frame: str = "local_ned"

    def problems(self) -> str:
        if self.version != 1:
            return "unsupported geofence configuration version"
        if not self.source:
            return "geofence configuration source is missing"
        if self.frame != "local_ned":
            return "geographic polygons are not enabled; local NED conversion is not independently validated"
        if not isinstance(self.altitude_frame, AltitudeFrame):
            return "altitude frame is missing"
        message = _polygon_problems(self.polygon_ned, "flight boundary")
        if message:
            return message
        if not all(math.isfinite(v) for v in (self.min_altitude_m, self.max_altitude_m, self.margin_m,
                                               self.approach_distance_m, self.uncertainty_sigma,
                                               self.prediction_horizon_s, self.recovery_min_altitude_m)):
            return "geofence limits must be finite"
        if self.min_altitude_m > self.max_altitude_m:
            return "maximum altitude is below minimum altitude"
        if self.margin_m < 0.0 or self.approach_distance_m < 0.0:
            return "margin and approach distance must be non-negative"
        if self.uncertainty_sigma <= 0.0 or self.prediction_horizon_s <= 0.0:
            return "uncertainty sigma and prediction horizon must be positive"
        if self.recovery_min_altitude_m > self.max_altitude_m:
            return "recovery altitude exception exceeds the maximum altitude"
        if self.recovery_polygon_ned is not None:
            message = _polygon_problems(self.recovery_polygon_ned, "recovery area")
            if message:
                return message
            for north, east in self.recovery_polygon_ned:
                if not _contains(self.polygon_ned, north, east):
                    return "recovery area leaves the flight boundary"
        return ""

    @staticmethod
    def from_fence(
        fence: Fence,
        *,
        source: str,
        min_altitude_m: float,
        altitude_frame: AltitudeFrame = AltitudeFrame.LOCAL,
        margin_m: float = 1.0,
        approach_distance_m: float = 2.0,
        recovery_polygon_ned: tuple[tuple[float, float], ...] | None = None,
        recovery_min_altitude_m: float = 0.0,
    ) -> "GeofenceConfig":
        """Reuse ``Fence`` corner order. Altitude uses the named frame, not a second unit."""
        polygon = (
            (fence.north_min, fence.east_min),
            (fence.north_max, fence.east_min),
            (fence.north_max, fence.east_max),
            (fence.north_min, fence.east_max),
        )
        return GeofenceConfig(
            source=source,
            polygon_ned=polygon,
            altitude_frame=altitude_frame,
            min_altitude_m=min_altitude_m,
            max_altitude_m=fence.altitude_max,
            margin_m=margin_m,
            approach_distance_m=approach_distance_m,
            recovery_polygon_ned=recovery_polygon_ned,
            recovery_min_altitude_m=recovery_min_altitude_m,
        )


@dataclass(frozen=True)
class GeofenceDecision:
    status: str
    reason_code: str
    time_s: float
    mission_state: str
    north_m: float | None
    east_m: float | None
    altitude_m: float | None
    frame: str
    position_uncertainty_m: float | None
    boundary_distance_m: float | None
    altitude_margin_m: float | None
    nearest_boundary: str
    speed_mps: float | None
    approach_time_s: float | None
    predicted_status: str | None = None
    predicted_reason: str = ""

    def as_dict(self) -> dict:
        return {
            "status": self.status,
            "reason_code": self.reason_code,
            "time_s": self.time_s,
            "mission_state": self.mission_state,
            "north_m": self.north_m,
            "east_m": self.east_m,
            "altitude_m": self.altitude_m,
            "frame": self.frame,
            "horizontal_frame": "local_ned",
            "position_uncertainty_m": self.position_uncertainty_m,
            "boundary_distance_m": self.boundary_distance_m,
            "altitude_margin_m": self.altitude_margin_m,
            "nearest_boundary": self.nearest_boundary,
            "speed_mps": self.speed_mps,
            "approach_time_s": self.approach_time_s,
            "predicted_status": self.predicted_status,
            "predicted_reason": self.predicted_reason,
            "units": {"distance": "m", "speed": "m/s", "time": "s", "altitude": "m"},
            "simulated": True,
            "estimated": True,
        }


@dataclass(frozen=True)
class CommandVerdict:
    admissible: bool
    reason_code: str
    response: str
    boundary: str = ""


def evaluate_vehicle(
    config: GeofenceConfig,
    *,
    north_m: float | None,
    east_m: float | None,
    altitude_m: float | None,
    altitude_valid: bool,
    altitude_frame: AltitudeFrame | None,
    position_uncertainty_m: float | None,
    velocity_ned: tuple[float, float, float] | None,
    velocity_uncertainty_mps: float | None,
    time_s: float,
    mission_state: str,
) -> GeofenceDecision:
    """Constant-velocity prediction is attached as an advisory. It does not change a clear OUTSIDE."""
    problem = config.problems()
    if problem:
        return _decision(INVALID, "invalid_configuration", time_s, mission_state, config, None, None, None, None, problem)
    if north_m is None or east_m is None or not math.isfinite(north_m) or not math.isfinite(east_m):
        return _decision(INSUFFICIENT, "position_invalid", time_s, mission_state, config, north_m, east_m, altitude_m, position_uncertainty_m, "")
    if altitude_frame is not config.altitude_frame:
        return _decision(INSUFFICIENT, "altitude_frame_mismatch", time_s, mission_state, config, north_m, east_m, altitude_m, position_uncertainty_m, "")
    if not altitude_valid or altitude_m is None or not math.isfinite(altitude_m):
        return _decision(INSUFFICIENT, "altitude_unavailable", time_s, mission_state, config, north_m, east_m, altitude_m, position_uncertainty_m, "")

    inside, distance, nearest = _horizontal(config.polygon_ned, north_m, east_m)
    floor = _floor(config, north_m, east_m)
    alt_margin = min(altitude_m - floor, config.max_altitude_m - altitude_m)
    speed = _speed(velocity_ned)
    closing = _closing_speed(config.polygon_ned, north_m, east_m, velocity_ned, nearest)
    approach_time = None if closing is None or closing <= 1e-6 or distance is None or distance <= 0.0 else distance / closing
    uncertainty = None if position_uncertainty_m is None else config.uncertainty_sigma * max(0.0, position_uncertainty_m)
    on_boundary = distance is not None and abs(distance) <= 1e-6

    if on_boundary and (uncertainty is None or uncertainty <= 1e-9) and floor <= altitude_m <= config.max_altitude_m:
        status, reason = INSIDE, "on_boundary"
    elif not inside:
        status, reason = OUTSIDE, "outside_horizontal"
        if distance is not None and uncertainty is not None and abs(distance) <= uncertainty:
            status, reason = INSUFFICIENT, "uncertain_proximity"
    elif altitude_m < floor:
        status, reason = OUTSIDE, "below_min_altitude"
    elif altitude_m > config.max_altitude_m:
        status, reason = OUTSIDE, "above_max_altitude"
    elif uncertainty is not None and distance is not None and distance <= uncertainty:
        status, reason = APPROACHING, "uncertain_proximity"
    elif uncertainty is None and distance is not None and distance < config.approach_distance_m:
        status, reason = INSUFFICIENT, "uncertainty_not_reported"
    elif distance is not None and distance < config.approach_distance_m:
        status, reason = APPROACHING, "approaching_boundary"
    else:
        status, reason = INSIDE, "inside_boundary"

    predicted_status, predicted_reason = _predict(
        config, north_m, east_m, altitude_m, velocity_ned, position_uncertainty_m, velocity_uncertainty_mps
    )
    decision = _decision(
        status, reason, time_s, mission_state, config, north_m, east_m, altitude_m, position_uncertainty_m, nearest
    )
    return GeofenceDecision(
        status=decision.status,
        reason_code=decision.reason_code,
        time_s=decision.time_s,
        mission_state=decision.mission_state,
        north_m=decision.north_m,
        east_m=decision.east_m,
        altitude_m=decision.altitude_m,
        frame=decision.frame,
        position_uncertainty_m=decision.position_uncertainty_m,
        boundary_distance_m=distance,
        altitude_margin_m=alt_margin,
        nearest_boundary=nearest,
        speed_mps=speed,
        approach_time_s=approach_time,
        predicted_status=predicted_status,
        predicted_reason=predicted_reason,
    )


def command_admissible(
    config: GeofenceConfig,
    vehicle: GeofenceDecision,
    *,
    use_position: bool,
    position_ned: tuple[float, float, float] | None,
    velocity_ned: tuple[float, float, float] | None,
    north_m: float | None,
    east_m: float | None,
    altitude_m: float | None,
) -> CommandVerdict:
    """Reject an inadmissible setpoint. A prediction alone does not reject a hold that stays inside.

    PX4 position setpoints may carry NaN velocity. PX4 velocity setpoints may omit position.
    A position command with a non-finite component, or a velocity command with a non-finite
    component, is rejected. NaN in the unused field is ignored.
    """
    if config.problems():
        return CommandVerdict(False, "invalid_configuration", NAV_LAND, "configuration")
    if vehicle.status in (INSUFFICIENT, INVALID) and vehicle.reason_code in ("position_invalid", "altitude_unavailable", "altitude_frame_mismatch"):
        return CommandVerdict(False, vehicle.reason_code, NAV_LAND, vehicle.nearest_boundary)
    if use_position:
        if position_ned is None or not all(math.isfinite(v) for v in position_ned):
            return CommandVerdict(False, "setpoint_position_invalid", _fallback(vehicle), "setpoint")
        if (
            north_m is not None
            and east_m is not None
            and altitude_m is not None
            and vehicle.status in (INSIDE, APPROACHING)
            and math.dist((north_m, east_m, altitude_m), (position_ned[0], position_ned[1], -position_ned[2])) < 0.05
        ):
            return CommandVerdict(True, "hold_current", ADMIT, "")
        target = _point_status(config, position_ned[0], position_ned[1], -position_ned[2])
        if target != INSIDE:
            return CommandVerdict(False, "setpoint_outside", _fallback(vehicle), "setpoint")
        if north_m is not None and east_m is not None and _segment_exits(config.polygon_ned, north_m, east_m, position_ned[0], position_ned[1]):
            return CommandVerdict(False, "return_route_blocked", _fallback(vehicle), "segment")
        return CommandVerdict(True, "admitted", ADMIT, "")
    if velocity_ned is None or not all(math.isfinite(v) for v in velocity_ned):
        return CommandVerdict(False, "setpoint_velocity_invalid", _fallback(vehicle), "setpoint")
    if north_m is None or east_m is None or altitude_m is None:
        return CommandVerdict(False, "position_invalid", NAV_LAND, "velocity")
    predicted_n = north_m + velocity_ned[0] * config.prediction_horizon_s
    predicted_e = east_m + velocity_ned[1] * config.prediction_horizon_s
    predicted_alt = altitude_m - velocity_ned[2] * config.prediction_horizon_s
    inside, distance, _nearest = _horizontal(config.polygon_ned, predicted_n, predicted_e)
    predicted_floor = _floor(config, predicted_n, predicted_e)
    horizontal_ok = inside and (distance is None or distance >= config.margin_m)
    current_floor = _floor(config, north_m, east_m)
    in_recovery = (
        config.recovery_polygon_ned is not None
        and _contains(config.recovery_polygon_ned, north_m, east_m)
        and _contains(config.recovery_polygon_ned, predicted_n, predicted_e)
    )
    # A downward command inside the recovery area may cross the floor by the
    # land-detector lag. A climb, or a prediction that leaves the recovery
    # area, is not part of that exception.
    landing = (
        horizontal_ok
        and velocity_ned[2] >= 0.0
        and in_recovery
        and altitude_m >= current_floor - RECOVERY_TOUCHDOWN_LAG_M
    )
    if landing or (horizontal_ok and predicted_floor <= predicted_alt <= config.max_altitude_m):
        return CommandVerdict(True, "admitted", ADMIT, "")
    return CommandVerdict(False, "predicted_command_exit", _fallback(vehicle), "velocity")


def _fallback(vehicle: GeofenceDecision) -> str:
    """Hold when the fix is inside or only approaching. Land when it is outside or unknown."""
    if vehicle.status in (INSIDE, APPROACHING):
        return HOLD_INSIDE
    return NAV_LAND


def _predict(config, north, east, altitude, velocity, position_uncertainty, velocity_uncertainty):
    if velocity is None or not all(math.isfinite(v) for v in velocity):
        return None, "velocity_invalid"
    horizon = config.prediction_horizon_s
    ahead_n = north + velocity[0] * horizon
    ahead_e = east + velocity[1] * horizon
    ahead_alt = altitude - velocity[2] * horizon
    status = _point_status(config, ahead_n, ahead_e, ahead_alt)
    if status == OUTSIDE:
        return OUTSIDE, "predicted_boundary_violation"
    pad = 0.0
    if position_uncertainty is not None and math.isfinite(position_uncertainty):
        pad += config.uncertainty_sigma * position_uncertainty
    if velocity_uncertainty is not None and math.isfinite(velocity_uncertainty):
        pad += config.uncertainty_sigma * velocity_uncertainty * horizon
    if pad > 0.0:
        _inside, distance, _nearest = _horizontal(config.polygon_ned, ahead_n, ahead_e)
        if distance is not None and distance < pad:
            return APPROACHING, "uncertain_predicted_crossing"
    return INSIDE, "prediction_clear"


def _point_status(config: GeofenceConfig, north: float, east: float, altitude: float) -> str:
    inside, distance, _nearest = _horizontal(config.polygon_ned, north, east)
    if not inside:
        return OUTSIDE
    if distance is not None and distance < config.margin_m:
        return APPROACHING
    floor = _floor(config, north, east)
    if altitude < floor or altitude > config.max_altitude_m:
        return OUTSIDE
    return INSIDE


def _floor(config: GeofenceConfig, north: float, east: float) -> float:
    if config.recovery_polygon_ned is not None and _contains(config.recovery_polygon_ned, north, east):
        return config.recovery_min_altitude_m
    return config.min_altitude_m


def _decision(status, reason, time_s, mission_state, config, north, east, altitude, uncertainty, nearest) -> GeofenceDecision:
    return GeofenceDecision(
        status=status,
        reason_code=reason,
        time_s=time_s,
        mission_state=mission_state,
        north_m=north,
        east_m=east,
        altitude_m=altitude,
        frame=config.altitude_frame.value,
        position_uncertainty_m=uncertainty,
        boundary_distance_m=None,
        altitude_margin_m=None,
        nearest_boundary=nearest,
        speed_mps=None,
        approach_time_s=None,
    )


def _polygon_problems(points: tuple[tuple[float, float], ...], name: str) -> str:
    if len(points) < 3:
        return f"{name} needs at least three vertices"
    if not all(math.isfinite(value) for point in points for value in point):
        return f"{name} has a nonfinite coordinate"
    if _self_intersects(points):
        return f"{name} is self-intersecting"
    return ""


def _self_intersects(points: tuple[tuple[float, float], ...]) -> bool:
    edges = list(zip(points, points[1:] + points[:1]))
    count = len(edges)
    for i in range(count):
        for j in range(i + 1, count):
            if abs(i - j) % count <= 1 or abs(i - j) == count - 1:
                continue
            if _proper_cross(edges[i][0], edges[i][1], edges[j][0], edges[j][1]):
                return True
    return False


def _proper_cross(a, b, c, d) -> bool:
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    o1, o2 = orient(a, b, c), orient(a, b, d)
    o3, o4 = orient(c, d, a), orient(c, d, b)
    return o1 * o2 < 0.0 and o3 * o4 < 0.0


def _contains(polygon, north: float, east: float) -> bool:
    inside, distance, _nearest = _horizontal(polygon, north, east)
    return inside or (distance is not None and abs(distance) <= 1e-6)


def _horizontal(polygon, north: float, east: float) -> tuple[bool, float | None, str]:
    distance, nearest = _boundary_distance(polygon, north, east)
    on_edge = distance <= 1e-6
    inside = on_edge or _ray(polygon, north, east)
    signed = distance if inside else -distance
    return inside, signed, nearest


def _ray(polygon, north: float, east: float) -> bool:
    inside = False
    count = len(polygon)
    j = count - 1
    for i in range(count):
        ni, ei = polygon[i]
        nj, ej = polygon[j]
        if ((ei > east) != (ej > east)) and (
            north < (nj - ni) * (east - ei) / (ej - ei + 0.0) + ni
        ):
            inside = not inside
        j = i
    return inside


def _boundary_distance(polygon, north: float, east: float) -> tuple[float, str]:
    best = float("inf")
    nearest = ""
    count = len(polygon)
    for index in range(count):
        start = polygon[index]
        end = polygon[(index + 1) % count]
        distance = _segment_distance(north, east, start, end)
        if distance < best:
            best = distance
            nearest = f"edge_{index}"
    return best, nearest


def _segment_distance(north, east, start, end) -> float:
    dn, de = end[0] - start[0], end[1] - start[1]
    length_sq = dn * dn + de * de
    if length_sq <= 1e-18:
        return math.hypot(north - start[0], east - start[1])
    t = max(0.0, min(1.0, ((north - start[0]) * dn + (east - start[1]) * de) / length_sq))
    return math.hypot(north - (start[0] + t * dn), east - (start[1] + t * de))


def _segment_exits(polygon, north0, east0, north1, east1) -> bool:
    if not _contains(polygon, north0, east0) or not _contains(polygon, north1, east1):
        return True
    for step in range(1, 32):
        fraction = step / 32.0
        north = north0 + (north1 - north0) * fraction
        east = east0 + (east1 - east0) * fraction
        if not _contains(polygon, north, east):
            return True
    return False


def _speed(velocity) -> float | None:
    if velocity is None or not all(math.isfinite(v) for v in velocity[:2]):
        return None
    return math.hypot(velocity[0], velocity[1])


def _closing_speed(polygon, north, east, velocity, nearest: str) -> float | None:
    if velocity is None or not nearest.startswith("edge_") or not all(math.isfinite(v) for v in velocity[:2]):
        return None
    index = int(nearest.split("_")[1])
    start = polygon[index]
    end = polygon[(index + 1) % len(polygon)]
    dn, de = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dn, de)
    if length <= 1e-9:
        return None
    # Left normal of a CCW edge points inside for a CCW polygon. Closing speed
    # is motion against that inward normal, toward the edge.
    inward_n, inward_e = -de / length, dn / length
    toward_edge = -(velocity[0] * inward_n + velocity[1] * inward_e)
    return toward_edge
