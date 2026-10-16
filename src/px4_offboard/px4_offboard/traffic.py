"""Traffic as a release constraint.

Tracks use WGS84 latitude and longitude. Altitude carries an explicit frame.
AMSL is compared with the vehicle's AMSL (local-origin AMSL plus local height).
LOCAL is compared with local height above the same mission origin. Barometric
altitude and AGL are not converted. Heading is degrees clockwise from true
north. Ground speed and vertical speed are metres per second; vertical speed
is up-positive. Times are seconds on the snapshot clock.

Separation numbers are simulation thresholds. They are not regulatory minima.
A passing decision means no reported track enters those thresholds inside the
snapshot's declared coverage. It does not mean every aircraft was seen, and it
is not a collision-avoidance command.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Protocol, Sequence

from .airspace import offset_latlon
from .weather import ClockDomain, Provenance, horizontal_distance_m

EARTH_RADIUS_M = 6_371_000.0
MAX_GROUND_SPEED_MPS = 400.0
MAX_VERTICAL_SPEED_MPS = 100.0


class TrafficAltitudeFrame(Enum):
    AMSL = "amsl"
    AGL = "agl"
    BAROMETRIC = "barometric"
    LOCAL = "local"


class ConflictStatus:
    NO_REPORTED_CONFLICT = "no_reported_conflict"
    POTENTIAL_CONFLICT = "potential_conflict"
    CONFLICT_DETECTED = "conflict_detected"
    INSUFFICIENT_DATA = "insufficient_data"
    INVALID_TRAFFIC_DATA = "invalid_traffic_data"


class Reason:
    TRAFFIC_DATA_MISSING = "traffic_data_missing"
    TRAFFIC_DATA_STALE = "traffic_data_stale"
    TRAFFIC_COVERAGE_INSUFFICIENT = "traffic_coverage_insufficient"
    TRAFFIC_TRACK_INVALID = "traffic_track_invalid"
    TRAFFIC_ALTITUDE_FRAME_UNKNOWN = "traffic_altitude_frame_unknown"
    TRAFFIC_SOURCE_UNAVAILABLE = "traffic_source_unavailable"
    TRAFFIC_FUTURE_OBSERVATION = "traffic_future_observation"
    TRAFFIC_DUPLICATE_TRACK = "traffic_duplicate_track"
    TRAFFIC_INVALID_VELOCITY = "traffic_invalid_velocity"
    TRAFFIC_CLOCK_MISMATCH = "traffic_clock_domain_mismatch"
    TRAFFIC_CONFLICT_DETECTED = "traffic_conflict_detected"
    TRAFFIC_PREDICTED_CONFLICT = "traffic_predicted_conflict"
    TRAFFIC_UNCERTAINTY_EXCEEDS_LIMIT = "traffic_uncertainty_exceeds_limit"
    TRAFFIC_TRACK_STALE = "traffic_track_stale"


_CONFLICT_RANK = {
    Reason.TRAFFIC_CONFLICT_DETECTED: 0,
    Reason.TRAFFIC_PREDICTED_CONFLICT: 1,
    Reason.TRAFFIC_UNCERTAINTY_EXCEEDS_LIMIT: 2,
}

_STATUS_FOR_REASON = {
    Reason.TRAFFIC_CONFLICT_DETECTED: ConflictStatus.CONFLICT_DETECTED,
    Reason.TRAFFIC_PREDICTED_CONFLICT: ConflictStatus.POTENTIAL_CONFLICT,
    Reason.TRAFFIC_UNCERTAINTY_EXCEEDS_LIMIT: ConflictStatus.INSUFFICIENT_DATA,
}

UNITS = {
    "horizontal_separation": "m",
    "vertical_separation": "m",
    "ground_speed": "m/s",
    "vertical_speed": "m/s",
    "heading": "deg_clockwise_from_true_north",
    "time": "s",
    "altitude": "m",
    "uncertainty": "m",
}


@dataclass(frozen=True)
class AircraftTrack:
    track_id: str
    observed_at_s: float | None
    received_at_s: float | None
    source: str
    latitude_deg: float | None
    longitude_deg: float | None
    altitude_m: float | None
    altitude_frame: TrafficAltitudeFrame | None
    ground_speed_mps: float | None
    heading_deg: float | None
    vertical_speed_mps: float | None
    position_uncertainty_m: float | None
    velocity_uncertainty_mps: float | None = None
    valid: bool = True
    source_quality: str = ""


@dataclass(frozen=True)
class TrafficSnapshot:
    source: str
    provenance: Provenance
    clock: ClockDomain
    observed_at_s: float | None
    received_at_s: float | None
    coverage_latitude_deg: float | None
    coverage_longitude_deg: float | None
    coverage_radius_m: float | None
    coverage_floor_amsl_m: float | None
    coverage_ceiling_amsl_m: float | None
    tracks: tuple[AircraftTrack, ...] = ()
    source_quality: str = ""

    def as_dict(self) -> dict:
        return {
            "source": self.source,
            "provenance": self.provenance.value,
            "clock": self.clock.value,
            "observed_at_s": self.observed_at_s,
            "received_at_s": self.received_at_s,
            "coverage_latitude_deg": self.coverage_latitude_deg,
            "coverage_longitude_deg": self.coverage_longitude_deg,
            "coverage_radius_m": self.coverage_radius_m,
            "coverage_floor_amsl_m": self.coverage_floor_amsl_m,
            "coverage_ceiling_amsl_m": self.coverage_ceiling_amsl_m,
            "source_quality": self.source_quality,
            "track_count": len(self.tracks),
            "surveillance_complete": False,
            "units": {"coverage_radius": "m", "altitude": "m", "time": "s"},
        }


@dataclass(frozen=True)
class TrafficFetch:
    snapshot: TrafficSnapshot | None
    reason_code: str
    detail: str = ""


class TrafficProvider(Protocol):
    def latest(self, now_s: float) -> TrafficFetch:
        """Return the cached snapshot. This method must not perform network I/O."""


@dataclass(frozen=True)
class TrafficLimits:
    """Simulation-only separation. Not a regulatory minimum."""

    horizontal_separation_m: float = 50.0
    vertical_separation_m: float = 30.0
    prediction_horizon_s: float = 30.0
    max_position_uncertainty_m: float = 150.0
    simulation_only: bool = True

    def __post_init__(self) -> None:
        for name in ("horizontal_separation_m", "vertical_separation_m", "prediction_horizon_s"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if not math.isfinite(self.max_position_uncertainty_m) or self.max_position_uncertainty_m < 0.0:
            raise ValueError("max_position_uncertainty_m must be non-negative")

    def as_dict(self) -> dict:
        return {
            "horizontal_separation_m": self.horizontal_separation_m,
            "vertical_separation_m": self.vertical_separation_m,
            "prediction_horizon_s": self.prediction_horizon_s,
            "max_position_uncertainty_m": self.max_position_uncertainty_m,
            "simulation_only": self.simulation_only,
            "units": UNITS,
            "label": "simulation_only_not_a_regulatory_minimum",
        }


@dataclass(frozen=True)
class TrafficPolicy:
    max_age_s: float = 600.0
    max_track_age_s: float = 30.0
    max_future_s: float = 0.0
    origin_latitude_deg: float = 47.0
    origin_longitude_deg: float = -122.0
    unix_origin_s: float | None = None
    limits: TrafficLimits | None = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.max_age_s) or self.max_age_s <= 0.0:
            raise ValueError("max_age_s must be positive")
        if not math.isfinite(self.max_track_age_s) or self.max_track_age_s <= 0.0:
            raise ValueError("max_track_age_s must be positive")
        if not math.isfinite(self.max_future_s) or self.max_future_s < 0.0:
            raise ValueError("max_future_s must be non-negative")
        if self.limits is None:
            object.__setattr__(self, "limits", TrafficLimits())


@dataclass(frozen=True)
class TrafficDecision:
    time_s: float
    authorized: bool
    reason_code: str
    status: str
    conflict_status: str
    source: str
    provenance: str | None
    clock: str | None
    observed_at_s: float | None
    received_at_s: float | None
    age_s: float | None
    surveillance_complete: bool
    operational_approval: bool
    conflicts: tuple[dict, ...]
    limits: dict
    snapshot: dict | None
    track_count: int

    def as_dict(self) -> dict:
        return {
            "gate": "traffic",
            "time_s": self.time_s,
            "authorized": self.authorized,
            "reason_code": self.reason_code,
            "status": self.status,
            "conflict_status": self.conflict_status,
            "source": self.source,
            "provenance": self.provenance,
            "clock": self.clock,
            "observed_at_s": self.observed_at_s,
            "received_at_s": self.received_at_s,
            "age_s": self.age_s,
            "surveillance_complete": False,
            "operational_approval": False,
            "conflicts": list(self.conflicts),
            "limits": self.limits,
            "snapshot": self.snapshot,
            "track_count": self.track_count,
            "units": UNITS,
            "collision_avoidance": False,
        }


class TrafficAuthorizer:
    def __init__(self, provider: TrafficProvider, policy: TrafficPolicy | None = None) -> None:
        self.provider = provider
        self.policy = policy or TrafficPolicy()

    def evaluate(
        self,
        now_s: float,
        north_m: float,
        east_m: float,
        down_m: float,
        velocity_ned: Sequence[float],
        ref_alt_m: float | None,
    ) -> TrafficDecision:
        latitude, longitude = offset_latlon(
            self.policy.origin_latitude_deg,
            self.policy.origin_longitude_deg,
            north_m,
            east_m,
        )
        return decide(
            self.provider.latest(now_s),
            self.policy,
            now_s,
            north_m,
            east_m,
            down_m,
            velocity_ned,
            ref_alt_m,
            latitude,
            longitude,
        )


def simulation_snapshot(**overrides: object) -> TrafficSnapshot:
    values: dict = {
        "source": "simulation",
        "provenance": Provenance.SIMULATED,
        "clock": ClockDomain.MISSION,
        "observed_at_s": 0.0,
        "received_at_s": 0.0,
        "coverage_latitude_deg": 47.0,
        "coverage_longitude_deg": -122.0,
        "coverage_radius_m": 100_000.0,
        "coverage_floor_amsl_m": 0.0,
        "coverage_ceiling_amsl_m": 20_000.0,
        "tracks": (),
        "source_quality": "simulation",
    }
    values.update(overrides)
    values["tracks"] = tuple(values["tracks"])
    return TrafficSnapshot(**values)


def track_at_ned(
    north_m: float,
    east_m: float,
    height_m: float,
    *,
    ref_alt_m: float = 0.0,
    track_id: str = "AC1",
    **overrides: object,
) -> AircraftTrack:
    latitude, longitude = offset_latlon(47.0, -122.0, north_m, east_m)
    values: dict = {
        "track_id": track_id,
        "observed_at_s": 0.0,
        "received_at_s": 0.0,
        "source": "simulation",
        "latitude_deg": latitude,
        "longitude_deg": longitude,
        "altitude_m": ref_alt_m + height_m,
        "altitude_frame": TrafficAltitudeFrame.AMSL,
        "ground_speed_mps": 0.0,
        "heading_deg": 0.0,
        "vertical_speed_mps": 0.0,
        "position_uncertainty_m": 5.0,
        "velocity_uncertainty_mps": 1.0,
        "valid": True,
        "source_quality": "simulation",
    }
    values.update(overrides)
    return AircraftTrack(**values)


def simulation_traffic_policy(**overrides: object) -> TrafficPolicy:
    values: dict = {"max_age_s": 3600.0, "max_track_age_s": 3600.0}
    values.update(overrides)
    return TrafficPolicy(**values)


def simulation_traffic_authorizer(
    snapshot: TrafficSnapshot | None = None,
    policy: TrafficPolicy | None = None,
) -> TrafficAuthorizer:
    from .traffic_provider import StaticTrafficProvider

    return TrafficAuthorizer(StaticTrafficProvider(snapshot or simulation_snapshot()), policy or simulation_traffic_policy())


def ned_offset(origin_lat: float, origin_lon: float, latitude: float, longitude: float) -> tuple[float, float]:
    north = math.radians(latitude - origin_lat) * EARTH_RADIUS_M
    east = math.radians(longitude - origin_lon) * EARTH_RADIUS_M * math.cos(math.radians(origin_lat))
    return north, east


def closest_approach(
    own_north_m: float,
    own_east_m: float,
    own_height_m: float,
    own_vn_mps: float,
    own_ve_mps: float,
    own_vh_mps: float,
    other_north_m: float,
    other_east_m: float,
    other_height_m: float,
    other_vn_mps: float,
    other_ve_mps: float,
    other_vh_mps: float,
) -> dict:
    """Constant-velocity closest approach. ``t_cpa_s`` is unconstrained and may be negative.

    Heights and ``*_vh_mps`` are up-positive. Horizontal distances are metres in the
    local north/east plane. This is a geometry report, not a collision prediction.
    """
    rn = other_north_m - own_north_m
    re = other_east_m - own_east_m
    rh = other_height_m - own_height_m
    vn = other_vn_mps - own_vn_mps
    ve = other_ve_mps - own_ve_mps
    vh = other_vh_mps - own_vh_mps
    speed_sq = vn * vn + ve * ve + vh * vh
    t_cpa = 0.0 if speed_sq < 1e-12 else -(rn * vn + re * ve + rh * vh) / speed_sq
    horizontal_now = math.hypot(rn, re)
    vertical_now = abs(rh)
    horizontal_cpa = math.hypot(rn + vn * t_cpa, re + ve * t_cpa)
    vertical_cpa = abs(rh + vh * t_cpa)
    return {
        "t_cpa_s": t_cpa,
        "horizontal_separation_m": horizontal_now,
        "vertical_separation_m": vertical_now,
        "cpa_horizontal_m": horizontal_cpa,
        "cpa_vertical_m": vertical_cpa,
        "units": UNITS,
    }


def decide(
    fetch: TrafficFetch,
    policy: TrafficPolicy,
    now_s: float,
    north_m: float,
    east_m: float,
    down_m: float,
    velocity_ned: Sequence[float],
    ref_alt_m: float | None,
    latitude_deg: float,
    longitude_deg: float,
) -> TrafficDecision:
    snapshot = fetch.snapshot
    if fetch.reason_code or snapshot is None:
        reason = fetch.reason_code or Reason.TRAFFIC_DATA_MISSING
        return _decision(now_s, None, policy, reason, ConflictStatus.INSUFFICIENT_DATA, None, ())
    mapped = _to_mission_clock(snapshot, policy)
    if isinstance(mapped, str):
        return _decision(now_s, snapshot, policy, mapped, ConflictStatus.INVALID_TRAFFIC_DATA, None, ())
    observed_s, received_s = mapped
    if observed_s is None or received_s is None:
        return _decision(
            now_s, snapshot, policy, Reason.TRAFFIC_TRACK_INVALID, ConflictStatus.INVALID_TRAFFIC_DATA, None, ()
        )
    age = now_s - observed_s
    if age < -policy.max_future_s:
        return _decision(
            now_s, snapshot, policy, Reason.TRAFFIC_FUTURE_OBSERVATION, ConflictStatus.INVALID_TRAFFIC_DATA, age, ()
        )
    if age > policy.max_age_s:
        return _decision(now_s, snapshot, policy, Reason.TRAFFIC_DATA_STALE, ConflictStatus.INSUFFICIENT_DATA, age, ())
    if not snapshot.source or not snapshot.source_quality:
        return _decision(
            now_s, snapshot, policy, Reason.TRAFFIC_SOURCE_UNAVAILABLE, ConflictStatus.INSUFFICIENT_DATA, age, ()
        )
    if not _coverage_declared(snapshot):
        return _decision(
            now_s, snapshot, policy, Reason.TRAFFIC_COVERAGE_INSUFFICIENT, ConflictStatus.INSUFFICIENT_DATA, age, ()
        )
    if ref_alt_m is None or not math.isfinite(ref_alt_m) or not math.isfinite(down_m):
        return _decision(
            now_s, snapshot, policy, Reason.TRAFFIC_COVERAGE_INSUFFICIENT, ConflictStatus.INSUFFICIENT_DATA, age, ()
        )
    own_amsl = ref_alt_m - down_m
    if not _inside_coverage(snapshot, latitude_deg, longitude_deg, own_amsl):
        return _decision(
            now_s, snapshot, policy, Reason.TRAFFIC_COVERAGE_INSUFFICIENT, ConflictStatus.INSUFFICIENT_DATA, age, ()
        )
    invalid = _invalid_track_reason(snapshot, policy, now_s)
    if invalid is not None:
        status = (
            ConflictStatus.INSUFFICIENT_DATA
            if invalid in (Reason.TRAFFIC_TRACK_STALE, Reason.TRAFFIC_SOURCE_UNAVAILABLE)
            else ConflictStatus.INVALID_TRAFFIC_DATA
        )
        return _decision(now_s, snapshot, policy, invalid, status, age, ())
    conflicts = _conflicts(snapshot, policy, north_m, east_m, -down_m, velocity_ned, ref_alt_m)
    if not conflicts:
        return _decision(now_s, snapshot, policy, "", ConflictStatus.NO_REPORTED_CONFLICT, age, ())
    chosen = min(conflicts, key=lambda item: _CONFLICT_RANK[item["reason_code"]])
    return _decision(
        now_s,
        snapshot,
        policy,
        chosen["reason_code"],
        _STATUS_FOR_REASON[chosen["reason_code"]],
        age,
        tuple(conflicts),
    )


def _coverage_declared(snapshot: TrafficSnapshot) -> bool:
    values = (
        snapshot.coverage_latitude_deg,
        snapshot.coverage_longitude_deg,
        snapshot.coverage_radius_m,
        snapshot.coverage_floor_amsl_m,
        snapshot.coverage_ceiling_amsl_m,
    )
    if any(value is None or not math.isfinite(value) for value in values):
        return False
    assert snapshot.coverage_radius_m is not None
    return snapshot.coverage_radius_m > 0.0


def _inside_coverage(snapshot: TrafficSnapshot, latitude: float, longitude: float, own_amsl: float) -> bool:
    assert snapshot.coverage_latitude_deg is not None
    assert snapshot.coverage_longitude_deg is not None
    assert snapshot.coverage_radius_m is not None
    assert snapshot.coverage_floor_amsl_m is not None
    assert snapshot.coverage_ceiling_amsl_m is not None
    distance = horizontal_distance_m(
        latitude, longitude, snapshot.coverage_latitude_deg, snapshot.coverage_longitude_deg
    )
    return distance <= snapshot.coverage_radius_m and snapshot.coverage_floor_amsl_m <= own_amsl <= snapshot.coverage_ceiling_amsl_m


def _invalid_track_reason(snapshot: TrafficSnapshot, policy: TrafficPolicy, now_s: float) -> str | None:
    seen: set[str] = set()
    for track in snapshot.tracks:
        if not track.track_id or track.track_id in seen:
            return Reason.TRAFFIC_DUPLICATE_TRACK if track.track_id in seen else Reason.TRAFFIC_TRACK_INVALID
        seen.add(track.track_id)
        problem = _track_problem(track, policy, now_s)
        if problem is not None:
            return problem
    return None


def _track_problem(track: AircraftTrack, policy: TrafficPolicy, now_s: float) -> str | None:
    if not track.valid:
        return Reason.TRAFFIC_TRACK_INVALID
    if not track.source or not track.source_quality:
        return Reason.TRAFFIC_SOURCE_UNAVAILABLE
    if track.observed_at_s is None or track.received_at_s is None:
        return Reason.TRAFFIC_TRACK_INVALID
    if not math.isfinite(track.observed_at_s) or not math.isfinite(track.received_at_s):
        return Reason.TRAFFIC_TRACK_INVALID
    if track.observed_at_s > now_s + policy.max_future_s:
        return Reason.TRAFFIC_FUTURE_OBSERVATION
    if now_s - track.observed_at_s > policy.max_track_age_s:
        return Reason.TRAFFIC_TRACK_STALE
    if (
        track.latitude_deg is None
        or track.longitude_deg is None
        or not math.isfinite(track.latitude_deg)
        or not math.isfinite(track.longitude_deg)
        or not (-90.0 <= track.latitude_deg <= 90.0)
        or not (-180.0 <= track.longitude_deg <= 180.0)
    ):
        return Reason.TRAFFIC_TRACK_INVALID
    if track.altitude_frame not in (TrafficAltitudeFrame.AMSL, TrafficAltitudeFrame.LOCAL):
        return Reason.TRAFFIC_ALTITUDE_FRAME_UNKNOWN
    if track.altitude_m is None or not math.isfinite(track.altitude_m):
        return Reason.TRAFFIC_TRACK_INVALID
    if not _velocity_ok(track):
        return Reason.TRAFFIC_INVALID_VELOCITY
    if (
        track.position_uncertainty_m is None
        or not math.isfinite(track.position_uncertainty_m)
        or track.position_uncertainty_m < 0.0
    ):
        return Reason.TRAFFIC_TRACK_INVALID
    if track.velocity_uncertainty_mps is not None and (
        not math.isfinite(track.velocity_uncertainty_mps) or track.velocity_uncertainty_mps < 0.0
    ):
        return Reason.TRAFFIC_TRACK_INVALID
    return None


def _velocity_ok(track: AircraftTrack) -> bool:
    speed = track.ground_speed_mps
    heading = track.heading_deg
    vertical = track.vertical_speed_mps
    if speed is None or vertical is None or not math.isfinite(speed) or not math.isfinite(vertical):
        return False
    if speed < 0.0 or speed > MAX_GROUND_SPEED_MPS or abs(vertical) > MAX_VERTICAL_SPEED_MPS:
        return False
    if heading is None or not math.isfinite(heading) or not (0.0 <= heading < 360.0):
        return False
    return True


def _conflicts(
    snapshot: TrafficSnapshot,
    policy: TrafficPolicy,
    own_north: float,
    own_east: float,
    own_height: float,
    velocity_ned: Sequence[float],
    ref_alt_m: float,
) -> list[dict]:
    limits = policy.limits or TrafficLimits()
    own_vn, own_ve, own_vd = (float(velocity_ned[0]), float(velocity_ned[1]), float(velocity_ned[2]))
    found: list[dict] = []
    for track in snapshot.tracks:
        report = _one_conflict(
            track, policy, limits, own_north, own_east, own_height, own_vn, own_ve, -own_vd, ref_alt_m
        )
        if report is not None:
            found.append(report)
    return found


def _one_conflict(
    track: AircraftTrack,
    policy: TrafficPolicy,
    limits: TrafficLimits,
    own_north: float,
    own_east: float,
    own_height: float,
    own_vn: float,
    own_ve: float,
    own_vh: float,
    ref_alt_m: float,
) -> dict | None:
    assert track.latitude_deg is not None and track.longitude_deg is not None
    assert track.altitude_m is not None and track.ground_speed_mps is not None
    assert track.heading_deg is not None and track.vertical_speed_mps is not None
    assert track.position_uncertainty_m is not None
    other_north, other_east = ned_offset(
        policy.origin_latitude_deg, policy.origin_longitude_deg, track.latitude_deg, track.longitude_deg
    )
    if track.altitude_frame is TrafficAltitudeFrame.AMSL:
        other_height = track.altitude_m - ref_alt_m
    else:
        other_height = track.altitude_m
    heading = math.radians(track.heading_deg)
    other_vn = track.ground_speed_mps * math.cos(heading)
    other_ve = track.ground_speed_mps * math.sin(heading)
    geometry = closest_approach(
        own_north,
        own_east,
        own_height,
        own_vn,
        own_ve,
        own_vh,
        other_north,
        other_east,
        other_height,
        other_vn,
        other_ve,
        track.vertical_speed_mps,
    )
    horizontal = geometry["horizontal_separation_m"]
    vertical = geometry["vertical_separation_m"]
    t_cpa = geometry["t_cpa_s"]
    reason = None
    if horizontal <= limits.horizontal_separation_m and vertical <= limits.vertical_separation_m:
        reason = Reason.TRAFFIC_CONFLICT_DETECTED
    elif (
        0.0 <= t_cpa <= limits.prediction_horizon_s
        and geometry["cpa_horizontal_m"] <= limits.horizontal_separation_m
        and geometry["cpa_vertical_m"] <= limits.vertical_separation_m
    ):
        reason = Reason.TRAFFIC_PREDICTED_CONFLICT
    elif _uncertainty_reaches(horizontal, vertical, track.position_uncertainty_m, limits):
        reason = Reason.TRAFFIC_UNCERTAINTY_EXCEEDS_LIMIT
    if reason is None:
        return None
    return {
        "track_id": track.track_id,
        "reason_code": reason,
        "conflict_status": _STATUS_FOR_REASON[reason],
        "horizontal_separation_m": horizontal,
        "vertical_separation_m": vertical,
        "t_cpa_s": t_cpa,
        "cpa_horizontal_m": geometry["cpa_horizontal_m"],
        "cpa_vertical_m": geometry["cpa_vertical_m"],
        "position_uncertainty_m": track.position_uncertainty_m,
        "altitude_frame": track.altitude_frame.value,
        "units": UNITS,
        "simulation_only": True,
    }


def _uncertainty_reaches(horizontal_m: float, vertical_m: float, uncertainty_m: float, limits: TrafficLimits) -> bool:
    if uncertainty_m <= limits.max_position_uncertainty_m:
        return False
    return (horizontal_m - uncertainty_m) <= limits.horizontal_separation_m and (
        vertical_m - uncertainty_m
    ) <= limits.vertical_separation_m


def _to_mission_clock(snapshot: TrafficSnapshot, policy: TrafficPolicy) -> tuple[float | None, float | None] | str:
    if snapshot.clock is ClockDomain.MISSION:
        return snapshot.observed_at_s, snapshot.received_at_s
    if snapshot.clock is ClockDomain.UNIX and policy.unix_origin_s is not None:
        observed = None if snapshot.observed_at_s is None else snapshot.observed_at_s - policy.unix_origin_s
        received = None if snapshot.received_at_s is None else snapshot.received_at_s - policy.unix_origin_s
        return observed, received
    return Reason.TRAFFIC_CLOCK_MISMATCH


def _decision(
    now_s: float,
    snapshot: TrafficSnapshot | None,
    policy: TrafficPolicy,
    reason: str,
    conflict_status: str,
    age_s: float | None,
    conflicts: tuple[dict, ...],
) -> TrafficDecision:
    limits = policy.limits or TrafficLimits()
    return TrafficDecision(
        time_s=now_s,
        authorized=reason == "",
        reason_code=reason,
        status="valid" if reason == "" or reason in _CONFLICT_RANK else conflict_status,
        conflict_status=conflict_status,
        source="" if snapshot is None else snapshot.source,
        provenance=None if snapshot is None else snapshot.provenance.value,
        clock=None if snapshot is None else snapshot.clock.value,
        observed_at_s=None if snapshot is None else snapshot.observed_at_s,
        received_at_s=None if snapshot is None else snapshot.received_at_s,
        age_s=age_s,
        surveillance_complete=False,
        operational_approval=False,
        conflicts=conflicts,
        limits=limits.as_dict(),
        snapshot=None if snapshot is None else snapshot.as_dict(),
        track_count=0 if snapshot is None else len(snapshot.tracks),
    )
