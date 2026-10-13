"""Weather as a release constraint.

Observations are immutable. Missing numbers stay missing. Temperatures are
degrees Celsius, wind is metres per second, pressure is pascals, altitudes are
metres, and timestamps are seconds on a named clock.

``simulation_constraints`` is a set of simulation knobs. A passing decision is
a software check. It is not an approval to fly.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Protocol


class ClockDomain(Enum):
    """``MISSION`` matches ``SeedingSupervisor`` time. ``UNIX`` is seconds since 1970-01-01 UTC."""

    MISSION = "mission"
    UNIX = "unix"


class Applicability(Enum):
    SURFACE = "surface"
    ALTITUDE = "altitude"
    FORECAST = "forecast"


class Provenance(Enum):
    SIMULATED = "simulated"
    RECORDED = "recorded"
    LIVE = "live"


class Status(Enum):
    VALID = "valid"
    MISSING = "missing"
    INVALID = "invalid"
    STALE = "stale"
    UNSUPPORTED = "unsupported"


class Reason:
    MISSING_OBSERVATION = "missing_observation"
    PROVIDER_FAILURE = "provider_failure"
    MALFORMED_RECORD = "malformed_record"
    CLOCK_DOMAIN_MISMATCH = "clock_domain_mismatch"
    MISSING_OBSERVATION_TIMESTAMP = "missing_observation_timestamp"
    MISSING_RECEIPT_TIMESTAMP = "missing_receipt_timestamp"
    FUTURE_OBSERVATION = "future_observation"
    STALE_OBSERVATION = "stale_observation"
    INVALID_WIND_SPEED = "invalid_wind_speed"
    INVALID_WIND_GUST = "invalid_wind_gust"
    INVALID_WIND_DIRECTION = "invalid_wind_direction"
    INVALID_TEMPERATURE = "invalid_temperature"
    INVALID_PRESSURE = "invalid_pressure"
    INVALID_HUMIDITY = "invalid_humidity"
    INVALID_COORDINATES = "invalid_coordinates"
    MISSING_COORDINATES = "missing_coordinates"
    OUTSIDE_AREA = "outside_area"
    SURFACE_ELEVATION_MISSING = "surface_elevation_missing"
    SURFACE_OBSERVATION_INSUFFICIENT = "surface_observation_insufficient"
    ALTITUDE_APPLICABILITY_MISSING = "altitude_applicability_missing"
    ALTITUDE_NOT_APPLICABLE = "altitude_not_applicable"
    UNSUPPORTED_PRODUCT = "unsupported_product"
    RELEASE_ALTITUDE_UNAVAILABLE = "release_altitude_unavailable"
    MISSING_WIND_SPEED = "missing_wind_speed"
    MISSING_WIND_GUST = "missing_wind_gust"
    MISSING_TEMPERATURE = "missing_temperature"
    MISSING_PRESSURE = "missing_pressure"
    MISSING_HUMIDITY = "missing_humidity"
    WIND_ABOVE_LIMIT = "wind_above_limit"
    GUST_ABOVE_LIMIT = "gust_above_limit"
    TEMPERATURE_BELOW_LIMIT = "temperature_below_limit"
    TEMPERATURE_ABOVE_LIMIT = "temperature_above_limit"
    PRESSURE_OUT_OF_RANGE = "pressure_out_of_range"
    HUMIDITY_OUT_OF_RANGE = "humidity_out_of_range"


# Data-integrity bounds for a reported number. These are not release limits.
MAX_PLAUSIBLE_WIND_MPS = 200.0
MIN_PLAUSIBLE_TEMPERATURE_C = -100.0
MAX_PLAUSIBLE_TEMPERATURE_C = 70.0
MIN_PLAUSIBLE_PRESSURE_PA = 1_000.0
MAX_PLAUSIBLE_PRESSURE_PA = 110_000.0
EARTH_RADIUS_M = 6_371_000.0


@dataclass(frozen=True)
class WeatherObservation:
    """One weather snapshot. ``None`` means the field was not reported."""

    source: str
    provenance: Provenance
    clock: ClockDomain
    applicability: Applicability
    observed_at_s: float | None
    received_at_s: float | None
    latitude_deg: float | None
    longitude_deg: float | None
    elevation_m: float | None = None
    altitude_min_m: float | None = None
    altitude_max_m: float | None = None
    wind_speed_mps: float | None = None
    wind_direction_deg: float | None = None
    wind_gust_mps: float | None = None
    temperature_c: float | None = None
    pressure_pa: float | None = None
    relative_humidity_pct: float | None = None


@dataclass(frozen=True)
class WeatherFetch:
    """A provider read. ``reason_code`` is empty when ``observation`` is present."""

    observation: WeatherObservation | None
    reason_code: str
    detail: str = ""


class WeatherProvider(Protocol):
    def latest(self, now_s: float) -> WeatherFetch:
        """Return the cached observation. This method must not perform network I/O."""


@dataclass(frozen=True)
class MissionArea:
    latitude_deg: float
    longitude_deg: float
    radius_m: float

    def __post_init__(self) -> None:
        if not _in_range(self.latitude_deg, -90.0, 90.0):
            raise ValueError("mission latitude must be in [-90, 90]")
        if not _in_range(self.longitude_deg, -180.0, 180.0):
            raise ValueError("mission longitude must be in [-180, 180]")
        if not math.isfinite(self.radius_m) or self.radius_m <= 0.0:
            raise ValueError("radius_m must be positive")


@dataclass(frozen=True)
class WeatherConstraints:
    """Simulation release limits. ``simulation_only`` does not change the check.

    Setting it false does not turn the decision into an operational approval.
    """

    max_wind_mps: float
    max_gust_mps: float
    min_temperature_c: float
    max_temperature_c: float
    min_pressure_pa: float | None = None
    max_pressure_pa: float | None = None
    min_humidity_pct: float | None = None
    max_humidity_pct: float | None = None
    simulation_only: bool = True

    def __post_init__(self) -> None:
        if not _in_range(self.max_wind_mps, 0.0, MAX_PLAUSIBLE_WIND_MPS):
            raise ValueError("max_wind_mps is outside the simulation range")
        if not _in_range(self.max_gust_mps, 0.0, MAX_PLAUSIBLE_WIND_MPS):
            raise ValueError("max_gust_mps is outside the simulation range")
        if self.max_gust_mps < self.max_wind_mps:
            raise ValueError("max_gust_mps must not be below max_wind_mps")
        if self.min_temperature_c > self.max_temperature_c:
            raise ValueError("min_temperature_c must not exceed max_temperature_c")
        _optional_span("pressure", self.min_pressure_pa, self.max_pressure_pa)
        _optional_span("humidity", self.min_humidity_pct, self.max_humidity_pct)


@dataclass(frozen=True)
class WeatherPolicy:
    max_age_s: float
    max_future_s: float
    area: MissionArea
    surface_layer_m: float
    surface_product_ceiling_amsl_m: float
    constraints: WeatherConstraints
    unix_origin_s: float | None = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.max_age_s) or self.max_age_s <= 0.0:
            raise ValueError("max_age_s must be positive")
        if not math.isfinite(self.max_future_s) or self.max_future_s < 0.0:
            raise ValueError("max_future_s must be non-negative")
        if not math.isfinite(self.surface_layer_m) or self.surface_layer_m < 0.0:
            raise ValueError("surface_layer_m must be non-negative")
        if not math.isfinite(self.surface_product_ceiling_amsl_m):
            raise ValueError("surface_product_ceiling_amsl_m must be finite")
        if self.unix_origin_s is not None and not math.isfinite(self.unix_origin_s):
            raise ValueError("unix_origin_s must be finite")


@dataclass(frozen=True)
class WeatherDecision:
    time_s: float
    authorized: bool
    reason_code: str
    status: str
    source: str
    provenance: str | None
    clock: str | None
    observed_at_s: float | None
    received_at_s: float | None
    age_s: float | None
    latitude_deg: float | None
    longitude_deg: float | None
    applicability: str | None
    elevation_m: float | None
    altitude_min_m: float | None
    altitude_max_m: float | None
    release_altitude_amsl_m: float | None
    measurements: dict
    limits: dict
    operational_approval: bool = False

    def as_dict(self) -> dict:
        return {
            "time_s": self.time_s,
            "authorized": self.authorized,
            "reason_code": self.reason_code,
            "status": self.status,
            "source": self.source,
            "provenance": self.provenance,
            "clock": self.clock,
            "observed_at_s": self.observed_at_s,
            "received_at_s": self.received_at_s,
            "age_s": self.age_s,
            "latitude_deg": self.latitude_deg,
            "longitude_deg": self.longitude_deg,
            "applicability": self.applicability,
            "elevation_m": self.elevation_m,
            "altitude_min_m": self.altitude_min_m,
            "altitude_max_m": self.altitude_max_m,
            "release_altitude_amsl_m": self.release_altitude_amsl_m,
            "measurements": self.measurements,
            "limits": self.limits,
            "operational_approval": self.operational_approval,
        }


class WeatherAuthorizer:
    """Evaluates the provider's cache. It does not fetch weather itself."""

    def __init__(self, provider: WeatherProvider, policy: WeatherPolicy) -> None:
        self.provider = provider
        self.policy = policy

    def evaluate(self, now_s: float, release_altitude_amsl_m: float | None) -> WeatherDecision:
        return decide(self.provider.latest(now_s), self.policy, now_s, release_altitude_amsl_m)


def simulation_constraints() -> WeatherConstraints:
    """Knobs for the simulated release check, not a flight-manual limit."""
    return WeatherConstraints(
        max_wind_mps=12.0,
        max_gust_mps=18.0,
        min_temperature_c=-20.0,
        max_temperature_c=40.0,
    )


def simulation_policy(**overrides: object) -> WeatherPolicy:
    values: dict = {
        "max_age_s": 600.0,
        "max_future_s": 0.0,
        "area": MissionArea(47.0, -122.0, 50_000.0),
        "surface_layer_m": 150.0,
        "surface_product_ceiling_amsl_m": 300.0,
        "constraints": simulation_constraints(),
        "unix_origin_s": None,
    }
    values.update(overrides)
    return WeatherPolicy(**values)


def simulation_observation(**overrides: object) -> WeatherObservation:
    """A labeled simulation snapshot. Provenance stays ``SIMULATED`` unless the caller overrides it."""
    values: dict = {
        "source": "simulation",
        "provenance": Provenance.SIMULATED,
        "clock": ClockDomain.MISSION,
        "applicability": Applicability.SURFACE,
        "observed_at_s": 0.0,
        "received_at_s": 0.0,
        "latitude_deg": 47.0,
        "longitude_deg": -122.0,
        "elevation_m": 20.0,
        "wind_speed_mps": 3.0,
        "wind_direction_deg": 220.0,
        "wind_gust_mps": 5.0,
        "temperature_c": 12.0,
        "pressure_pa": 101_325.0,
        "relative_humidity_pct": 55.0,
    }
    values.update(overrides)
    return WeatherObservation(**values)


def decide(
    fetch: WeatherFetch,
    policy: WeatherPolicy,
    now_s: float,
    release_altitude_amsl_m: float | None,
) -> WeatherDecision:
    """First failing check wins. The same inputs and ``now_s`` always agree."""
    observation = fetch.observation
    if fetch.reason_code or observation is None:
        return _decision(
            now_s,
            observation,
            policy,
            release_altitude_amsl_m,
            status=Status.MISSING,
            reason=fetch.reason_code or Reason.MISSING_OBSERVATION,
            age_s=None,
        )

    mapped = _to_mission_clock(observation, policy)
    if isinstance(mapped, str):
        return _decision(
            now_s, observation, policy, release_altitude_amsl_m, status=Status.UNSUPPORTED, reason=mapped, age_s=None
        )
    observed_s, received_s = mapped
    if observed_s is None:
        return _decision(
            now_s,
            observation,
            policy,
            release_altitude_amsl_m,
            status=Status.MISSING,
            reason=Reason.MISSING_OBSERVATION_TIMESTAMP,
            age_s=None,
        )
    if received_s is None:
        return _decision(
            now_s,
            observation,
            policy,
            release_altitude_amsl_m,
            status=Status.MISSING,
            reason=Reason.MISSING_RECEIPT_TIMESTAMP,
            age_s=None,
        )

    age = now_s - observed_s
    if age < -policy.max_future_s:
        return _decision(
            now_s,
            observation,
            policy,
            release_altitude_amsl_m,
            status=Status.INVALID,
            reason=Reason.FUTURE_OBSERVATION,
            age_s=age,
        )
    if age > policy.max_age_s:
        return _decision(
            now_s,
            observation,
            policy,
            release_altitude_amsl_m,
            status=Status.STALE,
            reason=Reason.STALE_OBSERVATION,
            age_s=age,
        )

    integrity = _integrity(observation)
    if integrity is not None:
        return _decision(
            now_s,
            observation,
            policy,
            release_altitude_amsl_m,
            status=Status.INVALID,
            reason=integrity,
            age_s=age,
        )

    geographic = _geographic(observation, policy)
    if geographic is not None:
        status = Status.MISSING if geographic == Reason.MISSING_COORDINATES else Status.UNSUPPORTED
        if geographic == Reason.INVALID_COORDINATES:
            status = Status.INVALID
        return _decision(
            now_s, observation, policy, release_altitude_amsl_m, status=status, reason=geographic, age_s=age
        )

    if release_altitude_amsl_m is None or not math.isfinite(release_altitude_amsl_m):
        return _decision(
            now_s,
            observation,
            policy,
            release_altitude_amsl_m,
            status=Status.MISSING,
            reason=Reason.RELEASE_ALTITUDE_UNAVAILABLE,
            age_s=age,
        )

    applicability = _applicability(observation, policy, release_altitude_amsl_m)
    if applicability is not None:
        status = Status.MISSING if applicability == Reason.SURFACE_ELEVATION_MISSING else Status.UNSUPPORTED
        if applicability == Reason.ALTITUDE_APPLICABILITY_MISSING:
            status = Status.MISSING
        return _decision(
            now_s, observation, policy, release_altitude_amsl_m, status=status, reason=applicability, age_s=age
        )

    missing = _missing_measurement(observation, policy.constraints)
    if missing is not None:
        return _decision(
            now_s, observation, policy, release_altitude_amsl_m, status=Status.MISSING, reason=missing, age_s=age
        )

    limit = _limits(observation, policy.constraints)
    return _decision(
        now_s,
        observation,
        policy,
        release_altitude_amsl_m,
        status=Status.VALID,
        reason=limit or "",
        age_s=age,
    )


def _decision(
    now_s: float,
    observation: WeatherObservation | None,
    policy: WeatherPolicy,
    release_altitude_amsl_m: float | None,
    *,
    status: Status,
    reason: str,
    age_s: float | None,
) -> WeatherDecision:
    limits = {
        "max_wind_mps": policy.constraints.max_wind_mps,
        "max_gust_mps": policy.constraints.max_gust_mps,
        "min_temperature_c": policy.constraints.min_temperature_c,
        "max_temperature_c": policy.constraints.max_temperature_c,
        "min_pressure_pa": policy.constraints.min_pressure_pa,
        "max_pressure_pa": policy.constraints.max_pressure_pa,
        "min_humidity_pct": policy.constraints.min_humidity_pct,
        "max_humidity_pct": policy.constraints.max_humidity_pct,
        "max_age_s": policy.max_age_s,
        "radius_m": policy.area.radius_m,
        "surface_layer_m": policy.surface_layer_m,
        "surface_product_ceiling_amsl_m": policy.surface_product_ceiling_amsl_m,
        "simulation_only": policy.constraints.simulation_only,
    }
    if observation is None:
        return WeatherDecision(
            time_s=now_s,
            authorized=False,
            reason_code=reason,
            status=status.value,
            source="",
            provenance=None,
            clock=None,
            observed_at_s=None,
            received_at_s=None,
            age_s=age_s,
            latitude_deg=None,
            longitude_deg=None,
            applicability=None,
            elevation_m=None,
            altitude_min_m=None,
            altitude_max_m=None,
            release_altitude_amsl_m=_finite_or_none(release_altitude_amsl_m),
            measurements={},
            limits=limits,
        )
    return WeatherDecision(
        time_s=now_s,
        authorized=reason == "",
        reason_code=reason,
        status=status.value,
        source=observation.source,
        provenance=observation.provenance.value,
        clock=observation.clock.value,
        observed_at_s=_finite_or_none(observation.observed_at_s),
        received_at_s=_finite_or_none(observation.received_at_s),
        age_s=age_s,
        latitude_deg=_finite_or_none(observation.latitude_deg),
        longitude_deg=_finite_or_none(observation.longitude_deg),
        applicability=observation.applicability.value,
        elevation_m=_finite_or_none(observation.elevation_m),
        altitude_min_m=_finite_or_none(observation.altitude_min_m),
        altitude_max_m=_finite_or_none(observation.altitude_max_m),
        release_altitude_amsl_m=_finite_or_none(release_altitude_amsl_m),
        measurements={
            "wind_speed_mps": _finite_or_none(observation.wind_speed_mps),
            "wind_direction_deg": _finite_or_none(observation.wind_direction_deg),
            "wind_gust_mps": _finite_or_none(observation.wind_gust_mps),
            "temperature_c": _finite_or_none(observation.temperature_c),
            "pressure_pa": _finite_or_none(observation.pressure_pa),
            "relative_humidity_pct": _finite_or_none(observation.relative_humidity_pct),
        },
        limits=limits,
    )


def _to_mission_clock(
    observation: WeatherObservation, policy: WeatherPolicy
) -> tuple[float | None, float | None] | str:
    if observation.clock is ClockDomain.MISSION:
        return observation.observed_at_s, observation.received_at_s
    if observation.clock is ClockDomain.UNIX and policy.unix_origin_s is not None:
        origin = policy.unix_origin_s
        observed = None if observation.observed_at_s is None else observation.observed_at_s - origin
        received = None if observation.received_at_s is None else observation.received_at_s - origin
        return observed, received
    return Reason.CLOCK_DOMAIN_MISMATCH


def _integrity(observation: WeatherObservation) -> str | None:
    if _bad_number(observation.wind_speed_mps, 0.0, MAX_PLAUSIBLE_WIND_MPS):
        return Reason.INVALID_WIND_SPEED
    if _bad_number(observation.wind_gust_mps, 0.0, MAX_PLAUSIBLE_WIND_MPS):
        return Reason.INVALID_WIND_GUST
    if (
        _finite(observation.wind_speed_mps)
        and _finite(observation.wind_gust_mps)
        and observation.wind_gust_mps + 1e-9 < observation.wind_speed_mps
    ):
        return Reason.INVALID_WIND_GUST
    if _bad_number(observation.wind_direction_deg, 0.0, 360.0):
        return Reason.INVALID_WIND_DIRECTION
    if _bad_number(observation.temperature_c, MIN_PLAUSIBLE_TEMPERATURE_C, MAX_PLAUSIBLE_TEMPERATURE_C):
        return Reason.INVALID_TEMPERATURE
    if _bad_number(observation.pressure_pa, MIN_PLAUSIBLE_PRESSURE_PA, MAX_PLAUSIBLE_PRESSURE_PA):
        return Reason.INVALID_PRESSURE
    if _bad_number(observation.relative_humidity_pct, 0.0, 100.0):
        return Reason.INVALID_HUMIDITY
    if _bad_number(observation.latitude_deg, -90.0, 90.0) or _bad_number(observation.longitude_deg, -180.0, 180.0):
        return Reason.INVALID_COORDINATES
    if _bad_number(observation.elevation_m, -500.0, 10_000.0):
        return Reason.INVALID_COORDINATES
    return None


def _geographic(observation: WeatherObservation, policy: WeatherPolicy) -> str | None:
    if observation.latitude_deg is None or observation.longitude_deg is None:
        return Reason.MISSING_COORDINATES
    distance = horizontal_distance_m(
        observation.latitude_deg,
        observation.longitude_deg,
        policy.area.latitude_deg,
        policy.area.longitude_deg,
    )
    if distance > policy.area.radius_m:
        return Reason.OUTSIDE_AREA
    return None


def _applicability(observation: WeatherObservation, policy: WeatherPolicy, release_amsl_m: float) -> str | None:
    if observation.applicability is Applicability.FORECAST:
        return Reason.UNSUPPORTED_PRODUCT
    if observation.applicability is Applicability.SURFACE:
        if observation.elevation_m is None:
            return Reason.SURFACE_ELEVATION_MISSING
        if release_amsl_m > policy.surface_product_ceiling_amsl_m:
            return Reason.SURFACE_OBSERVATION_INSUFFICIENT
        if abs(release_amsl_m - observation.elevation_m) > policy.surface_layer_m:
            return Reason.SURFACE_OBSERVATION_INSUFFICIENT
        return None
    if observation.altitude_min_m is None or observation.altitude_max_m is None:
        return Reason.ALTITUDE_APPLICABILITY_MISSING
    if not (observation.altitude_min_m <= release_amsl_m <= observation.altitude_max_m):
        return Reason.ALTITUDE_NOT_APPLICABLE
    return None


def _missing_measurement(observation: WeatherObservation, constraints: WeatherConstraints) -> str | None:
    if observation.wind_speed_mps is None:
        return Reason.MISSING_WIND_SPEED
    if observation.wind_gust_mps is None:
        return Reason.MISSING_WIND_GUST
    if observation.temperature_c is None:
        return Reason.MISSING_TEMPERATURE
    if _pressure_limited(constraints) and observation.pressure_pa is None:
        return Reason.MISSING_PRESSURE
    if _humidity_limited(constraints) and observation.relative_humidity_pct is None:
        return Reason.MISSING_HUMIDITY
    return None


def _limits(observation: WeatherObservation, constraints: WeatherConstraints) -> str | None:
    assert observation.wind_speed_mps is not None
    assert observation.wind_gust_mps is not None
    assert observation.temperature_c is not None
    if observation.wind_speed_mps > constraints.max_wind_mps:
        return Reason.WIND_ABOVE_LIMIT
    if observation.wind_gust_mps > constraints.max_gust_mps:
        return Reason.GUST_ABOVE_LIMIT
    if observation.temperature_c < constraints.min_temperature_c:
        return Reason.TEMPERATURE_BELOW_LIMIT
    if observation.temperature_c > constraints.max_temperature_c:
        return Reason.TEMPERATURE_ABOVE_LIMIT
    if constraints.min_pressure_pa is not None and observation.pressure_pa is not None:
        if observation.pressure_pa < constraints.min_pressure_pa:
            return Reason.PRESSURE_OUT_OF_RANGE
    if constraints.max_pressure_pa is not None and observation.pressure_pa is not None:
        if observation.pressure_pa > constraints.max_pressure_pa:
            return Reason.PRESSURE_OUT_OF_RANGE
    if constraints.min_humidity_pct is not None and observation.relative_humidity_pct is not None:
        if observation.relative_humidity_pct < constraints.min_humidity_pct:
            return Reason.HUMIDITY_OUT_OF_RANGE
    if constraints.max_humidity_pct is not None and observation.relative_humidity_pct is not None:
        if observation.relative_humidity_pct > constraints.max_humidity_pct:
            return Reason.HUMIDITY_OUT_OF_RANGE
    return None


def horizontal_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    return 2.0 * EARTH_RADIUS_M * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def _pressure_limited(constraints: WeatherConstraints) -> bool:
    return constraints.min_pressure_pa is not None or constraints.max_pressure_pa is not None


def _humidity_limited(constraints: WeatherConstraints) -> bool:
    return constraints.min_humidity_pct is not None or constraints.max_humidity_pct is not None


def _finite(value: float | None) -> bool:
    return value is not None and math.isfinite(value)


def _finite_or_none(value: float | None) -> float | None:
    if value is None or not math.isfinite(value):
        return None
    return value


def _bad_number(value: float | None, low: float, high: float) -> bool:
    """True when a reported number is non-finite or outside the integrity span. Missing is allowed."""
    if value is None:
        return False
    return not math.isfinite(value) or value < low or value > high


def _in_range(value: float, low: float, high: float) -> bool:
    return math.isfinite(value) and low <= value <= high


def _optional_span(name: str, low: float | None, high: float | None) -> None:
    if low is not None and not math.isfinite(low):
        raise ValueError(f"min_{name} must be finite")
    if high is not None and not math.isfinite(high):
        raise ValueError(f"max_{name} must be finite")
    if low is not None and high is not None and low > high:
        raise ValueError(f"min_{name} must not exceed max_{name}")
