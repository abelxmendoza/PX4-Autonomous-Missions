"""Airspace as a release constraint.

A chart is an immutable snapshot. The release point is the seeding profile's
north/east, offset from the policy origin, at the release-point AMSL. A volume
on the chart can deny the dispenser. An empty chart does not mean the sky is
clear: the dataset has to say which area it covers. A passing decision is a
software check. It is not an FAA authorization and it is not an approval to fly.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Protocol, Sequence

from .weather import ClockDomain, Provenance, horizontal_distance_m

EARTH_RADIUS_M = 6_371_000.0


class AirspaceKind(Enum):
    PROHIBITED = "prohibited"
    RESTRICTED = "restricted"
    CONTROLLED = "controlled"
    TFR = "tfr"


class Reason:
    MISSING_AIRSPACE = "missing_airspace"
    PROVIDER_FAILURE = "provider_failure"
    CLOCK_DOMAIN_MISMATCH = "clock_domain_mismatch"
    MISSING_OBSERVATION_TIMESTAMP = "missing_observation_timestamp"
    MISSING_RECEIPT_TIMESTAMP = "missing_receipt_timestamp"
    FUTURE_AIRSPACE = "future_airspace"
    STALE_AIRSPACE = "stale_airspace"
    COVERAGE_MISSING = "coverage_missing"
    OUTSIDE_CHART_COVERAGE = "outside_chart_coverage"
    RELEASE_ALTITUDE_UNAVAILABLE = "release_altitude_unavailable"
    INSIDE_PROHIBITED = "inside_prohibited_airspace"
    INSIDE_RESTRICTED = "inside_restricted_airspace"
    INSIDE_TFR = "inside_active_tfr"
    TFR_WINDOW_MISSING = "tfr_window_missing"
    INSIDE_CONTROLLED = "inside_controlled_airspace"
    INVALID_AIRSPACE = "invalid_airspace"


_SEVERITY = {
    Reason.INSIDE_PROHIBITED: 0,
    Reason.INSIDE_RESTRICTED: 1,
    Reason.TFR_WINDOW_MISSING: 2,
    Reason.INSIDE_TFR: 3,
    Reason.INSIDE_CONTROLLED: 4,
}


@dataclass(frozen=True)
class AirspaceVolume:
    """Inclusive lat/lon box and AMSL band. Times are on the dataset clock."""

    name: str
    kind: AirspaceKind
    latitude_min: float
    latitude_max: float
    longitude_min: float
    longitude_max: float
    floor_amsl_m: float
    ceiling_amsl_m: float
    effective_from_s: float | None = None
    effective_until_s: float | None = None

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("airspace volume needs a name")
        if self.latitude_min > self.latitude_max or self.longitude_min > self.longitude_max:
            raise ValueError(f"{self.name} bounds are reversed")
        if not math.isfinite(self.floor_amsl_m) or not math.isfinite(self.ceiling_amsl_m):
            raise ValueError(f"{self.name} altitude must be finite")
        if self.floor_amsl_m > self.ceiling_amsl_m:
            raise ValueError(f"{self.name} floor is above its ceiling")
        for value in (self.effective_from_s, self.effective_until_s):
            if value is not None and not math.isfinite(value):
                raise ValueError(f"{self.name} effective time must be finite")
        if (
            self.effective_from_s is not None
            and self.effective_until_s is not None
            and self.effective_from_s > self.effective_until_s
        ):
            raise ValueError(f"{self.name} effective window is reversed")


@dataclass(frozen=True)
class AirspaceDataset:
    source: str
    provenance: Provenance
    clock: ClockDomain
    observed_at_s: float | None
    received_at_s: float | None
    coverage_latitude_deg: float | None
    coverage_longitude_deg: float | None
    coverage_radius_m: float | None
    volumes: tuple[AirspaceVolume, ...] = ()


@dataclass(frozen=True)
class AirspaceFetch:
    dataset: AirspaceDataset | None
    reason_code: str
    detail: str = ""


class AirspaceProvider(Protocol):
    def latest(self, now_s: float) -> AirspaceFetch:
        """Return the cached chart. This method must not perform network I/O."""


@dataclass(frozen=True)
class AirspacePolicy:
    """``authorization_id`` is a simulation token for controlled airspace only.

    It does not waive a prohibited area, a restricted area, or a TFR, and it
    does not set ``operational_approval``.
    """

    max_age_s: float = 600.0
    max_future_s: float = 0.0
    origin_latitude_deg: float = 47.0
    origin_longitude_deg: float = -122.0
    unix_origin_s: float | None = None
    authorization_id: str | None = None

    def __post_init__(self) -> None:
        if self.max_age_s <= 0.0 or self.max_future_s < 0.0:
            raise ValueError("airspace age limits must be non-negative, and max_age_s positive")
        if not (-90.0 <= self.origin_latitude_deg <= 90.0):
            raise ValueError("origin latitude must be in [-90, 90]")
        if not (-180.0 <= self.origin_longitude_deg <= 180.0):
            raise ValueError("origin longitude must be in [-180, 180]")


@dataclass(frozen=True)
class AirspaceDecision:
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
    release_latitude_deg: float | None
    release_longitude_deg: float | None
    release_altitude_amsl_m: float | None
    volume_name: str | None
    volume_kind: str | None
    authorization_id: str | None
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
            "release_latitude_deg": self.release_latitude_deg,
            "release_longitude_deg": self.release_longitude_deg,
            "release_altitude_amsl_m": self.release_altitude_amsl_m,
            "volume_name": self.volume_name,
            "volume_kind": self.volume_kind,
            "authorization_id": self.authorization_id,
            "operational_approval": self.operational_approval,
        }


class AirspaceAuthorizer:
    def __init__(self, provider: AirspaceProvider, policy: AirspacePolicy | None = None) -> None:
        self.provider = provider
        self.policy = policy or AirspacePolicy()

    def evaluate(
        self,
        now_s: float,
        north_m: float,
        east_m: float,
        release_altitude_amsl_m: float | None,
    ) -> AirspaceDecision:
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
            latitude,
            longitude,
            release_altitude_amsl_m,
        )


class StaticAirspaceProvider:
    def __init__(self, dataset: AirspaceDataset | None = None, *, failure: str | None = None) -> None:
        self._dataset = dataset
        self._failure = failure

    def latest(self, now_s: float) -> AirspaceFetch:
        del now_s
        if self._failure:
            return AirspaceFetch(None, Reason.PROVIDER_FAILURE, self._failure)
        if self._dataset is None:
            return AirspaceFetch(None, Reason.MISSING_AIRSPACE, "no chart")
        return AirspaceFetch(self._dataset, "")


class SimulatedAirspaceProvider:
    def __init__(self, segments: Sequence[tuple[float, AirspaceDataset | None]]) -> None:
        if not segments:
            raise ValueError("simulated airspace needs at least one segment")
        self._segments = tuple(sorted(segments, key=lambda item: item[0]))

    def latest(self, now_s: float) -> AirspaceFetch:
        active: tuple[float, AirspaceDataset | None] | None = None
        for segment in self._segments:
            if segment[0] <= now_s:
                active = segment
        if active is None or active[1] is None:
            return AirspaceFetch(None, Reason.MISSING_AIRSPACE, "no chart")
        return AirspaceFetch(active[1], "")


def simulation_dataset(**overrides: object) -> AirspaceDataset:
    values: dict = {
        "source": "simulation",
        "provenance": Provenance.SIMULATED,
        "clock": ClockDomain.MISSION,
        "observed_at_s": 0.0,
        "received_at_s": 0.0,
        "coverage_latitude_deg": 47.0,
        "coverage_longitude_deg": -122.0,
        "coverage_radius_m": 100_000.0,
        "volumes": (),
    }
    values.update(overrides)
    volumes = values["volumes"]
    values["volumes"] = tuple(volumes)
    return AirspaceDataset(**values)


def simulation_authorizer(
    dataset: AirspaceDataset | None = None,
    policy: AirspacePolicy | None = None,
) -> AirspaceAuthorizer:
    return AirspaceAuthorizer(StaticAirspaceProvider(dataset or simulation_dataset()), policy)


def offset_latlon(latitude_deg: float, longitude_deg: float, north_m: float, east_m: float) -> tuple[float, float]:
    latitude = latitude_deg + math.degrees(north_m / EARTH_RADIUS_M)
    cosine = math.cos(math.radians(latitude_deg))
    longitude = longitude_deg if abs(cosine) < 1e-12 else longitude_deg + math.degrees(east_m / (EARTH_RADIUS_M * cosine))
    return latitude, longitude


def decide(
    fetch: AirspaceFetch,
    policy: AirspacePolicy,
    now_s: float,
    latitude_deg: float,
    longitude_deg: float,
    release_altitude_amsl_m: float | None,
) -> AirspaceDecision:
    dataset = fetch.dataset
    if fetch.reason_code or dataset is None:
        return _decision(
            now_s, dataset, policy, latitude_deg, longitude_deg, release_altitude_amsl_m,
            status="missing", reason=fetch.reason_code or Reason.MISSING_AIRSPACE, age_s=None,
        )
    mapped = _to_mission_clock(dataset, policy)
    if isinstance(mapped, str):
        return _decision(
            now_s, dataset, policy, latitude_deg, longitude_deg, release_altitude_amsl_m,
            status="unsupported", reason=mapped, age_s=None,
        )
    observed_s, received_s = mapped
    if observed_s is None:
        return _decision(
            now_s, dataset, policy, latitude_deg, longitude_deg, release_altitude_amsl_m,
            status="missing", reason=Reason.MISSING_OBSERVATION_TIMESTAMP, age_s=None,
        )
    if received_s is None:
        return _decision(
            now_s, dataset, policy, latitude_deg, longitude_deg, release_altitude_amsl_m,
            status="missing", reason=Reason.MISSING_RECEIPT_TIMESTAMP, age_s=None,
        )
    age = now_s - observed_s
    if age < -policy.max_future_s:
        return _decision(
            now_s, dataset, policy, latitude_deg, longitude_deg, release_altitude_amsl_m,
            status="invalid", reason=Reason.FUTURE_AIRSPACE, age_s=age,
        )
    if age > policy.max_age_s:
        return _decision(
            now_s, dataset, policy, latitude_deg, longitude_deg, release_altitude_amsl_m,
            status="stale", reason=Reason.STALE_AIRSPACE, age_s=age,
        )
    if (
        dataset.coverage_latitude_deg is None
        or dataset.coverage_longitude_deg is None
        or dataset.coverage_radius_m is None
        or not math.isfinite(dataset.coverage_radius_m)
        or dataset.coverage_radius_m <= 0.0
    ):
        return _decision(
            now_s, dataset, policy, latitude_deg, longitude_deg, release_altitude_amsl_m,
            status="missing", reason=Reason.COVERAGE_MISSING, age_s=age,
        )
    distance = horizontal_distance_m(
        latitude_deg, longitude_deg, dataset.coverage_latitude_deg, dataset.coverage_longitude_deg
    )
    if distance > dataset.coverage_radius_m:
        return _decision(
            now_s, dataset, policy, latitude_deg, longitude_deg, release_altitude_amsl_m,
            status="unsupported", reason=Reason.OUTSIDE_CHART_COVERAGE, age_s=age,
        )
    if release_altitude_amsl_m is None or not math.isfinite(release_altitude_amsl_m):
        return _decision(
            now_s, dataset, policy, latitude_deg, longitude_deg, release_altitude_amsl_m,
            status="missing", reason=Reason.RELEASE_ALTITUDE_UNAVAILABLE, age_s=age,
        )
    hit = _blocking_volume(dataset, policy, now_s, latitude_deg, longitude_deg, release_altitude_amsl_m)
    if hit is None:
        return _decision(
            now_s, dataset, policy, latitude_deg, longitude_deg, release_altitude_amsl_m,
            status="valid", reason="", age_s=age,
        )
    reason, volume = hit
    return _decision(
        now_s, dataset, policy, latitude_deg, longitude_deg, release_altitude_amsl_m,
        status="valid", reason=reason, age_s=age, volume=volume,
    )


def _blocking_volume(
    dataset: AirspaceDataset,
    policy: AirspacePolicy,
    now_s: float,
    latitude_deg: float,
    longitude_deg: float,
    release_amsl_m: float,
) -> tuple[str, AirspaceVolume] | None:
    found: tuple[str, AirspaceVolume] | None = None
    for volume in dataset.volumes:
        if not _inside_box(volume, latitude_deg, longitude_deg, release_amsl_m):
            continue
        reason = _volume_reason(volume, dataset, policy, now_s)
        if reason is None:
            continue
        if found is None or _SEVERITY[reason] < _SEVERITY[found[0]]:
            found = (reason, volume)
    return found


def _volume_reason(
    volume: AirspaceVolume,
    dataset: AirspaceDataset,
    policy: AirspacePolicy,
    now_s: float,
) -> str | None:
    if volume.kind is AirspaceKind.TFR and (volume.effective_from_s is None or volume.effective_until_s is None):
        return Reason.TFR_WINDOW_MISSING
    if not _in_effect(volume, dataset, policy, now_s):
        return None
    if volume.kind is AirspaceKind.PROHIBITED:
        return Reason.INSIDE_PROHIBITED
    if volume.kind is AirspaceKind.RESTRICTED:
        return Reason.INSIDE_RESTRICTED
    if volume.kind is AirspaceKind.TFR:
        return Reason.INSIDE_TFR
    if volume.kind is AirspaceKind.CONTROLLED and not policy.authorization_id:
        return Reason.INSIDE_CONTROLLED
    return None


def _in_effect(volume: AirspaceVolume, dataset: AirspaceDataset, policy: AirspacePolicy, now_s: float) -> bool:
    start = _map_optional(volume.effective_from_s, dataset, policy)
    end = _map_optional(volume.effective_until_s, dataset, policy)
    if start is not None and now_s < start:
        return False
    if end is not None and now_s > end:
        return False
    return True


def _inside_box(volume: AirspaceVolume, latitude: float, longitude: float, altitude_m: float) -> bool:
    return (
        volume.latitude_min <= latitude <= volume.latitude_max
        and volume.longitude_min <= longitude <= volume.longitude_max
        and volume.floor_amsl_m <= altitude_m <= volume.ceiling_amsl_m
    )


def _map_optional(value: float | None, dataset: AirspaceDataset, policy: AirspacePolicy) -> float | None:
    if value is None or dataset.clock is ClockDomain.MISSION:
        return value
    assert policy.unix_origin_s is not None
    return value - policy.unix_origin_s


def _to_mission_clock(dataset: AirspaceDataset, policy: AirspacePolicy) -> tuple[float | None, float | None] | str:
    if dataset.clock is ClockDomain.MISSION:
        return dataset.observed_at_s, dataset.received_at_s
    if dataset.clock is ClockDomain.UNIX and policy.unix_origin_s is not None:
        observed = None if dataset.observed_at_s is None else dataset.observed_at_s - policy.unix_origin_s
        received = None if dataset.received_at_s is None else dataset.received_at_s - policy.unix_origin_s
        return observed, received
    return Reason.CLOCK_DOMAIN_MISMATCH


def _decision(
    now_s: float,
    dataset: AirspaceDataset | None,
    policy: AirspacePolicy,
    latitude_deg: float,
    longitude_deg: float,
    release_altitude_amsl_m: float | None,
    *,
    status: str,
    reason: str,
    age_s: float | None,
    volume: AirspaceVolume | None = None,
) -> AirspaceDecision:
    return AirspaceDecision(
        time_s=now_s,
        authorized=reason == "",
        reason_code=reason,
        status=status,
        source="" if dataset is None else dataset.source,
        provenance=None if dataset is None else dataset.provenance.value,
        clock=None if dataset is None else dataset.clock.value,
        observed_at_s=None if dataset is None else dataset.observed_at_s,
        received_at_s=None if dataset is None else dataset.received_at_s,
        age_s=age_s,
        release_latitude_deg=latitude_deg,
        release_longitude_deg=longitude_deg,
        release_altitude_amsl_m=release_altitude_amsl_m if release_altitude_amsl_m is not None and math.isfinite(release_altitude_amsl_m) else None,
        volume_name=None if volume is None else volume.name,
        volume_kind=None if volume is None else volume.kind.value,
        authorization_id=policy.authorization_id or None,
    )
