"""Altitude frames from PX4 sensor and estimator outputs.

This monitor does not estimate altitude. It reads one snapshot of the
autopilot's own products and reports each frame separately:

- barometric: ``vehicle_air_data.baro_alt_meter``
- local: height above the local NED origin, ``-vehicle_local_position.z``
- AMSL: ``sensor_gps.alt`` when a fix is present, otherwise
  ``vehicle_local_position.ref_alt`` plus local height
- AGL: ``vehicle_local_position.dist_bottom`` when that field is valid

A release decision names one of those frames. Local height is never substituted
for AGL, and barometric altitude is never substituted for AMSL.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


class AltitudeFrame(Enum):
    BAROMETRIC = "barometric"
    AMSL = "amsl"
    AGL = "agl"
    LOCAL = "local"


@dataclass(frozen=True)
class Px4AltitudeSnapshot:
    """Metres and seconds. GPS altitude is already converted from the message's units."""

    time_s: float
    baro_alt_m: float | None = None
    local_z_m: float | None = None
    local_vz_mps: float | None = None
    z_valid: bool = False
    vz_valid: bool = False
    dist_bottom_m: float | None = None
    dist_bottom_valid: bool = False
    ref_alt_m: float | None = None
    gps_alt_amsl_m: float | None = None
    baro_fault: bool = False


@dataclass(frozen=True)
class FrameAltitude:
    frame: AltitudeFrame
    altitude_m: float | None
    source: str
    reason: str = ""


@dataclass(frozen=True)
class AltitudeReport:
    time_s: float
    sample_time_s: float
    age_s: float
    fresh: bool
    fresh_reason: str
    vertical_up_mps: float | None
    estimator_ok: bool
    estimator_reason: str
    barometric: FrameAltitude
    amsl: FrameAltitude
    agl: FrameAltitude
    local: FrameAltitude
    baro_amsl_delta_m: float | None

    def frame(self, which: AltitudeFrame) -> FrameAltitude:
        return {
            AltitudeFrame.BAROMETRIC: self.barometric,
            AltitudeFrame.AMSL: self.amsl,
            AltitudeFrame.AGL: self.agl,
            AltitudeFrame.LOCAL: self.local,
        }[which]


@dataclass(frozen=True)
class AltitudeBand:
    """Authorized release altitudes in one named frame."""

    frame: AltitudeFrame
    floor_m: float
    ceiling_m: float
    agreement_tolerance_m: float = 30.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.floor_m) or not math.isfinite(self.ceiling_m):
            raise ValueError("altitude band bounds must be finite")
        if self.floor_m > self.ceiling_m:
            raise ValueError("floor_m must not exceed ceiling_m")
        if self.agreement_tolerance_m < 0.0:
            raise ValueError("agreement_tolerance_m must be non-negative")

    def decide(self, report: AltitudeReport) -> AltitudeDecision:
        reading = report.frame(self.frame)
        cross, delta = _cross_check(report.baro_amsl_delta_m, self.agreement_tolerance_m)
        inside = _inside(reading.altitude_m, self.floor_m, self.ceiling_m)
        reason = _denial(self, report, reading, cross, inside)
        return AltitudeDecision(
            time_s=report.time_s,
            frame=self.frame.value,
            altitude_m=reading.altitude_m,
            source=reading.source,
            vertical_up_mps=report.vertical_up_mps,
            fresh=report.fresh,
            estimator_ok=report.estimator_ok,
            inside_band=inside,
            cross_check=cross,
            baro_amsl_delta_m=delta,
            authorized=reason == "",
            reason=reason,
        )


@dataclass(frozen=True)
class AltitudeDecision:
    time_s: float
    frame: str
    altitude_m: float | None
    source: str
    vertical_up_mps: float | None
    fresh: bool
    estimator_ok: bool
    inside_band: bool | None
    cross_check: str
    baro_amsl_delta_m: float | None
    authorized: bool
    reason: str

    def as_dict(self) -> dict:
        return {
            "time_s": self.time_s,
            "frame": self.frame,
            "altitude_m": self.altitude_m,
            "source": self.source,
            "vertical_up_mps": self.vertical_up_mps,
            "fresh": self.fresh,
            "estimator_ok": self.estimator_ok,
            "inside_band": self.inside_band,
            "cross_check": self.cross_check,
            "baro_amsl_delta_m": self.baro_amsl_delta_m,
            "authorized": self.authorized,
            "reason": self.reason,
        }


class AltitudeMonitor:
    """Freshness and frame selection for one stream of PX4 altitude snapshots."""

    def __init__(self, max_age_s: float = 0.5) -> None:
        if max_age_s <= 0.0:
            raise ValueError("max_age_s must be positive")
        self.max_age_s = max_age_s

    def update(self, snapshot: Px4AltitudeSnapshot, now_s: float) -> AltitudeReport:
        age = now_s - snapshot.time_s
        if age < 0.0:
            fresh, fresh_reason = False, "altitude sample is from the future"
        elif age > self.max_age_s:
            fresh, fresh_reason = False, "altitude sample is stale"
        else:
            fresh, fresh_reason = True, ""

        local = _local(snapshot)
        barometric = _barometric(snapshot)
        amsl = _amsl(snapshot, local)
        agl = _agl(snapshot)
        estimator_ok, estimator_reason = _estimator(snapshot)
        vertical = -snapshot.local_vz_mps if _finite(snapshot.local_vz_mps) else None
        delta = None
        if barometric.altitude_m is not None and amsl.altitude_m is not None:
            delta = barometric.altitude_m - amsl.altitude_m
        return AltitudeReport(
            time_s=now_s,
            sample_time_s=snapshot.time_s,
            age_s=age,
            fresh=fresh,
            fresh_reason=fresh_reason,
            vertical_up_mps=vertical,
            estimator_ok=estimator_ok,
            estimator_reason=estimator_reason,
            barometric=barometric,
            amsl=amsl,
            agl=agl,
            local=local,
            baro_amsl_delta_m=delta,
        )


def _finite(value: float | None) -> bool:
    return value is not None and math.isfinite(value)


def _reading(frame: AltitudeFrame, altitude: float | None, source: str, reason: str) -> FrameAltitude:
    if reason:
        return FrameAltitude(frame, None, "", reason)
    return FrameAltitude(frame, altitude, source, "")


def _local(snapshot: Px4AltitudeSnapshot) -> FrameAltitude:
    if snapshot.z_valid and _finite(snapshot.local_z_m):
        assert snapshot.local_z_m is not None
        return _reading(AltitudeFrame.LOCAL, -snapshot.local_z_m, "vehicle_local_position.z", "")
    return _reading(AltitudeFrame.LOCAL, None, "", "local altitude unavailable")


def _barometric(snapshot: Px4AltitudeSnapshot) -> FrameAltitude:
    if snapshot.baro_fault:
        return _reading(AltitudeFrame.BAROMETRIC, None, "", "barometer fault")
    if not _finite(snapshot.baro_alt_m):
        return _reading(AltitudeFrame.BAROMETRIC, None, "", "barometric altitude missing")
    return _reading(AltitudeFrame.BAROMETRIC, snapshot.baro_alt_m, "vehicle_air_data.baro_alt_meter", "")


def _amsl(snapshot: Px4AltitudeSnapshot, local: FrameAltitude) -> FrameAltitude:
    if _finite(snapshot.gps_alt_amsl_m):
        return _reading(AltitudeFrame.AMSL, snapshot.gps_alt_amsl_m, "sensor_gps.alt", "")
    if local.altitude_m is not None and _finite(snapshot.ref_alt_m):
        assert snapshot.ref_alt_m is not None
        return _reading(
            AltitudeFrame.AMSL,
            snapshot.ref_alt_m + local.altitude_m,
            "vehicle_local_position.ref_alt",
            "",
        )
    return _reading(AltitudeFrame.AMSL, None, "", "AMSL unavailable")


def _agl(snapshot: Px4AltitudeSnapshot) -> FrameAltitude:
    if snapshot.dist_bottom_valid and _finite(snapshot.dist_bottom_m):
        return _reading(AltitudeFrame.AGL, snapshot.dist_bottom_m, "vehicle_local_position.dist_bottom", "")
    return _reading(AltitudeFrame.AGL, None, "", "AGL unavailable")


def _estimator(snapshot: Px4AltitudeSnapshot) -> tuple[bool, str]:
    if not snapshot.z_valid or not _finite(snapshot.local_z_m):
        return False, "estimated altitude invalid"
    if not snapshot.vz_valid or not _finite(snapshot.local_vz_mps):
        return False, "vertical velocity invalid"
    return True, ""


def _cross_check(delta: float | None, tolerance_m: float) -> tuple[str, float | None]:
    if delta is None:
        return "not_compared", None
    if abs(delta) > tolerance_m:
        return "diverge", delta
    return "agree", delta


def _inside(altitude: float | None, floor_m: float, ceiling_m: float) -> bool | None:
    if altitude is None:
        return None
    return floor_m <= altitude <= ceiling_m


def _denial(
    band: AltitudeBand,
    report: AltitudeReport,
    reading: FrameAltitude,
    cross: str,
    inside: bool | None,
) -> str:
    if not report.fresh:
        return report.fresh_reason
    if not report.estimator_ok:
        return report.estimator_reason
    if reading.altitude_m is None:
        return reading.reason
    if band.frame in (AltitudeFrame.BAROMETRIC, AltitudeFrame.AMSL) and cross == "diverge":
        assert report.baro_amsl_delta_m is not None
        return f"barometric altitude and AMSL differ by {abs(report.baro_amsl_delta_m):.1f} m"
    if inside is False and reading.altitude_m < band.floor_m:
        return (
            f"{band.frame.value} altitude {reading.altitude_m:.1f} m "
            f"is below the {band.floor_m:.1f} m floor"
        )
    if inside is False:
        return (
            f"{band.frame.value} altitude {reading.altitude_m:.1f} m "
            f"is above the {band.ceiling_m:.1f} m ceiling"
        )
    return ""
