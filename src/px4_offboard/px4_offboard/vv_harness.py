"""Verification & Validation harness for offline flight-log replay.

Maps named requirements to deterministic checks against a ``FlightTrace``.
Designed so interviewers can see: requirement ID → evidence → PASS/FAIL.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from px4_offboard.flight_replay import (
    AIRBORNE_STATES,
    TERMINAL_STATES,
    FlightTrace,
    iter_transitions,
)
from px4_offboard.localization_logic import DEFAULT_GPS_DENIED_ZONE, GpsDeniedZone
from px4_offboard.mission_logic import Fence
from px4_offboard.mission_state import LEGAL_TRANSITIONS, State


# Default keep-in matches config/offboard_mission.yaml
DEFAULT_FENCE = Fence(
    north_min=-5.0,
    north_max=55.0,
    east_min=-23.0,
    east_max=17.0,
    altitude_max=12.0,
)

# Allowed consecutive state pairs observed in logs (same-state collapsed).
_STATE_NAME = {s.name: s for s in State}


class Severity(Enum):
    MUST = "must"
    SHOULD = "should"


@dataclass(frozen=True)
class Requirement:
    id: str
    title: str
    severity: Severity
    description: str


REQUIREMENTS: tuple[Requirement, ...] = (
    Requirement(
        id="REQ-STATE-01",
        title="Legal mission state sequence",
        severity=Severity.MUST,
        description="Logged state transitions must obey the mission state machine.",
    ),
    Requirement(
        id="REQ-WP-01",
        title="Waypoint index monotonic",
        severity=Severity.MUST,
        description="wp_index must never decrease during a mission.",
    ),
    Requirement(
        id="REQ-GEOCAGE-01",
        title="Geo-cage clamps setpoints",
        severity=Severity.MUST,
        description="When caged=1, the commanded setpoint must lie inside the fence.",
    ),
    Requirement(
        id="REQ-GEOFENCE-01",
        title="Hard geofence → FAILSAFE",
        severity=Severity.MUST,
        description=(
            "If geofence is enabled and position is outside while airborne, "
            "FAILSAFE must appear within the response window."
        ),
    ),
    Requirement(
        id="REQ-ALT-01",
        title="Altitude keep-in",
        severity=Severity.MUST,
        description="Altitude must stay within fence altitude_max while geofence is on, "
        "unless already in FAILSAFE.",
    ),
    Requirement(
        id="REQ-TERM-01",
        title="Terminal state finality",
        severity=Severity.MUST,
        description="After LANDING or FAILSAFE, the mission must not return to MOVE.",
    ),
    Requirement(
        id="REQ-EXEC-01",
        title="Executive abort consistency",
        severity=Severity.SHOULD,
        description="If executive_mode becomes ABORT, FAILSAFE or LANDING should follow.",
    ),
    Requirement(
        id="REQ-AVOID-SENSOR-01",
        title="Avoidance backed by live sensor evidence",
        severity=Severity.MUST,
        description="Every direct avoidance classification must have fresh LiDAR range evidence.",
    ),
    Requirement(
        id="REQ-CLEARANCE-01",
        title="No mapped-obstacle intersection",
        severity=Severity.MUST,
        description="The vehicle must retain positive clearance from every mapped obstacle.",
    ),
    Requirement(
        id="REQ-GPS-ZONE-01",
        title="GPS-denied zone flag consistency",
        severity=Severity.SHOULD,
        description=(
            "When GPS-denied columns are present, in_gps_denied_zone must match "
            "whether the sample position lies inside the configured denied AABB."
        ),
    ),
    Requirement(
        id="REQ-GPS-INJECT-01",
        title="Injected deny does not claim healthy GPS",
        severity=Severity.MUST,
        description=(
            "While gps_injected_deny is set, loc_source must not claim healthy GPS."
        ),
    ),
    Requirement(
        id="REQ-GPS-POLICY-01",
        title="Localization failsafe reaches terminal/hold",
        severity=Severity.MUST,
        description=(
            "If loc_event records LOC_FAILSAFE, FAILSAFE or LANDING must appear "
            "within the response window (Pass-1 hold/land policy)."
        ),
    ),
    Requirement(
        id="REQ-GPS-ACTUAL-01",
        title="PX4 GPS sensor actually becomes unavailable",
        severity=Severity.MUST,
        description="After failure injection settles, raw GPS health must be false.",
    ),
    Requirement(
        id="REQ-VIO-FUSION-01",
        title="External vision sustains GPS-denied flight",
        severity=Severity.MUST,
        description="PX4 EKF must fuse external-vision position while GPS is unavailable.",
    ),
)


@dataclass
class CheckResult:
    requirement_id: str
    title: str
    severity: Severity
    passed: bool
    detail: str
    evidence: list[str] = field(default_factory=list)
    skipped: bool = False


@dataclass
class VvReport:
    source: str
    results: list[CheckResult]
    summary: dict = field(default_factory=dict)

    @property
    def must_failed(self) -> list[CheckResult]:
        return [
            r
            for r in self.results
            if not r.skipped and not r.passed and r.severity is Severity.MUST
        ]

    @property
    def passed(self) -> bool:
        return not self.must_failed

    def format_text(self) -> str:
        lines = [
            f"V&V report — {self.source}",
            f"samples={self.summary.get('samples', 0)}  "
            f"duration_s={self.summary.get('duration_s', 0)}  "
            f"states={self.summary.get('state_sequence', [])}",
            "",
        ]
        for result in self.results:
            if result.skipped:
                status = "SKIP"
            elif result.passed:
                status = "PASS"
            else:
                status = "FAIL"
            lines.append(
                f"[{status}] {result.requirement_id} ({result.severity.value}) "
                f"{result.title}"
            )
            lines.append(f"       {result.detail}")
            for item in result.evidence[:5]:
                lines.append(f"         • {item}")
        lines.append("")
        lines.append(
            "RESULT: "
            + ("PASS (all MUST requirements satisfied)" if self.passed else "FAIL")
        )
        return "\n".join(lines)


def _req(req_id: str) -> Requirement:
    for requirement in REQUIREMENTS:
        if requirement.id == req_id:
            return requirement
    raise KeyError(req_id)


def check_legal_state_sequence(trace: FlightTrace) -> CheckResult:
    req = _req("REQ-STATE-01")
    if not trace.samples:
        return CheckResult(
            req.id, req.title, req.severity, False, "empty log", skipped=False
        )

    illegal: list[str] = []
    for from_name, to_name, t_s in iter_transitions(trace):
        if from_name not in _STATE_NAME or to_name not in _STATE_NAME:
            illegal.append(f"t={t_s:.2f}s unknown state {from_name}->{to_name}")
            continue
        allowed = {s.name for s in LEGAL_TRANSITIONS[_STATE_NAME[from_name]]}
        if to_name not in allowed:
            illegal.append(f"t={t_s:.2f}s illegal {from_name}->{to_name}")

    # Logs often start mid-mission in MOVE — allow first observed state freely.
    passed = not illegal
    return CheckResult(
        req.id,
        req.title,
        req.severity,
        passed,
        "all transitions legal" if passed else f"{len(illegal)} illegal transition(s)",
        evidence=illegal,
    )


def check_wp_monotonic(trace: FlightTrace) -> CheckResult:
    req = _req("REQ-WP-01")
    drops: list[str] = []
    prev = -1
    for sample in trace.samples:
        if sample.wp_index < prev:
            drops.append(
                f"t={sample.t_s:.2f}s wp {prev}->{sample.wp_index} in {sample.state}"
            )
        prev = sample.wp_index
    passed = not drops
    return CheckResult(
        req.id,
        req.title,
        req.severity,
        passed,
        "wp_index non-decreasing" if passed else f"{len(drops)} decrease(s)",
        evidence=drops,
    )


def check_geocage_setpoints(
    trace: FlightTrace,
    fence: Fence = DEFAULT_FENCE,
    margin: float = 1.0,
) -> CheckResult:
    req = _req("REQ-GEOCAGE-01")
    violations: list[str] = []
    checked = 0
    for sample in trace.samples:
        if not sample.caged:
            continue
        checked += 1
        clamped, changed = fence.clamp(list(sample.setpoint), margin=margin)
        # Setpoint should already be inside; re-clamp must not change it.
        if changed:
            violations.append(
                f"t={sample.t_s:.2f}s setpoint {sample.setpoint} "
                f"outside cage (would clamp to {tuple(clamped)})"
            )
    if checked == 0:
        return CheckResult(
            req.id,
            req.title,
            req.severity,
            True,
            "no caged samples (N/A — treated as pass)",
            skipped=False,
        )
    passed = not violations
    return CheckResult(
        req.id,
        req.title,
        req.severity,
        passed,
        f"checked {checked} caged setpoints"
        + ("" if passed else f"; {len(violations)} outside"),
        evidence=violations,
    )


def check_geofence_response(
    trace: FlightTrace,
    response_window_s: float = 1.5,
) -> CheckResult:
    """O(N) rising-edge check: only evaluate each breach onset once."""
    req = _req("REQ-GEOFENCE-01")
    samples = trace.samples
    if not samples:
        return CheckResult(
            req.id, req.title, req.severity, True, "empty log"
        )

    unresolved: list[str] = []
    breach_events = 0
    previously_breaching = False
    cursor = 0

    for index, sample in enumerate(samples):
        breaching = (
            sample.geofence
            and sample.state in AIRBORNE_STATES
            and not sample.inside
        )
        rising = breaching and not previously_breaching
        previously_breaching = breaching
        if not rising:
            continue

        breach_events += 1
        t_breach = sample.t_s
        window_end = t_breach + response_window_s
        if cursor < index:
            cursor = index
        resolved = False
        while cursor < len(samples) and samples[cursor].t_s <= window_end:
            later = samples[cursor]
            if later.state == "FAILSAFE":
                resolved = True
                break
            if later.inside and later.state in AIRBORNE_STATES:
                resolved = True
                break
            cursor += 1
        if not resolved:
            unresolved.append(
                f"t={t_breach:.2f}s breach without FAILSAFE/"
                f"recovery within {response_window_s:.1f}s"
            )

    if breach_events == 0:
        return CheckResult(
            req.id,
            req.title,
            req.severity,
            True,
            "no airborne geofence breaches observed",
        )

    passed = not unresolved
    return CheckResult(
        req.id,
        req.title,
        req.severity,
        passed,
        (
            f"{breach_events} breach event(s); all resolved"
            if passed
            else f"{len(unresolved)} unresolved breach event(s)"
        ),
        evidence=unresolved,
    )


def check_altitude_limit(
    trace: FlightTrace,
    fence: Fence = DEFAULT_FENCE,
    tolerance_m: float = 0.25,
) -> CheckResult:
    req = _req("REQ-ALT-01")
    violations: list[str] = []
    for sample in trace.samples:
        if not sample.geofence or sample.state == "FAILSAFE":
            continue
        if sample.altitude_m > fence.altitude_max + tolerance_m:
            violations.append(
                f"t={sample.t_s:.2f}s alt={sample.altitude_m:.2f}m "
                f"> limit {fence.altitude_max}m ({sample.state})"
            )
    passed = not violations
    return CheckResult(
        req.id,
        req.title,
        req.severity,
        passed,
        "altitude within keep-in" if passed else f"{len(violations)} over-limit sample(s)",
        evidence=violations[:8],
    )


def check_terminal_finality(trace: FlightTrace) -> CheckResult:
    req = _req("REQ-TERM-01")
    terminal_seen = False
    violations: list[str] = []
    for sample in trace.samples:
        if sample.state in TERMINAL_STATES:
            terminal_seen = True
            continue
        if terminal_seen and sample.state == "MOVE":
            violations.append(
                f"t={sample.t_s:.2f}s returned to MOVE after terminal state"
            )
    passed = not violations
    detail = (
        "no post-terminal MOVE"
        if passed
        else f"{len(violations)} illegal restart(s)"
    )
    if not terminal_seen:
        detail = "no terminal state in log (ok if truncated)"
    return CheckResult(
        req.id, req.title, req.severity, passed, detail, evidence=violations
    )


def check_executive_abort(trace: FlightTrace, response_window_s: float = 2.0) -> CheckResult:
    """O(N) rising-edge ABORT → terminal-state window check."""
    req = _req("REQ-EXEC-01")
    samples = trace.samples
    abort_events: list[tuple[float, int]] = []
    previously_abort = False
    for index, sample in enumerate(samples):
        is_abort = sample.executive_mode == "ABORT"
        if is_abort and not previously_abort:
            abort_events.append((sample.t_s, index))
        previously_abort = bool(is_abort)

    if not abort_events:
        return CheckResult(
            req.id,
            req.title,
            req.severity,
            True,
            "no executive ABORT in log",
            skipped=True,
        )

    unresolved: list[str] = []
    cursor = 0
    for t_abort, index in abort_events:
        window_end = t_abort + response_window_s
        if cursor < index:
            cursor = index
        ok = False
        while cursor < len(samples) and samples[cursor].t_s <= window_end:
            if samples[cursor].state in TERMINAL_STATES:
                ok = True
                break
            cursor += 1
        if not ok:
            unresolved.append(
                f"t={t_abort:.2f}s ABORT without FAILSAFE/LANDING "
                f"within {response_window_s:.1f}s"
            )

    passed = not unresolved
    return CheckResult(
        req.id,
        req.title,
        req.severity,
        passed,
        "ABORT followed by terminal state" if passed else "ABORT not closed out",
        evidence=unresolved,
    )


def check_sensor_backed_avoidance(trace: FlightTrace) -> CheckResult:
    req = _req("REQ-AVOID-SENSOR-01")
    if "sensor_fresh" not in trace.columns:
        return CheckResult(
            req.id, req.title, req.severity, True,
            "legacy log has no sensor-evidence columns", skipped=True,
        )
    direct = [
        s for s in trace.samples if s.obstacle in {"front", "left", "right"}
    ]
    violations: list[str] = []
    for sample in direct:
        sector_range = {
            "front": sample.lidar_front_m,
            "left": sample.lidar_left_m,
            "right": sample.lidar_right_m,
        }[sample.obstacle]
        # -1.0 is lidar_sectors.py's own sentinel for "nothing detected in
        # this sector" (a fresh, valid reading meaning the direction is
        # clear) — NOT a missing/stale reading. Treating it as a violation
        # flagged the single safest case (steering toward a confirmed-clear
        # sector) as unverified. Only a genuinely stale sample or a schema
        # gap (column absent for this sample) counts as missing evidence.
        if not sample.sensor_fresh or sector_range is None:
            violations.append(
                f"t={sample.t_s:.2f}s {sample.obstacle} without fresh sector range"
            )
    sensor_only_stale = [
        s for s in trace.samples
        if s.state == "MOVE" and s.obstacle_source == "sensor_only" and not s.sensor_fresh
    ]
    if sensor_only_stale:
        violations.append(
            f"{len(sensor_only_stale)} sensor-only MOVE sample(s) used stale LiDAR"
        )
    return CheckResult(
        req.id,
        req.title,
        req.severity,
        not violations,
        f"{len(direct)} direct avoidance sample(s) backed by LiDAR"
        if not violations else f"{len(violations)} sensor-evidence violation(s)",
        evidence=violations[:8],
    )


def check_gps_denied_zone(
    trace: FlightTrace,
    zone: GpsDeniedZone | None = None,
) -> CheckResult:
    """REQ-GPS-ZONE-01: in_gps_denied_zone flag matches default/config AABB."""
    req = _req("REQ-GPS-ZONE-01")
    if "in_gps_denied_zone" not in trace.columns:
        return CheckResult(
            req.id, req.title, req.severity, True,
            "legacy log has no GPS-denied zone column", skipped=True,
        )
    zone = zone or DEFAULT_GPS_DENIED_ZONE
    mismatches: list[str] = []
    inside_count = 0
    for sample in trace.samples:
        expected = zone.contains(sample.north, sample.east, sample.down)
        if expected:
            inside_count += 1
        if bool(sample.in_gps_denied_zone) != expected:
            mismatches.append(
                f"t={sample.t_s:.2f}s N={sample.north:.1f} E={sample.east:.1f} "
                f"flag={int(sample.in_gps_denied_zone)} expected={int(expected)}"
            )
    return CheckResult(
        req.id,
        req.title,
        req.severity,
        not mismatches,
        (
            f"{inside_count} in-zone sample(s); flag matches AABB"
            if not mismatches
            else f"{len(mismatches)} zone-flag mismatch(es)"
        ),
        evidence=mismatches[:8],
    )


def check_gps_inject_source(trace: FlightTrace) -> CheckResult:
    """REQ-GPS-INJECT-01: injected deny must not claim healthy GPS source."""
    req = _req("REQ-GPS-INJECT-01")
    if "gps_injected_deny" not in trace.columns or "loc_source" not in trace.columns:
        return CheckResult(
            req.id, req.title, req.severity, True,
            "legacy log has no GPS-inject columns", skipped=True,
        )
    violations: list[str] = []
    injected = 0
    for sample in trace.samples:
        if not sample.gps_injected_deny:
            continue
        injected += 1
        source = (sample.loc_source or "").upper()
        if source in ("", "GPS"):
            violations.append(
                f"t={sample.t_s:.2f}s inject=1 loc_source={source or '∅'}"
            )
    return CheckResult(
        req.id,
        req.title,
        req.severity,
        not violations,
        (
            f"{injected} injected-deny sample(s); sources healthy"
            if not violations
            else f"{len(violations)} inject/source violation(s)"
        ),
        evidence=violations[:8],
    )


def check_gps_policy_response(
    trace: FlightTrace,
    response_window_s: float = 2.0,
) -> CheckResult:
    """REQ-GPS-POLICY-01: LOC_FAILSAFE → FAILSAFE/LANDING within window."""
    req = _req("REQ-GPS-POLICY-01")
    if "loc_event" not in trace.columns:
        return CheckResult(
            req.id, req.title, req.severity, True,
            "legacy log has no loc_event column", skipped=True,
        )
    samples = trace.samples
    events = [
        (i, s)
        for i, s in enumerate(samples)
        if (s.loc_event or "").upper() == "LOC_FAILSAFE"
    ]
    if not events:
        return CheckResult(
            req.id, req.title, req.severity, True,
            "no LOC_FAILSAFE events in log",
        )
    unresolved: list[str] = []
    for index, sample in events:
        deadline = sample.t_s + response_window_s
        resolved = False
        for later in samples[index:]:
            if later.t_s > deadline:
                break
            if later.state in ("FAILSAFE", "LANDING"):
                resolved = True
                break
        if not resolved:
            unresolved.append(
                f"t={sample.t_s:.2f}s LOC_FAILSAFE without FAILSAFE/LANDING "
                f"within {response_window_s:.1f}s"
            )
    return CheckResult(
        req.id,
        req.title,
        req.severity,
        not unresolved,
        (
            f"{len(events)} LOC_FAILSAFE event(s) resolved"
            if not unresolved
            else f"{len(unresolved)} policy response miss(es)"
        ),
        evidence=unresolved[:8],
    )


def check_actual_gps_failure(trace: FlightTrace, settle_s: float = 1.5) -> CheckResult:
    req = _req("REQ-GPS-ACTUAL-01")
    required = {"gps_failure_active", "raw_gps_healthy"}
    if not required.issubset(trace.columns):
        return CheckResult(req.id, req.title, req.severity, True,
                           "legacy log has no PX4 GPS-failure evidence", skipped=True)
    active = [s for s in trace.samples if s.gps_failure_active]
    if not active:
        return CheckResult(req.id, req.title, req.severity, True,
                           "PX4 GPS failure was not requested", skipped=True)
    settled = [s for s in active if s.t_s >= active[0].t_s + settle_s]
    violations = [f"t={s.t_s:.2f}s raw_gps_healthy=1" for s in settled
                  if s.raw_gps_healthy]
    passed = bool(settled) and not violations
    detail = (f"GPS unavailable in {len(settled)} settled failure sample(s)"
              if passed else "GPS remained healthy or the failure interval was too short")
    return CheckResult(req.id, req.title, req.severity, passed, detail,
                       evidence=violations[:8])


def check_vio_fusion(trace: FlightTrace, settle_s: float = 1.5) -> CheckResult:
    req = _req("REQ-VIO-FUSION-01")
    required = {"gps_failure_active", "raw_gps_healthy", "vio_stream_healthy", "ev_pos_fused"}
    if not required.issubset(trace.columns):
        return CheckResult(req.id, req.title, req.severity, True,
                           "legacy log has no VIO fusion evidence", skipped=True)
    active = [s for s in trace.samples if s.gps_failure_active]
    if not active:
        return CheckResult(req.id, req.title, req.severity, True,
                           "PX4 GPS failure was not requested", skipped=True)
    denied = [s for s in active
              if s.t_s >= active[0].t_s + settle_s and not s.raw_gps_healthy]
    violations = [f"t={s.t_s:.2f}s stream={int(s.vio_stream_healthy)} fusion={int(s.ev_pos_fused)}"
                  for s in denied if not (s.vio_stream_healthy and s.ev_pos_fused)]
    passed = bool(denied) and not violations
    detail = (f"external vision fused in {len(denied)} GPS-denied sample(s)"
              if passed else "external vision was not continuously fused during GPS loss")
    return CheckResult(req.id, req.title, req.severity, passed, detail,
                       evidence=violations[:8])


def check_obstacle_clearance(trace: FlightTrace) -> CheckResult:
    req = _req("REQ-CLEARANCE-01")
    if "mapped_clearance_m" not in trace.columns:
        return CheckResult(
            req.id, req.title, req.severity, True,
            "legacy log has no mapped-clearance column", skipped=True,
        )
    measured = [
        s for s in trace.samples if s.mapped_clearance_m is not None
    ]
    collisions = [
        f"t={s.t_s:.2f}s clearance={s.mapped_clearance_m:.3f}m"
        for s in measured if s.mapped_clearance_m <= 0.0
    ]
    minimum = min((s.mapped_clearance_m for s in measured), default=None)
    return CheckResult(
        req.id,
        req.title,
        req.severity,
        not collisions,
        (
            "no clearance samples"
            if minimum is None
            else f"minimum mapped clearance {minimum:.3f}m"
        ),
        evidence=collisions[:8],
    )


def run_vv(
    trace: FlightTrace,
    fence: Fence | None = None,
    geocage_margin: float = 1.0,
    geofence_response_s: float = 1.5,
) -> VvReport:
    fence = fence or DEFAULT_FENCE
    results = [
        check_legal_state_sequence(trace),
        check_wp_monotonic(trace),
        check_geocage_setpoints(trace, fence, margin=geocage_margin),
        check_geofence_response(trace, response_window_s=geofence_response_s),
        check_altitude_limit(trace, fence),
        check_terminal_finality(trace),
        check_executive_abort(trace),
        check_sensor_backed_avoidance(trace),
        check_obstacle_clearance(trace),
        check_gps_denied_zone(trace),
        check_gps_inject_source(trace),
        check_gps_policy_response(trace),
        check_actual_gps_failure(trace),
        check_vio_fusion(trace),
    ]
    return VvReport(source=trace.source, results=results, summary=trace.summary())
