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
    req = _req("REQ-GEOFENCE-01")
    breaches: list[tuple[float, int]] = []
    for index, sample in enumerate(trace.samples):
        if (
            sample.geofence
            and sample.state in AIRBORNE_STATES
            and not sample.inside
        ):
            breaches.append((sample.t_s, index))

    if not breaches:
        return CheckResult(
            req.id,
            req.title,
            req.severity,
            True,
            "no airborne geofence breaches observed",
        )

    unresolved: list[str] = []
    for t_breach, index in breaches:
        window_end = t_breach + response_window_s
        resolved = False
        for later in trace.samples[index:]:
            if later.t_s > window_end:
                break
            if later.state == "FAILSAFE":
                resolved = True
                break
            # Recovered inside before timeout also acceptable
            if later.inside and later.state in AIRBORNE_STATES:
                resolved = True
                break
        if not resolved:
            unresolved.append(
                f"t={t_breach:.2f}s breach without FAILSAFE/"
                f"recovery within {response_window_s:.1f}s"
            )

    # Deduplicate burst evidence
    unique = list(dict.fromkeys(unresolved))
    passed = not unique
    return CheckResult(
        req.id,
        req.title,
        req.severity,
        passed,
        (
            f"{len(breaches)} breach sample(s); all resolved"
            if passed
            else f"{len(unique)} unresolved breach event(s)"
        ),
        evidence=unique,
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
    req = _req("REQ-EXEC-01")
    abort_times = [
        s.t_s for s in trace.samples if s.executive_mode == "ABORT"
    ]
    if not abort_times:
        return CheckResult(
            req.id,
            req.title,
            req.severity,
            True,
            "no executive ABORT in log",
            skipped=True,
        )

    unresolved: list[str] = []
    for t_abort in abort_times:
        ok = any(
            s.state in TERMINAL_STATES
            and t_abort <= s.t_s <= t_abort + response_window_s
            for s in trace.samples
        )
        if not ok:
            unresolved.append(
                f"t={t_abort:.2f}s ABORT without FAILSAFE/LANDING "
                f"within {response_window_s:.1f}s"
            )
    unique = list(dict.fromkeys(unresolved))
    passed = not unique
    return CheckResult(
        req.id,
        req.title,
        req.severity,
        passed,
        "ABORT followed by terminal state" if passed else "ABORT not closed out",
        evidence=unique,
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
    ]
    return VvReport(source=trace.source, results=results, summary=trace.summary())
