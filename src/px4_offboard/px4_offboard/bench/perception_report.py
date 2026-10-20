"""Offline verification adapter for recorded depth-perception reports.

The depth policy remains owned by Jetson-RealSense-Perception. This module
checks the portable v1 report and derives verification results; it has no PX4,
MAVLink, ROS, simulator, or vehicle-command imports.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Any, Mapping

REPORT_TYPE = "bench_replay_report"
SCHEMA_VERSION = 1
STATUSES = {"CLEAR", "STOP", "SENSOR_FAULT"}
OUTCOMES = {"PASS", "FAIL", "SKIP"}


class PerceptionReportError(ValueError):
    """The report is not a valid v1 perception replay artifact."""


@dataclass(frozen=True)
class PerceptionRequirementResult:
    req_id: str
    description: str
    passed: bool
    detail: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class PerceptionVerification:
    recording_name: str
    source: str
    event_count: int
    status_counts: dict[str, int]
    outcome_counts: dict[str, int]
    requirements: tuple[PerceptionRequirementResult, ...]

    @property
    def passed(self) -> bool:
        return all(result.passed for result in self.requirements)

    def to_dict(self) -> dict[str, Any]:
        return {
            "verification_type": "perception_replay_verification",
            "schema_version": 1,
            "recording_name": self.recording_name,
            "source": self.source,
            "event_count": self.event_count,
            "status_counts": self.status_counts,
            "outcome_counts": self.outcome_counts,
            "requirements": [result.to_dict() for result in self.requirements],
            "passed": self.passed,
        }


def _object(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise PerceptionReportError(f"{label} must be an object")
    return value


def _declared_counts(value: Any, keys: set[str], label: str) -> dict[str, int]:
    counts = _object(value, label)
    if set(counts) != keys:
        raise PerceptionReportError(f"{label} must contain exactly {sorted(keys)}")
    if any(not isinstance(count, int) or isinstance(count, bool) or count < 0 for count in counts.values()):
        raise PerceptionReportError(f"{label} values must be non-negative integers")
    return dict(counts)


def verify_perception_report(report: Any, *, allow_skip: bool = False) -> PerceptionVerification:
    root = _object(report, "report")
    if root.get("report_type") != REPORT_TYPE:
        raise PerceptionReportError(f"report_type must be {REPORT_TYPE!r}")
    if type(root.get("schema_version")) is not int or root.get("schema_version") != SCHEMA_VERSION:
        raise PerceptionReportError(f"unsupported schema_version {root.get('schema_version')!r}")
    recording = _object(root.get("recording"), "recording")
    name, source = recording.get("name"), recording.get("source")
    if not isinstance(name, str) or not name:
        raise PerceptionReportError("recording.name must be a non-empty string")
    if not isinstance(source, str) or source not in {"synthetic", "hardware", "unknown"}:
        raise PerceptionReportError("recording.source must be synthetic, hardware, or unknown")
    events = root.get("events")
    if not isinstance(events, list) or not events:
        raise PerceptionReportError("events must be a non-empty array")
    observed_statuses = {key: 0 for key in STATUSES}
    observed_outcomes = {key: 0 for key in OUTCOMES}
    previous_time = -math.inf
    for index, raw_event in enumerate(events):
        event = _object(raw_event, f"events[{index}]")
        status, outcome, timestamp = event.get("status"), event.get("outcome"), event.get("timestamp_s")
        if not isinstance(status, str) or status not in STATUSES:
            raise PerceptionReportError(f"events[{index}].status is invalid")
        if not isinstance(outcome, str) or outcome not in OUTCOMES:
            raise PerceptionReportError(f"events[{index}].outcome is invalid")
        if (not isinstance(timestamp, (int, float)) or isinstance(timestamp, bool)
                or not math.isfinite(timestamp) or timestamp < previous_time):
            raise PerceptionReportError("event timestamps must be finite and non-decreasing")
        previous_time = float(timestamp)
        observed_statuses[status] += 1
        observed_outcomes[outcome] += 1
    summary = _object(root.get("summary"), "summary")
    if type(summary.get("total")) is not int or summary.get("total") != len(events):
        raise PerceptionReportError("summary.total does not match events")
    if _declared_counts(summary.get("statuses"), STATUSES, "summary.statuses") != observed_statuses:
        raise PerceptionReportError("summary.statuses does not match events")
    if _declared_counts(summary.get("outcomes"), OUTCOMES, "summary.outcomes") != observed_outcomes:
        raise PerceptionReportError("summary.outcomes does not match events")
    failed, skipped = observed_outcomes["FAIL"], observed_outcomes["SKIP"]
    requirements = (
        PerceptionRequirementResult("REQ-PERCEPTION-001", "Expected depth-policy states match replay results", failed == 0, f"{failed} FAIL event(s)"),
        PerceptionRequirementResult("REQ-PERCEPTION-002", "Every event has an expected state", allow_skip or skipped == 0, f"{skipped} SKIP event(s)" + (" allowed" if allow_skip else "")),
        PerceptionRequirementResult("REQ-PERCEPTION-003", "Sensor faults remain visible", True, f"{observed_statuses['SENSOR_FAULT']} SENSOR_FAULT event(s) preserved"),
    )
    return PerceptionVerification(name, source, len(events), observed_statuses, observed_outcomes, requirements)


def verify_perception_report_file(path: str | Path, *, allow_skip: bool = False) -> PerceptionVerification:
    try:
        report = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise PerceptionReportError(f"could not read report: {exc}") from exc
    return verify_perception_report(report, allow_skip=allow_skip)
