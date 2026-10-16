"""Campaign statistics from mission reports that were actually written.

A missing report is an incomplete run. It stays in the denominator of every
rate that includes its scenario. This module does not invent flights.
"""

from __future__ import annotations

import math
from typing import Any


_FAULT_SCENARIOS = {
    "weather_during_dispense",
    "airspace_during_dispense",
    "geofence_boundary",
    "marker_loss",
    "altitude_denial",
    "weather_denial",
    "airspace_denial",
}


def _numbers(values: list[float]) -> dict[str, Any]:
    if not values:
        return {"sample_count": 0}
    ordered = sorted(values)
    n = len(ordered)
    mean = sum(ordered) / n
    mid = n // 2
    median = ordered[mid] if n % 2 else (ordered[mid - 1] + ordered[mid]) / 2.0
    variance = sum((item - mean) ** 2 for item in ordered) / n
    result: dict[str, Any] = {
        "sample_count": n,
        "mean": mean,
        "median": median,
        "minimum": ordered[0],
        "maximum": ordered[-1],
        "standard_deviation": math.sqrt(variance),
    }
    if n >= 5:
        rank = (n - 1) * 0.95
        lo = math.floor(rank)
        hi = math.ceil(rank)
        if lo == hi:
            result["p95"] = ordered[lo]
        else:
            result["p95"] = ordered[lo] + (ordered[hi] - ordered[lo]) * (rank - lo)
    else:
        result["p95"] = None
        result["p95_note"] = "not computed; fewer than 5 samples"
    return result


def _rejected(report: dict) -> int:
    evidence = report.get("orchestrator_evidence") or {}
    return len(evidence.get("rejected_setpoints") or [])


def _outside(report: dict) -> int:
    evidence = report.get("orchestrator_evidence") or {}
    return sum(1 for item in evidence.get("geofence_decisions") or [] if item.get("status") == "outside")


def _latencies(report: dict) -> list[float]:
    evidence = report.get("orchestrator_evidence") or {}
    values = []
    for item in evidence.get("fault_response_events") or []:
        latency = item.get("latency_s")
        if isinstance(latency, (int, float)) and math.isfinite(latency):
            values.append(float(latency))
    return values


def expectation_met(report: dict) -> bool:
    """True when this written report shows the designed response for its scenario."""
    scenario = report.get("scenario_id")
    result = report.get("mission_result")
    samples = report.get("release_sample_count") or 0
    if scenario == "nominal":
        return result == "completed"
    if scenario in ("weather_during_dispense", "airspace_during_dispense"):
        reason = ((report.get("orchestrator_evidence") or {}).get("mission") or {}).get("reason")
        return samples > 0 and result != "completed" and reason == "provider_failure"
    if scenario == "geofence_boundary":
        return result != "completed" and _rejected(report) > 0
    if scenario == "marker_loss":
        codes = report.get("fault_codes") or []
        return any(code in ("marker_lost", "marker_lost_during_descent", "marker_stale", "marker_never_acquired") for code in codes)
    if scenario in ("altitude_denial", "weather_denial", "airspace_denial", "traffic_conflict", "clearance_expired"):
        return result == "aborted_recovered"
    return False


def _class(report: dict) -> str:
    if report.get("failure") or report.get("mission_result") == "in_progress":
        return "infrastructure_failure"
    if report.get("scenario_id") == "geofence_boundary" and report.get("mission_result") == "simulation_terminated":
        return "controller_outcome"
    if report.get("mission_result") == "simulation_terminated":
        return "incomplete_run"
    return "controller_outcome"


def summarize_campaign(reports: list[dict], planned: dict[str, int] | None = None) -> dict:
    """Summarize reports that exist. ``planned`` counts runs that were not started."""
    planned = planned or {}
    by_scenario: dict[str, list[dict]] = {}
    for report in reports:
        by_scenario.setdefault(str(report.get("scenario_id")), []).append(report)

    guided = [
        item for item in reports
        if item.get("mission_result") in ("completed", "aborted_recovered")
        and isinstance(item.get("touchdown_horizontal_error_m"), (int, float))
    ]
    errors = [float(item["touchdown_horizontal_error_m"]) for item in guided]
    marker_loss = by_scenario.get("marker_loss", [])
    nominal = by_scenario.get("nominal", [])
    nominal_completed = sum(1 for item in nominal if item.get("mission_result") == "completed")
    not_started = {}
    for key, count in planned.items():
        missing = count - len(by_scenario.get(key, []))
        if missing > 0:
            not_started[key] = missing
    fault_reports = [item for item in reports if item.get("scenario_id") in _FAULT_SCENARIOS]
    detected = sum(1 for item in fault_reports if expectation_met(item))
    latencies = [value for item in reports for value in _latencies(item)]

    return {
        "schema": 1,
        "kind": "mission_verification_summary",
        "simulated": True,
        "physical_flight_validated": False,
        "population_reliability": False,
        "sample_note": "Rates use the runs in this campaign only. They are not a population reliability claim.",
        "run_count": len(reports),
        "planned_not_started": not_started,
        "nominal_completion_rate": None if not nominal else nominal_completed / len(nominal),
        "nominal_run_count": len(nominal),
        "nominal_completed": nominal_completed,
        "release_authorization_correct": sum(
            1 for item in nominal if item.get("mission_result") == "completed" and item.get("release_sample_count", 0) > 0
        ),
        "fault_detection_rate": None if not fault_reports else detected / len(fault_reports),
        "fault_run_count": len(fault_reports),
        "fault_responses_meeting_expectation": detected,
        "fault_response_latency_s": _numbers(latencies),
        "geofence_outside_decision_count": sum(_outside(item) for item in reports),
        "unsafe_setpoint_rejection_count": sum(_rejected(item) for item in reports),
        "precision_landing_success_rate": None if not nominal else nominal_completed / len(nominal),
        "touchdown_horizontal_error_m": _numbers(errors),
        "touchdown_note": "Horizontal error includes completed and aborted_recovered landings. Marker-loss positions are listed separately and are not in this distribution.",
        "marker_loss_outcomes": [
            {
                "mission_result": item.get("mission_result"),
                "touchdown_horizontal_error_m": item.get("touchdown_horizontal_error_m"),
            }
            for item in marker_loss
        ],
        "class_counts": {
            name: sum(1 for item in reports if _class(item) == name)
            for name in ("controller_outcome", "infrastructure_failure", "incomplete_run")
        },
        "runs": [
            {
                "scenario_id": item.get("scenario_id"),
                "simulation_seed": item.get("simulation_seed", 0),
                "px4_version": item.get("px4_version"),
                "mission_result": item.get("mission_result"),
                "expectation_met": expectation_met(item),
                "class": _class(item),
                "release_sample_count": item.get("release_sample_count"),
                "touchdown_horizontal_error_m": item.get("touchdown_horizontal_error_m"),
                "rejected_setpoints": _rejected(item),
                "fault_codes": item.get("fault_codes"),
            }
            for item in reports
        ],
    }
