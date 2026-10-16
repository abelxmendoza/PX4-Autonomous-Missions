"""Requirement status for the mission-safety work.

UNIT_VERIFIED and INTEGRATION_VERIFIED come from pytest. SITL_VERIFIED is set
only when a campaign artifact for that requirement exists and passed. A skipped
or failed check stays unverified.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TraceItem:
    requirement_id: str
    status: str
    test_reference: str
    evidence_path: str


_VERIFIED = {"UNIT_VERIFIED", "INTEGRATION_VERIFIED", "SITL_VERIFIED"}


def summarize(items: tuple[TraceItem, ...] | list[TraceItem]) -> dict:
    ids = [item.requirement_id for item in items]
    if len(ids) != len(set(ids)):
        raise ValueError("requirement ids must be unique")
    rows = []
    for item in items:
        if item.status not in {"NOT_TESTED", "UNIT_VERIFIED", "INTEGRATION_VERIFIED", "SITL_VERIFIED", "FAILED"}:
            raise ValueError(f"unknown status {item.status}")
        counts = item.status in _VERIFIED and item.status != "FAILED"
        if item.status == "SITL_VERIFIED" and not item.evidence_path:
            counts = False
        rows.append({
            "id": item.requirement_id,
            "status": item.status,
            "test_reference": item.test_reference,
            "evidence_path": item.evidence_path,
            "counts_as_verified": counts,
            "physical_flight_validated": False,
        })
    return {"schema": 1, "kind": "mission_safety_traceability", "requirements": rows}
