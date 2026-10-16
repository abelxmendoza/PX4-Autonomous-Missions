"""Records an integrated seeding sortie for the browser replay.

The flight path is sampled at a fixed rate from PX4's own estimate, tagged with the
orchestrator's state, setpoint owner and dispenser state at that moment. Everything else
(state transitions, release-gate verdicts, release samples, geofence) is taken from the
orchestrator's evidence, on the same mission clock, so the replay and the report agree.

Ground truth (the marker positions in worlds/search_field_targets.json) is not part of the
trace: the viewer loads it separately, for display and scoring only, as the search replay does.
"""

from __future__ import annotations

import math
from typing import Sequence

SCHEMA = 1
GATES = ("altitude", "weather", "airspace", "traffic", "comms")


def _yaw_deg(q: Sequence[float]) -> float:
    w, x, y, z = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


class SeedingTraceRecorder:
    """Pose samples at ``rate_hz`` on any monotonic clock; ``build`` shifts them to mission time."""

    def __init__(self, rate_hz: float = 10.0) -> None:
        self._period = 1.0 / rate_hz
        self._next: float | None = None
        self._last = -math.inf
        self.frames: list[dict] = []
        self.detections: list[dict] = []

    def sample(self, clock_s: float, position_ned: Sequence[float], q: Sequence[float],
               state: str, owner: str, dispense: bool) -> bool:
        if clock_s < self._last:
            raise ValueError(f"time went backwards: {clock_s} < {self._last}")
        self._last = clock_s
        if self._next is not None and clock_s < self._next - 1e-9:
            return False
        self._next = clock_s + self._period
        n, e, d = (round(float(v), 3) for v in position_ned)
        self.frames.append({"t": clock_s, "n": n, "e": e, "d": d, "yaw_deg": round(_yaw_deg(q), 1),
                            "state": state, "owner": owner, "dispense": bool(dispense)})
        return True

    def detection(self, clock_s: float, marker_id: int) -> None:
        self.detections.append({"t": clock_s, "id": int(marker_id)})


def _gate_verdicts(evidence: dict) -> dict:
    """Each release gate's verdict for the release decision.

    Weather, airspace, traffic and comms report their own final verdict. Altitude is a
    stream of decisions; the one that counts is the last before the vehicle left
    hold_for_release (to dispensing, or to the return when the release was denied).
    """
    out = {}
    for gate, key in (("weather", "weather_evidence"), ("airspace", "airspace_evidence"),
                      ("traffic", "traffic_evidence"), ("comms", "communications_evidence")):
        item = evidence.get(key) or {}
        out[gate] = {"authorized": bool(item.get("authorized")), "reason": item.get("reason_code") or "",
                     "consulted": bool(item.get("consulted")), "provenance": item.get("provenance", "")}
    left_hold = next((h["time_s"] for h in evidence.get("mission_state_history", [])
                      if h.get("source") == "hold_for_release"), None)
    history = evidence.get("release_authorization_history", [])
    chosen = None
    for item in history:
        if left_hold is None or item["time_s"] <= left_hold + 1e-6:
            chosen = item
    out["altitude"] = {"authorized": bool(chosen and chosen.get("inside_band")),
                       "reason": "" if chosen is None or chosen.get("inside_band") else chosen.get("reason", ""),
                       "consulted": chosen is not None,
                       "altitude_m": None if chosen is None else round(float(chosen["altitude_m"]), 2)}
    return {gate: out[gate] for gate in GATES}


def build(report: dict, recorder: SeedingTraceRecorder, mission_t0_clock_s: float, *,
          release_ned: Sequence[float], expected_result: str, camera: dict | None = None) -> dict:
    """The replay document: the recorded path and detections shifted to mission time, plus
    the report's evidence trimmed to what the viewer draws."""
    ev = report.get("orchestrator_evidence") or {}
    shift = lambda t: round(t - mission_t0_clock_s, 3)  # noqa: E731
    frames = [{**f, "t": shift(f["t"])} for f in recorder.frames]
    geofence = ev.get("geofence_configuration") or {}
    return {
        "schema": SCHEMA,
        "kind": "seeding_sortie",
        "scenario_id": report.get("scenario_id"),
        "expected_result": expected_result,
        "mission_result": report.get("mission_result"),
        "result_as_designed": report.get("mission_result") == expected_result,
        "validation": report.get("validation"),
        "world": "search_field",
        "px4_version": report.get("px4_version"),
        "simulated": True,
        "truth_used_for_guidance": False,
        "release_ned": [round(float(v), 3) for v in release_ned],
        "camera": camera,
        "geofence_polygon_ned": geofence.get("polygon_ned"),
        "recovery_polygon_ned": geofence.get("recovery_polygon_ned"),
        "outcomes": ev.get("outcomes", {}),
        "gates": _gate_verdicts(ev),
        "release": {"authorized": bool(report.get("release_authorized")),
                    "amount_kg": round(float(report.get("release_amount_kg") or 0.0), 4),
                    "samples": [{"t": round(s["time_s"], 3), "n": round(s["north_m"], 3), "e": round(s["east_m"], 3),
                                 "d": round(s["down_m"], 3)} for s in ev.get("release_samples", [])]},
        "states": [{"t": round(h["time_s"], 3), "from": h["source"], "to": h["destination"], "reason": h.get("reason", "")}
                   for h in ev.get("mission_state_history", [])],
        "owners": [{"t": round(h["time_s"], 3), "owner": h.get("owner") or h.get("destination")}
                   for h in ev.get("controller_ownership_history", [])],
        "marker_id": report.get("marker_id"),
        "detections": [{**d, "t": shift(d["t"])} for d in recorder.detections],
        "touchdown": report.get("touchdown_position"),
        "touchdown_horizontal_error_m": report.get("touchdown_horizontal_error_m"),
        "fault_codes": report.get("fault_codes", []),
        "mission_duration_s": report.get("mission_duration_s"),
        "frames": frames,
    }
