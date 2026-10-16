"""Assertions over recorded telemetry. Each returns PASS, FAIL or NOT_EVALUATED
with the threshold, the observed value and the sample indices it came from.
No telemetry means NOT_EVALUATED, never PASS."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Sequence

from px4_offboard.altitude_reference import AltitudeFrame
from px4_offboard.mission_geofence import APPROACHING, INSIDE, evaluate_vehicle

from .schema import AssertionSpec, geofence_config
from .telemetry import Sample

PASS = "PASS"
FAIL = "FAIL"
NOT_EVALUATED = "NOT_EVALUATED"


@dataclass
class AssertionResult:
    id: str
    type: str
    requirements: tuple[str, ...]
    status: str
    expected: Any = None
    observed: Any = None
    detail: str = ""
    samples: tuple[int, int] | None = None  # telemetry index range examined
    worst_sample: int | None = None  # index of the deciding sample
    extra: dict[str, Any] = field(default_factory=dict)

    def as_dict(self, telemetry_file: str) -> dict[str, Any]:
        return {
            "id": self.id,
            "type": self.type,
            "requirements": list(self.requirements),
            "status": self.status,
            "expected": self.expected,
            "observed": self.observed,
            "detail": self.detail,
            "telemetry_ref": {
                "file": telemetry_file,
                "sample_range": list(self.samples) if self.samples else None,
                "deciding_sample": self.worst_sample,
            },
            **self.extra,
        }


def _geofence(spec: AssertionSpec, samples: Sequence[Sample]) -> dict:
    config = geofence_config(spec.params, f"test-spec assertion {spec.id}")
    counts: dict[str, int] = {}
    first_violation = None
    closest = None  # (boundary distance, index)
    for s in samples:
        d = evaluate_vehicle(
            config,
            north_m=s.north_m,
            east_m=s.east_m,
            altitude_m=s.altitude_m,
            altitude_valid=True,
            altitude_frame=AltitudeFrame.LOCAL,
            position_uncertainty_m=0.0,  # the mock's state is exact simulator truth
            velocity_ned=(s.vn_mps, s.ve_mps, s.vd_mps),
            velocity_uncertainty_mps=0.0,
            time_s=s.t_s,
            mission_state=f"step_{s.step}",
        )
        counts[d.status] = counts.get(d.status, 0) + 1
        if d.status not in (INSIDE, APPROACHING) and first_violation is None:
            first_violation = (s.index, d.status, d.reason_code)
        if d.boundary_distance_m is not None and (closest is None or d.boundary_distance_m < closest[0]):
            closest = (d.boundary_distance_m, s.index)
    observed = {
        "status_counts": dict(sorted(counts.items())),
        "min_boundary_distance_m": closest[0] if closest else None,
    }
    if first_violation:
        idx, status, reason = first_violation
        observed["first_violation"] = {"sample": idx, "status": status, "reason": reason}
        return {"passed": False, "observed": observed, "worst": idx,
                "detail": f"sample {idx} classified {status} ({reason}) by mission_geofence.evaluate_vehicle"}
    return {"passed": True, "observed": observed, "worst": closest[1] if closest else None,
            "detail": f"all {len(samples)} samples inside or approaching; classifier mission_geofence.evaluate_vehicle"}


def _argmax(samples: Sequence[Sample], key: Callable[[Sample], float]) -> Sample:
    return max(samples, key=key)


def _max_altitude(spec: AssertionSpec, samples: Sequence[Sample]) -> dict:
    top = _argmax(samples, lambda s: s.altitude_m)
    return {"passed": top.altitude_m <= spec.params["limit_m"], "observed": {"max_altitude_m": top.altitude_m},
            "worst": top.index, "detail": f"highest sample {top.index} at t={top.t_s:.2f} s"}


def _max_speed(spec: AssertionSpec, samples: Sequence[Sample]) -> dict:
    observed, passed, worst, notes = {}, True, None, []
    if "horizontal_mps" in spec.params:
        s = _argmax(samples, lambda s: math.hypot(s.vn_mps, s.ve_mps))
        v = math.hypot(s.vn_mps, s.ve_mps)
        observed["max_horizontal_mps"] = v
        ok = v <= spec.params["horizontal_mps"]
        passed &= ok
        worst = s.index if (worst is None or not ok) else worst
        notes.append(f"horizontal peak at sample {s.index}")
    if "vertical_mps" in spec.params:
        s = _argmax(samples, lambda s: abs(s.vd_mps))
        observed["max_vertical_mps"] = abs(s.vd_mps)
        ok = abs(s.vd_mps) <= spec.params["vertical_mps"]
        passed &= ok
        worst = s.index if (worst is None or not ok) else worst
        notes.append(f"vertical peak at sample {s.index}")
    return {"passed": passed, "observed": observed, "worst": worst, "detail": "; ".join(notes)}


def _final_position(spec: AssertionSpec, samples: Sequence[Sample]) -> dict:
    p, s = spec.params, samples[-1]
    error = math.dist((s.north_m, s.east_m, s.altitude_m), (p["north_m"], p["east_m"], p["altitude_m"]))
    return {"passed": error <= p["tolerance_m"],
            "observed": {"north_m": s.north_m, "east_m": s.east_m, "altitude_m": s.altitude_m, "error_m": error},
            "worst": s.index, "detail": f"last sample {s.index}"}


def _final_disarmed(spec: AssertionSpec, samples: Sequence[Sample]) -> dict:
    s = samples[-1]
    return {"passed": not s.armed, "observed": {"armed": s.armed}, "worst": s.index, "detail": f"last sample {s.index}"}


EVALUATORS: dict[str, Callable[[AssertionSpec, Sequence[Sample]], dict]] = {
    "geofence": _geofence,
    "max_altitude": _max_altitude,
    "max_speed": _max_speed,
    "final_position": _final_position,
    "final_disarmed": _final_disarmed,
}


def evaluate(spec: AssertionSpec, samples: Sequence[Sample]) -> AssertionResult:
    result = AssertionResult(spec.id, spec.type, spec.requirements, NOT_EVALUATED, expected=dict(spec.params),
                             extra={"note": spec.note} if spec.note else {})
    if spec.type == "geofence":
        result.expected = {**spec.params, "polygon_ned": [list(p) for p in spec.params["polygon_ned"]]}
    if not samples:
        result.detail = "no telemetry samples to evaluate"
        return result
    out = EVALUATORS[spec.type](spec, samples)
    result.status = PASS if out["passed"] else FAIL
    result.observed = out["observed"]
    result.worst_sample = out["worst"]
    result.detail = out["detail"]
    result.samples = (samples[0].index, samples[-1].index)
    return result
