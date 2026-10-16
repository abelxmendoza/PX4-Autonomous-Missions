"""Assertion evaluators on hand-built sample sequences (pure functions; no vehicle)."""
from __future__ import annotations

import math

import pytest

from px4_offboard.testspec.assertions import FAIL, NOT_EVALUATED, PASS, evaluate
from px4_offboard.testspec.schema import ASSERTIONS, AssertionSpec, _params
from px4_offboard.testspec.telemetry import Sample


def _spec(atype: str, **params) -> AssertionSpec:
    if "polygon_ned" in params:
        params["polygon_ned"] = [list(p) for p in params["polygon_ned"]]
    return AssertionSpec(f"a_{atype}", atype, _params(params, ASSERTIONS[atype], "test"), ("REQ-GEOFENCE-007",))


def _sample(i, n=0.0, e=0.0, alt=0.0, vn=0.0, ve=0.0, vd=0.0, armed=True) -> Sample:
    return Sample(i, 0.05 * i, 0, n, e, -alt, vn, ve, vd, armed, None, None, None)


SQUARE = [(-5, -5), (15, -5), (15, 13), (-5, 13)]


def _fence(**kw):
    return _spec("geofence", polygon_ned=SQUARE, min_altitude_m=-0.5, max_altitude_m=10.0, **kw)


def test_no_telemetry_is_not_evaluated_never_pass():
    for atype, spec in [("geofence", _fence()), ("final_disarmed", _spec("final_disarmed"))]:
        r = evaluate(spec, [])
        assert r.status == NOT_EVALUATED, atype
        assert r.samples is None


def test_geofence_passes_inside_and_reports_closest_approach():
    samples = [_sample(0), _sample(1, n=10, e=0, alt=5), _sample(2, n=14, e=0, alt=5)]
    r = evaluate(_fence(), samples)
    assert r.status == PASS
    assert r.observed["min_boundary_distance_m"] == pytest.approx(1.0)
    assert r.worst_sample == 2
    assert r.samples == (0, 2)


@pytest.mark.parametrize(
    "bad, reason",
    [
        (dict(n=16, e=0, alt=5), "outside_horizontal"),
        (dict(n=0, e=0, alt=11), "above_max_altitude"),
        (dict(n=0, e=0, alt=-1), "below_min_altitude"),
    ],
)
def test_geofence_fails_on_the_first_violating_sample(bad, reason):
    samples = [_sample(0), _sample(1, **bad), _sample(2, **bad)]
    r = evaluate(_fence(), samples)
    assert r.status == FAIL
    assert r.observed["first_violation"] == {"sample": 1, "status": "outside", "reason": reason}
    assert r.worst_sample == 1


def test_max_altitude_uses_the_highest_sample():
    samples = [_sample(0, alt=1), _sample(1, alt=6.01), _sample(2, alt=2)]
    assert evaluate(_spec("max_altitude", limit_m=6.0), samples).status == FAIL
    r = evaluate(_spec("max_altitude", limit_m=6.1), samples)
    assert r.status == PASS and r.worst_sample == 1 and r.observed["max_altitude_m"] == 6.01


def test_max_speed_checks_horizontal_and_vertical_separately():
    samples = [_sample(0, vn=1.5, ve=2.0), _sample(1, vd=-1.3)]
    r = evaluate(_spec("max_speed", horizontal_mps=2.5, vertical_mps=1.2), samples)
    assert r.status == FAIL
    assert r.observed == {"max_horizontal_mps": 2.5, "max_vertical_mps": 1.3}
    assert r.worst_sample == 1
    assert evaluate(_spec("max_speed", horizontal_mps=2.5), samples).status == PASS


def test_final_position_uses_only_the_last_sample():
    samples = [_sample(0, n=50), _sample(1, n=0.1, e=0.1, alt=0.1)]
    r = evaluate(_spec("final_position", north_m=0, east_m=0, altitude_m=0, tolerance_m=0.2), samples)
    assert r.status == PASS
    assert r.observed["error_m"] == pytest.approx(math.sqrt(0.03))
    assert evaluate(_spec("final_position", north_m=0, east_m=0, altitude_m=0, tolerance_m=0.1), samples).status == FAIL


def test_final_disarmed():
    assert evaluate(_spec("final_disarmed"), [_sample(0, armed=False)]).status == PASS
    assert evaluate(_spec("final_disarmed"), [_sample(0, armed=False), _sample(1, armed=True)]).status == FAIL


def test_result_serialises_with_a_telemetry_reference():
    r = evaluate(_spec("final_disarmed"), [_sample(0, armed=False)])
    d = r.as_dict("telemetry.csv")
    assert d["telemetry_ref"] == {"file": "telemetry.csv", "sample_range": [0, 0], "deciding_sample": 0}
    assert d["requirements"] == ["REQ-GEOFENCE-007"]


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_sample_cannot_be_hidden_by_a_valid_extremum(bad):
    samples = [_sample(0, alt=1), _sample(1, alt=bad), _sample(2, alt=2)]
    r = evaluate(_spec("max_altitude", limit_m=6), samples)
    assert r.status == NOT_EVALUATED
    assert "sample 1" in r.detail
