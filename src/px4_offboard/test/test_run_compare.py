"""Regression comparison between flight runs, including how run-to-run variance is shown."""
from __future__ import annotations

import gzip
import json
from pathlib import Path

import pytest

from px4_offboard.flight_replay import FlightSample, write_flight_log
from px4_offboard.run_compare import compare, compute_metrics, load_run, main

ROOT = Path(__file__).resolve().parents[3]


def sample(t, state="MOVE", n=0.0, e=0.0, d=-3.0, **kw):
    base = dict(
        t_s=t, state=state, north=n, east=e, down=d, tgt_n=0.0, tgt_e=0.0, tgt_d=-3.0,
        obstacle="none", wp_index=0, geocage=False, geofence=False, inside=False, caged=False,
        roll_deg=2.0, pitch_deg=-2.0, mapped_clearance_m=5.0, vo_healthy=True, vo_n=0.0, vo_err_m=0.5,
        vo_path_m=10.0 + t, vo_drift_frac=0.05, ctrl_mode="velocity_pid",
        vel_cmd_n=1.0, vel_cmd_e=0.0, vel_cmd_d=0.0, vn=1.0, ve=0.0, vd=0.0, mapped_ok=None,
    )
    base.pop("mapped_ok")
    base.update(kw)
    return FlightSample(**base)


def make_run(tmp_path, name, *, n=40, dt=0.25, final_state="LANDING", drift=0.05, err=0.5,
             clearance=5.0, tilt=2.0, healthy=lambda i: True, gz=False):
    samples = [sample(0.0, "HOVER", mapped_clearance_m=clearance, vo_drift_frac=drift)]
    for i in range(n):
        samples.append(sample(
            1.0 + i * dt, "MOVE", n=float(i), roll_deg=tilt, mapped_clearance_m=clearance,
            vo_healthy=healthy(i), vo_err_m=err, vo_drift_frac=drift, vo_path_m=25.0 + i,
        ))
    samples.append(sample(1.0 + n * dt, final_state, n=float(n), mapped_clearance_m=clearance,
                          vo_drift_frac=drift, vo_path_m=25.0 + n))
    path = tmp_path / f"{name}.csv"
    write_flight_log(path, samples)
    if gz:
        gz_path = tmp_path / f"{name}.csv.gz"
        gz_path.write_bytes(gzip.compress(path.read_bytes()))
        return gz_path
    return path


def test_metrics_from_a_clean_run(tmp_path):
    m = compute_metrics(load_run(make_run(tmp_path, "a")))
    assert m["mission_completed"] is True
    assert m["max_tilt_deg"] == pytest.approx(2.0)
    assert m["min_clearance_m"] == pytest.approx(5.0)
    assert m["final_drift_pct"] == pytest.approx(5.0)
    assert m["vo_availability_pct"] == pytest.approx(100.0)
    assert m["vel_tracking_rms_mps"] == pytest.approx(0.0, abs=1e-9)
    assert m["log_rate_hz"] == pytest.approx(4.0, rel=0.1)
    assert m["cpu_pct_mean"] is None  # not recorded: reported as unavailable, not zero


def test_vo_outages_give_longest_outage_and_recovery_time(tmp_path):
    healthy = lambda i: not (10 <= i < 14) and not (25 <= i < 27)  # noqa: E731
    m = compute_metrics(load_run(make_run(tmp_path, "gaps", healthy=healthy)))
    assert m["vo_outage_count"] == 2
    assert m["vo_longest_outage_s"] == pytest.approx(1.0, abs=0.3)
    assert m["vo_mean_recovery_s"] == pytest.approx(0.75, abs=0.3)
    assert m["vo_availability_pct"] == pytest.approx(100 * 34 / 40, abs=1.0)


def test_gzip_recordings_load(tmp_path):
    m = compute_metrics(load_run(make_run(tmp_path, "z", gz=True)))
    assert m["mission_completed"] is True


def test_failsafe_is_not_completion(tmp_path):
    m = compute_metrics(load_run(make_run(tmp_path, "f", final_state="FAILSAFE")))
    assert m["mission_completed"] is False


def rows(result):
    return {r["metric"]: r for r in result["metrics"]}


def test_a_clear_regression_and_a_clear_improvement_are_labelled_with_the_single_run_caveat(tmp_path):
    base = [load_run(make_run(tmp_path, "b", drift=0.05, clearance=5.0))]
    cand = [load_run(make_run(tmp_path, "c", drift=0.15, clearance=7.0))]
    r = rows(compare(base, cand))
    assert r["final_drift_pct"]["verdict"] == "REGRESSION"
    assert r["min_clearance_m"]["verdict"] == "IMPROVEMENT"
    assert r["final_drift_pct"]["confidence"] == "single_run"
    assert r["max_tilt_deg"]["verdict"] == "UNCHANGED"


def test_completion_lost_is_always_a_regression(tmp_path):
    base = [load_run(make_run(tmp_path, "b"))]
    cand = [load_run(make_run(tmp_path, "c", final_state="FAILSAFE"))]
    assert rows(compare(base, cand))["mission_completed"]["verdict"] == "REGRESSION"


def test_difference_inside_the_baseline_spread_is_inconclusive_not_a_regression(tmp_path):
    base = [load_run(make_run(tmp_path, f"b{i}", drift=d)) for i, d in enumerate((0.05, 0.25))]
    cand = [load_run(make_run(tmp_path, "c", drift=0.10))]
    row = rows(compare(base, cand))["final_drift_pct"]
    assert row["verdict"] == "INCONCLUSIVE"
    assert row["baseline"]["min"] == pytest.approx(5.0) and row["baseline"]["max"] == pytest.approx(25.0)
    assert "spread" in row["note"]


def test_difference_beyond_the_spread_with_repeated_runs_is_a_real_regression(tmp_path):
    base = [load_run(make_run(tmp_path, f"b{i}", drift=d)) for i, d in enumerate((0.05, 0.06))]
    cand = [load_run(make_run(tmp_path, f"c{i}", drift=d)) for i, d in enumerate((0.15, 0.17))]
    row = rows(compare(base, cand))["final_drift_pct"]
    assert row["verdict"] == "REGRESSION" and row["confidence"] == "repeated_runs"


def test_missing_metrics_are_reported_as_unavailable(tmp_path):
    r = rows(compare([load_run(make_run(tmp_path, "b"))], [load_run(make_run(tmp_path, "c"))]))
    assert r["cpu_pct_mean"]["verdict"] == "N/A"


def test_cli_prints_a_table_writes_json_and_can_fail_on_regression(tmp_path, capsys):
    b = make_run(tmp_path, "b", drift=0.05)
    c = make_run(tmp_path, "c", drift=0.15)
    out = tmp_path / "cmp.json"
    assert main([str(b), str(c), "--json", str(out)]) == 0
    text = capsys.readouterr().out
    assert "final_drift_pct" in text and "REGRESSION" in text and "single run" in text.lower()
    assert json.loads(out.read_text())["summary"]["regressions"] >= 1
    assert main([str(b), str(c), "--fail-on-regression"]) == 1
    assert main([str(b), str(b), "--fail-on-regression"]) == 0


def test_the_two_curated_live_flights_show_their_run_to_run_difference_honestly():
    k = ROOT / "evidence/stereo_vo/flight_K.csv.gz"
    l = ROOT / "evidence/stereo_vo/flight_L.csv.gz"
    result = compare([load_run(k)], [load_run(l)])
    row = rows(result)["final_drift_pct"]
    # K and L are the SAME configuration: whatever this reports is variance, and the
    # tool must say it could not tell (single run each) rather than blame a change.
    assert row["confidence"] == "single_run"
    assert result["notes"] and "single" in " ".join(result["notes"]).lower()


def test_drift_and_availability_agree_with_the_verifier_on_a_curated_flight():
    # compare_runs must not invent its own definitions: same numbers as vv_harness.
    from px4_offboard.vv_harness import check_vo_availability, check_vo_drift

    run = load_run(ROOT / "evidence/stereo_vo/flight_K.csv.gz")
    m = compute_metrics(run)
    assert f"final drift {m['final_drift_pct']:.1f}%" in check_vo_drift(run.trace).detail
    assert f"peak {m['peak_drift_pct']:.1f}%" in check_vo_drift(run.trace).detail
    healthy, total = [int(x) for x in check_vo_availability(run.trace).detail.split(" in ")[1].split(" MOVE")[0].split("/")]
    assert m["vo_availability_pct"] == pytest.approx(100.0 * healthy / total)
