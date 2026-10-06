"""Scenario execution and evidence records."""
from __future__ import annotations

import json

import pytest

from px4_offboard.faults.evidence import run_scenario, to_markdown
from px4_offboard.faults.schema import load_scenario_text


def scenario(body: str, duration: float = 40.0):
    return load_scenario_text(f"duration_s: {duration}\nfaults:\n{body}")


def test_every_fault_yields_injected_expected_observed_and_a_verdict():
    ev = run_scenario(scenario("  - {type: vo_dropout, start_s: 10, duration_s: 5}"))
    rec = ev.records[0]
    assert rec["fault"]["type"] == "vo_dropout"
    assert rec["injected"]["dropped"] > 0                      # the fault demonstrably fired
    assert "expected_text" in rec["expected"] and rec["expected"]["criteria"]
    assert rec["observed"]["detected"] is True
    assert 0 < rec["observed"]["detection_time_s"] < 1.0
    assert rec["observed"]["recovery_time_s"] is not None
    assert rec["passed"] is True
    assert {c["name"] for c in rec["checks"]} >= {"must_detect", "recover_within_s", "max_err_growth_m"}


def test_a_failed_expectation_is_reported_as_failed_not_hidden():
    ev = run_scenario(scenario(
        "  - type: vo_dropout\n    start_s: 10\n    duration_s: 5\n    expect: {recover_within_s: -1.0}"
    ))
    rec = ev.records[0]
    assert rec["passed"] is False
    assert [c for c in rec["checks"] if not c["passed"]][0]["name"] == "recover_within_s"
    assert rec["expected"]["overrides"] == {"recover_within_s": -1.0}  # overrides are visible
    assert ev.ok is False


def test_known_gap_failures_do_not_fail_the_gate_but_are_flagged():
    ev = run_scenario(scenario(
        "  - type: vo_dropout\n    start_s: 10\n    duration_s: 5\n"
        "    known_gap: BUG-XYZ\n    expect: {recover_within_s: -1.0}"
    ))
    assert ev.records[0]["status"] == "KNOWN_GAP"
    assert ev.ok is True


def test_a_known_gap_that_now_passes_is_called_out():
    ev = run_scenario(scenario("  - {type: vo_dropout, start_s: 10, duration_s: 5, known_gap: BUG-XYZ}"))
    assert ev.records[0]["status"] == "PASS_GAP_OBSOLETE"


def test_link_faults_are_evaluated_against_the_link_rig():
    ev = run_scenario(scenario("  - {type: comm_disconnect, start_s: 10, duration_s: 5}", 30.0))
    rec = ev.records[0]
    assert rec["domain"] == "link"
    assert rec["observed"]["detected"] and rec["observed"]["recovery_time_s"] is not None
    assert rec["injected"]["reconnects"] == 1


def test_records_are_json_serialisable_and_markdown_names_every_fault():
    ev = run_scenario(scenario(
        "  - {type: vo_dropout, start_s: 5, duration_s: 3}\n"
        "  - {type: mavlink_packet_loss, start_s: 15, duration_s: 3, probability: 0.3}",
    ))
    json.dumps(ev.to_dict(), allow_nan=False)
    md = to_markdown(ev)
    assert "vo_dropout" in md and "mavlink_packet_loss" in md and "recovery" in md.lower()


def test_scenarios_are_reproducible():
    body = "  - {type: dropped_messages, start_s: 10, duration_s: 10, probability: 0.4}"
    a = run_scenario(scenario(body)).to_dict()
    b = run_scenario(scenario(body)).to_dict()
    assert a == b


def test_background_condition_is_applied_to_baseline_and_fault_runs_alike():
    # With VO gone for 20 s the IMU is all that is left, so an IMU fault shows
    # up in the error; the background outage itself must not be blamed on it.
    body = (
        "duration_s: 50\nbackground:\n  - {type: vo_dropout, start_s: 15, duration_s: 25}\n"
        "faults:\n  - {type: corrupted_measurement, sensor: imu, start_s: 20, duration_s: 10, "
        "probability: 0.5, magnitude: 5.0}\n"
    )
    with_bg = run_scenario(load_scenario_text(body)).records[0]
    alone = run_scenario(scenario(
        "  - {type: corrupted_measurement, sensor: imu, start_s: 20, duration_s: 10, probability: 0.5, magnitude: 5.0}",
        50.0,
    )).records[0]
    assert with_bg["observed"]["err_growth_m"] > alone["observed"]["err_growth_m"] + 0.05
    assert with_bg["background"] == [{"type": "vo_dropout", "start_s": 15.0, "duration_s": 25.0}]
