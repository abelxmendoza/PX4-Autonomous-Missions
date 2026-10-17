"""Bench test-campaign requirement evaluation and the golden scripted run."""
from __future__ import annotations

from dataclasses import replace

from px4_offboard.bench.campaign import (
    CampaignSpec,
    Setpoint,
    evaluate_requirements,
    golden_spec,
    report_dict,
    run_campaign,
)
from px4_offboard.bench.virtual_rig import FaultWindow, RigConfig, RigSample


def test_golden_campaign_passes_every_requirement():
    samples, results = run_campaign(golden_spec())
    assert len(samples) == int(golden_spec().duration_s / golden_spec().config.dt_s)
    by_id = {r.req_id for r in results}
    assert {"REQ-BENCH-001", "REQ-BENCH-002", "REQ-BENCH-003", "REQ-BENCH-004"} <= by_id
    assert all(r.passed for r in results), [r for r in results if not r.passed]
    report = report_dict(golden_spec(), samples, results)
    assert report["passed"] is True
    assert report["sample_count"] == len(samples)


def test_req_bench_002_fails_when_a_declared_dropout_has_no_visible_effect():
    """Deliberately re-breaks the detector's input: a fault is declared but
    never actually shows up in the readings, as it would if VirtualTestRig
    silently dropped fault handling. The requirement must fail, not trust
    the declaration."""
    spec = replace(golden_spec(), faults=[FaultWindow("load_cell_dropout", start_s=1.0, duration_s=1.0)])
    samples = [
        RigSample(t=round(i * 0.05, 6), command_pct=50.0, rpm=4000.0, thrust_n=5.0,
                  load_cell_n=5.0, voltage_v=16.0, temp_c=30.0, fault_active="")
        for i in range(40)
    ]
    req002 = next(r for r in evaluate_requirements(samples, spec) if r.req_id == "REQ-BENCH-002")
    assert req002.passed is False


def test_req_bench_003_catches_an_uninjected_voltage_drop():
    spec = golden_spec()
    samples = [
        RigSample(t=round(i * 0.05, 6), command_pct=100.0, rpm=9000.0, thrust_n=30.0,
                  load_cell_n=30.0, voltage_v=5.0, temp_c=40.0, fault_active="")
        for i in range(10)
    ]
    req003 = next(r for r in evaluate_requirements(samples, spec) if r.req_id == "REQ-BENCH-003")
    assert req003.passed is False


def test_req_bench_003_does_not_flag_a_declared_brownout():
    spec = golden_spec()
    samples = [
        RigSample(t=round(i * 0.05, 6), command_pct=100.0, rpm=9000.0, thrust_n=30.0,
                  load_cell_n=30.0, voltage_v=5.0, temp_c=40.0, fault_active="voltage_brownout")
        for i in range(10)
    ]
    req003 = next(r for r in evaluate_requirements(samples, spec) if r.req_id == "REQ-BENCH-003")
    assert req003.passed is True


def test_req_bench_004_catches_a_dropped_sample():
    spec = golden_spec()
    samples = [
        RigSample(t=0.0, command_pct=0, rpm=0, thrust_n=0, load_cell_n=0, voltage_v=16.0, temp_c=25.0, fault_active=""),
        RigSample(t=1.0, command_pct=0, rpm=0, thrust_n=0, load_cell_n=0, voltage_v=16.0, temp_c=25.0, fault_active=""),
    ]
    req004 = next(r for r in evaluate_requirements(samples, spec) if r.req_id == "REQ-BENCH-004")
    assert req004.passed is False


def test_command_at_holds_the_last_setpoint_at_or_before_each_time():
    spec = CampaignSpec(
        name="t", duration_s=1.0,
        setpoints=[Setpoint(0.0, 0.0), Setpoint(0.5, 80.0)],
        config=RigConfig(dt_s=0.1, seed=1),
    )
    samples, _ = run_campaign(spec)
    assert samples[0].command_pct == 0.0
    assert samples[-1].command_pct == 80.0


def _flat_samples(temps, fault=""):
    return [RigSample(t=round(i * 0.05, 6), command_pct=50.0, rpm=4000.0, thrust_n=5.0, load_cell_n=5.0,
                      voltage_v=16.0, temp_c=v, fault_active=fault) for i, v in enumerate(temps)]


def test_req_bench_005_catches_the_golden_frozen_sensor_within_a_second():
    samples, results = run_campaign(golden_spec())
    req = next(r for r in results if r.req_id == "REQ-BENCH-005")
    assert req.passed, req.detail
    from px4_offboard.bench.campaign import detect_stuck_sensor
    detections = detect_stuck_sensor(samples, "temp_c")
    assert len(detections) == 1
    frozen = next(f for f in golden_spec().faults if f.name == "frozen_temp_sensor")
    assert frozen.start_s <= detections[0] <= frozen.start_s + 1.0


def test_req_bench_005_fails_when_a_declared_freeze_never_shows_up():
    """The fault is declared but the reading keeps moving: nothing was actually frozen,
    so the requirement must fail instead of trusting the declaration."""
    spec = replace(golden_spec(), faults=[FaultWindow("frozen_temp_sensor", start_s=0.5, duration_s=1.0)])
    samples = _flat_samples([30.0 + 0.01 * (i % 7) for i in range(60)])
    req = next(r for r in evaluate_requirements(samples, spec) if r.req_id == "REQ-BENCH-005")
    assert req.passed is False


def test_req_bench_005_fails_on_a_false_alarm_from_a_healthy_sensor():
    """A reading that repeats exactly with no fault injected is flagged: the check must
    report it rather than pass because no freeze was planned."""
    spec = replace(golden_spec(), faults=[])
    samples = _flat_samples([30.0] * 60)
    req = next(r for r in evaluate_requirements(samples, spec) if r.req_id == "REQ-BENCH-005")
    assert req.passed is False


def test_the_report_lists_when_the_stuck_sensor_was_flagged():
    samples, results = run_campaign(golden_spec())
    report = report_dict(golden_spec(), samples, results)
    assert report["events"] == [{"t": 11.5, "event": "stuck_sensor_flagged", "channel": "temp_c"}]
