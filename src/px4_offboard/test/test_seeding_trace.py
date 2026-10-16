"""The seeding replay document: sampling, mission-clock shift, and gate verdicts."""
import gzip
import json
from pathlib import Path

import pytest

from px4_offboard.seeding_trace import GATES, SeedingTraceRecorder, _gate_verdicts, build

ROOT = Path(__file__).resolve().parents[3]
CAMPAIGN = ROOT / "evidence" / "fault_scenarios" / "20261013T180619Z"


def _report(name):
    with gzip.open(CAMPAIGN / name / "sitl_report.json.gz", "rt") as handle:
        return json.load(handle)


def test_samples_at_the_rate_and_refuses_time_going_backwards():
    rec = SeedingTraceRecorder(rate_hz=10.0)
    taken = [rec.sample(100.0 + k * 0.02, (1.0, 2.0, -3.0), (1.0, 0.0, 0.0, 0.0), "dispensing", "seeding", True)
             for k in range(100)]   # 2 s at 50 Hz
    assert sum(taken) == 20
    assert rec.frames[0] == {"t": 100.0, "n": 1.0, "e": 2.0, "d": -3.0, "yaw_deg": 0.0,
                             "state": "dispensing", "owner": "seeding", "dispense": True}
    with pytest.raises(ValueError):
        rec.sample(50.0, (0, 0, 0), (1, 0, 0, 0), "idle", "none", False)


def test_frames_and_detections_are_shifted_onto_the_mission_clock():
    rec = SeedingTraceRecorder()
    rec.sample(500.0, (0, 0, -8), (1, 0, 0, 0), "takeoff", "px4", False)       # before the mission started
    rec.sample(512.5, (30, 10, -15), (1, 0, 0, 0), "dispensing", "seeding", True)
    rec.detection(520.0, 4)
    doc = build({"scenario_id": "nominal", "mission_result": "completed", "orchestrator_evidence": {}},
                rec, 510.0, release_ned=(30.0, 10.0, -15.0), expected_result="completed")
    assert [f["t"] for f in doc["frames"]] == [-10.0, 2.5]
    assert doc["detections"] == [{"t": 10.0, "id": 4}]
    assert doc["result_as_designed"] is True and doc["truth_used_for_guidance"] is False


@pytest.mark.parametrize("name, denied_gate", [
    ("01_altitude_denial", "altitude"), ("02_weather_denial", "weather"), ("03_airspace_denial", "airspace"),
    ("04_traffic_conflict", "traffic"), ("05_clearance_expired", "comms")])
def test_each_denial_scenario_shows_exactly_its_own_gate_closed(name, denied_gate):
    gates = _gate_verdicts(_report(name)["orchestrator_evidence"])
    assert set(gates) == set(GATES)
    closed = [g for g, v in gates.items() if not v["authorized"]]
    assert closed == [denied_gate], gates


def test_the_nominal_sortie_has_every_gate_open():
    gates = _gate_verdicts(_report("00_nominal")["orchestrator_evidence"])
    assert all(v["authorized"] for v in gates.values()), gates
    assert 12.0 <= gates["altitude"]["altitude_m"] <= 18.0


def test_terminal_pose_is_recorded_even_between_periodic_samples():
    rec = SeedingTraceRecorder(rate_hz=10)
    rec.sample(100, (1, 2, -0.1), (1, 0, 0, 0), "precision_landing", "landing", False)
    assert not rec.sample(100.02, (1, 2, 0), (1, 0, 0, 0), "landed", "none", False)
    assert rec.sample(100.02, (1, 2, 0), (1, 0, 0, 0), "landed", "none", False, force=True)
    doc = build({"orchestrator_evidence": {"mission_state_history": [
        {"time_s": 0.01, "source": "precision_landing", "destination": "landed"}]}},
        rec, 100, release_ned=(0, 0, 0), expected_result="completed")
    assert doc["frames"][-1]["t"] >= 0.01
    assert doc["frames"][-1]["state"] == "landed"
    with pytest.raises(ValueError):
        rec.sample(99, (0, 0, 0), (1, 0, 0, 0), "landed", "none", False, force=True)
