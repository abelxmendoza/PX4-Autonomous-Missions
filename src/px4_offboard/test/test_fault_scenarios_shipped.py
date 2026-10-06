"""The scenario files shipped in config/fault_scenarios must load, gate OK, and
carry exactly the known gaps we have documented -- no silent additions."""
from __future__ import annotations

from pathlib import Path

import pytest

from px4_offboard.faults.evidence import run_scenario
from px4_offboard.faults.schema import load_scenario

SCENARIO_DIR = Path(__file__).resolve().parents[3] / "config" / "fault_scenarios"
SCENARIOS = sorted(SCENARIO_DIR.glob("*.yaml"))

pytestmark = pytest.mark.integration  # runs several 60 s simulated flights


def test_scenarios_exist():
    assert {p.stem for p in SCENARIOS} >= {
        "vo_dropout_and_packet_loss",
        "sensor_faults",
        "link_faults",
        "imu_faults_during_vo_outage",
    }


@pytest.mark.parametrize("path", SCENARIOS, ids=lambda p: p.stem)
def test_shipped_scenario_gates_ok(path):
    evidence = run_scenario(load_scenario(path))
    failing = [r["id"] for r in evidence.records if r["status"] == "FAIL"]
    assert not failing, failing
    obsolete = [r["id"] for r in evidence.records if r["status"] == "PASS_GAP_OBSOLETE"]
    assert not obsolete, f"known_gap marker is stale (the fault now passes): {obsolete}"


def test_the_only_known_gaps_are_the_documented_ones():
    gaps = {}
    for path in SCENARIOS:
        for record in run_scenario(load_scenario(path)).records:
            if record["status"] == "KNOWN_GAP":
                gaps[record["id"]] = record["known_gap"]
    assert gaps == {
        "sensor_faults/9-frozen_sensor": "BUG-020",
        "imu_faults_during_vo_outage/4-frozen_sensor": "BUG-020",
    }
