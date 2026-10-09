"""The committed single-drone search flight still scores as recorded against the ground truth."""
import json
from pathlib import Path

from px4_offboard.search_score import score

ROOT = Path(__file__).resolve().parents[3]


def test_recorded_single_drone_sweep_found_every_target_within_one_metre():
    report = json.loads((ROOT / "evidence/search/single_drone_sweep_report.json").read_text())
    truth = json.loads((ROOT / "worlds/search_field_targets.json").read_text())
    s = score(report, truth)
    assert s.missed == [] and s.false_ids == []
    assert len(s.found) == len(truth["targets"]) == 8
    assert s.max_error_m < 1.0, s.found          # recorded: max 0.50 m, mean 0.32 m
    assert "ground truth not used" in report["position_source"]
