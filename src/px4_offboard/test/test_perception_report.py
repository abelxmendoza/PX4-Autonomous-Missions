"""Pure offline tests for perception replay verification."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import pytest
from px4_offboard.bench.perception_report import PerceptionReportError, verify_perception_report

ROOT = Path(__file__).resolve().parents[3]


def report(outcomes=("PASS", "PASS", "PASS")):
    statuses = ("CLEAR", "STOP", "SENSOR_FAULT")
    events = [{"timestamp_s": float(i), "status": s, "outcome": o} for i, (s, o) in enumerate(zip(statuses, outcomes))]
    return {
        "report_type": "bench_replay_report", "schema_version": 1,
        "recording": {"name": "fixture.jsonl", "source": "synthetic", "source_label": "Synthetic", "metadata": {}},
        "policy": {},
        "summary": {"total": 3, "statuses": {"CLEAR": 1, "STOP": 1, "SENSOR_FAULT": 1}, "outcomes": {key: outcomes.count(key) for key in ("PASS", "FAIL", "SKIP")}},
        "events": events,
    }


def test_complete_passing_report_meets_requirements():
    result = verify_perception_report(report())
    assert result.passed and result.status_counts["SENSOR_FAULT"] == 1


def test_skip_is_not_pass_unless_explicitly_allowed():
    payload = report(("PASS", "SKIP", "PASS"))
    assert not verify_perception_report(payload).passed
    assert verify_perception_report(payload, allow_skip=True).passed


def test_fail_fails_even_when_skip_is_allowed():
    assert not verify_perception_report(report(("PASS", "FAIL", "SKIP")), allow_skip=True).passed


@pytest.mark.parametrize("mutation", ["version", "counts", "timestamp"])
def test_invalid_or_tampered_reports_are_rejected(mutation):
    payload = copy.deepcopy(report())
    if mutation == "version": payload["schema_version"] = 2
    elif mutation == "counts": payload["summary"]["outcomes"]["PASS"] = 99
    else: payload["events"][2]["timestamp_s"] = -1
    with pytest.raises(PerceptionReportError):
        verify_perception_report(payload)


def test_cli_exit_codes_distinguish_pass_skip_and_malformed(tmp_path):
    tool, path = ROOT / "tools/verify_perception_report.py", tmp_path / "report.json"
    path.write_text(json.dumps(report()), encoding="utf-8")
    assert subprocess.run([sys.executable, str(tool), str(path)], capture_output=True).returncode == 0
    path.write_text(json.dumps(report(("PASS", "SKIP", "PASS"))), encoding="utf-8")
    assert subprocess.run([sys.executable, str(tool), str(path)], capture_output=True).returncode == 1
    assert subprocess.run([sys.executable, str(tool), str(path), "--allow-skip"], capture_output=True).returncode == 0
    path.write_text("{broken", encoding="utf-8")
    assert subprocess.run([sys.executable, str(tool), str(path)], capture_output=True).returncode == 2


@pytest.mark.parametrize('field', ['status', 'outcome'])
def test_unhashable_event_values_are_validation_errors(field):
    payload = report()
    payload['events'][0][field] = []
    with pytest.raises(PerceptionReportError):
        verify_perception_report(payload)


def test_boolean_schema_version_is_not_an_integer_version():
    payload = report()
    payload['schema_version'] = True
    with pytest.raises(PerceptionReportError):
        verify_perception_report(payload)
