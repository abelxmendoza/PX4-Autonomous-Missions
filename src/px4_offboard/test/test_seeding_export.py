"""Publishing cannot silently replace the validated demo with failed or ambiguous runs."""
import importlib.util
import json
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    "export_seeding_replay", Path(__file__).resolve().parents[3] / "tools/export_seeding_replay.py")
export = importlib.util.module_from_spec(spec)
spec.loader.exec_module(export)


def test_unexpected_result_is_rejected_before_any_publication(tmp_path, monkeypatch):
    run = tmp_path / "failed"
    run.mkdir()
    (run / "trace.json").write_text(json.dumps({"scenario_id": "airspace_denial", "mission_result": "failed",
                                               "expected_result": "aborted_recovered", "result_as_designed": False}))
    monkeypatch.setattr(export, "ROOT", tmp_path)
    monkeypatch.setattr(export.sys, "argv", ["export", str(run), "--stamp", "campaign"])
    with pytest.raises(SystemExit, match="2"):
        export.main()
    assert not (tmp_path / "web").exists()
    assert not (tmp_path / "evidence").exists()


def test_duplicate_scenario_is_not_silently_overwritten(tmp_path, monkeypatch):
    (tmp_path / "trace.json").write_text(json.dumps({"scenario_id": "nominal", "mission_result": "completed",
                                                   "expected_result": "completed", "result_as_designed": True}))
    monkeypatch.setattr(export, "ROOT", tmp_path)
    monkeypatch.setattr(export.sys, "argv", ["export", str(tmp_path), str(tmp_path), "--stamp", "campaign"])
    with pytest.raises(SystemExit, match="2"):
        export.main()
    assert not (tmp_path / "web").exists()
