"""Replay must preserve raw terminal evidence, including off-grid timestamps."""
import importlib.util
import json
from pathlib import Path

from test_swarm_logic import simulate

spec = importlib.util.spec_from_file_location(
    "export_swarm_replay", Path(__file__).resolve().parents[3] / "scripts/export_swarm_replay.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_export_preserves_terminal_sample_between_playback_ticks(tmp_path):
    path = tmp_path / "recovery.jsonl"
    simulate(path, dropout=True)
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    rows[-1]["time"] += 0.035
    path.write_text("\n".join(json.dumps(row) for row in rows) + "\n")
    replay = module.export(path)
    assert replay["verification"]["passed"]
    assert replay["frames"][-1]["phase"] == "COMPLETE"
    assert replay["frames"][-1]["time"] == rows[-1]["time"] - rows[1]["time"]
    assert all(v["state"] == "LANDED" for v in replay["frames"][-1]["vehicles"].values())


def test_export_recomputes_verdict_instead_of_trusting_sidecar(tmp_path):
    path = tmp_path / "truncated.jsonl"
    simulate(path)
    path.write_text("\n".join(path.read_text().splitlines()[:30]) + "\n")
    (tmp_path / "verification.json").write_text(json.dumps({"passed": True}))
    assert not module.export(path)["verification"]["passed"]
