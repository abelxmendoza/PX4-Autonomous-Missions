"""Tests for the ROS-free ULog verification entry point, using a stub pyulog module."""
import importlib.util
import json
import subprocess
import sys
import types
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "flight_log_verify.py"
spec = importlib.util.spec_from_file_location("flight_log_verify", SCRIPT)
verifier = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verifier)


def inspect(monkeypatch, fields=None, present=True):
    if fields is None:
        fields = {"timestamp": [0, 100000, 200000], "x": [0., 1., 2.],
                  "y": [0., 0., 0.], "z": [0., 0., 0.]}
    data = [types.SimpleNamespace(name="vehicle_local_position", multi_id=0, data=fields)] if present else []
    monkeypatch.setitem(sys.modules, "pyulog", types.SimpleNamespace(
        ULog=lambda *args, **kwargs: types.SimpleNamespace(data_list=data)))
    return verifier.inspect_ulog(Path("fixture.ulg"), max_gap_s=0.15)


def check(report, name):
    return next(item for item in report["checks"] if item["id"] == name)


def test_normal_samples_pass(monkeypatch):
    report = inspect(monkeypatch)
    assert report["summary"] == {"pass": 2, "fail": 0, "skip": 0}


def test_missing_topic_is_skip(monkeypatch):
    report = inspect(monkeypatch, present=False)
    assert report["summary"] == {"pass": 0, "fail": 0, "skip": 1}


def test_missing_field_is_skip(monkeypatch):
    report = inspect(monkeypatch, fields={"timestamp": [1], "x": [0]})
    assert check(report, "LOCAL_POSITION_VALID")["result"] == "SKIP"


def test_nan_position_fails(monkeypatch):
    report = inspect(monkeypatch, fields={"timestamp": [1, 2], "x": [0, float("nan")],
                                           "y": [0, 0], "z": [0, 0]})
    assert check(report, "LOCAL_POSITION_FINITE")["result"] == "FAIL"
    assert check(report, "LOCAL_POSITION_FINITE")["first_invalid_timestamp_us"] == 2


def test_large_gap_fails(monkeypatch):
    report = inspect(monkeypatch, fields={"timestamp": [0, 200000], "x": [0, 0],
                                           "y": [0, 0], "z": [0, 0]})
    assert check(report, "LOCAL_POSITION_SAMPLE_GAP")["result"] == "FAIL"


def test_reversed_timestamp_fails(monkeypatch):
    report = inspect(monkeypatch, fields={"timestamp": [100, 99], "x": [0, 0],
                                           "y": [0, 0], "z": [0, 0]})
    assert check(report, "LOCAL_POSITION_SAMPLE_GAP")["result"] == "FAIL"


def test_one_sample_skips_gap(monkeypatch):
    report = inspect(monkeypatch, fields={"timestamp": [1], "x": [0],
                                           "y": [0], "z": [0]})
    assert check(report, "LOCAL_POSITION_SAMPLE_GAP")["result"] == "SKIP"


def test_cli_rejects_nonpositive_gap(tmp_path):
    f = tmp_path / "test.ulg"
    f.touch()
    proc = subprocess.run([sys.executable, str(SCRIPT), str(f), "--max-gap-s", "0"],
                          capture_output=True, text=True)
    assert proc.returncode == 2
    assert "finite and positive" in proc.stderr


@pytest.mark.parametrize('fields', [
    {'timestamp': [0, 1], 'x': [0], 'y': [0, 0], 'z': [0, 0]},
    {'timestamp': [float('nan')], 'x': [0], 'y': [0], 'z': [0]},
])
def test_malformed_sample_arrays_are_input_errors(monkeypatch, fields):
    with pytest.raises(ValueError):
        inspect(monkeypatch, fields=fields)
