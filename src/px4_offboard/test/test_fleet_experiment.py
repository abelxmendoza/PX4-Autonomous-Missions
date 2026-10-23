"""Experiment preflight is strict and side-effect-free, not a flight test."""
from dataclasses import replace
import json
from pathlib import Path
import subprocess

import pytest

from px4_offboard.fleet_experiment import main, parse_experiment, validate_layout


def config(n=3):
    return dict(schema=1, fleet_size=n, scenario="gate_visit_nominal", seed=42,
                output_dir="demo_artifacts/fleet/example")


@pytest.mark.parametrize("n", (1, 2, 3, 4))
def test_supported_sizes_have_deterministic_unique_bindings_and_tasks(n):
    experiment = parse_experiment(config(n))
    assert experiment.to_dict() == parse_experiment(config(n)).to_dict()
    assert len(experiment.fleet.tasks) == 2 * n
    assert {v.namespace for v in experiment.vehicles} == {f"/px4_{i}" for i in range(1, n + 1)}
    assert [v.target_system_id for v in experiment.vehicles] == list(range(2, n + 2))
    assert len({v.mavlink_gcs_port for v in experiment.vehicles}) == n
    assert experiment.to_dict()["stage"] == "preflight_only"
    json.dumps(experiment.to_dict(), allow_nan=False)


@pytest.mark.parametrize("key,value", [
    ("schema", True), ("schema", 2), ("fleet_size", True), ("fleet_size", 0),
    ("fleet_size", 5), ("fleet_size", 2.0), ("seed", True), ("seed", -1),
    ("seed", 2**32), ("seed", "42"), ("scenario", "wind"), ("scenario", "gps_denial"),
    ("output_dir", ""), ("output_dir", "\x00"), ("output_dir", 42), ("faults", {}),
])
def test_rejects_invalid_inputs(key, value):
    data = config()
    data[key] = value
    with pytest.raises(ValueError):
        parse_experiment(data)


def test_unknown_fields_are_not_silently_ignored():
    data = config()
    data["wind_mps"] = 10
    with pytest.raises(ValueError, match="unknown fields"):
        parse_experiment(data)
    del data["wind_mps"]
    del data["seed"]
    with pytest.raises(ValueError, match="missing fields"):
        parse_experiment(data)


def dropout():
    return dict(config(), scenario="gate_visit_dropout", faults=[
        dict(vehicle="px4_2", kind="abort_and_land", after_ready_s=.2)])


def test_dropout_is_explicit_and_after_ready_clock_is_recorded():
    result = parse_experiment(dropout()).to_dict()
    assert result["faults"][0]["vehicle"] == "px4_2"
    assert "READY" in result["fault_clock"]


@pytest.mark.parametrize("key,value", [
    ("vehicle", "px4_9"), ("vehicle", []), ("kind", "gps_loss"),
    ("after_ready_s", 0), ("after_ready_s", -1), ("after_ready_s", True),
    ("after_ready_s", float("nan")), ("after_ready_s", float("inf")),
])
def test_fault_schedule_rejects_unexecutable_or_unsupported_faults(key, value):
    data = dropout()
    data["faults"][0][key] = value
    with pytest.raises(ValueError):
        parse_experiment(data)


def test_rejects_invalid_scenario_fault_combinations():
    for data in [dict(dropout(), fleet_size=1), dict(dropout(), faults=[]),
                 dict(dropout(), faults=dropout()["faults"] * 2),
                 dict(dropout(), scenario="gate_visit_nominal")]:
        with pytest.raises(ValueError):
            parse_experiment(data)


@pytest.mark.parametrize("field,value", [
    ("vehicle", "px4_1"), ("namespace", "/px4_1"), ("target_system_id", 2),
    ("px4_instance", 1), ("mavlink_gcs_port", 18571),
    ("mavlink_gcs_port", 65536), ("mavlink_gcs_port", True),
])
def test_preflight_rejects_identity_and_port_conflicts(field, value):
    experiment = parse_experiment(config())
    vehicles = list(experiment.vehicles)
    vehicles[1] = replace(vehicles[1], **{field: value})
    with pytest.raises(ValueError):
        validate_layout(experiment.fleet, tuple(vehicles))


@pytest.mark.parametrize("position", [(-3., -4., 0.), (100., 0., 0.),
                                      (float("nan"), 0., 0.), (24., 0., 0.)])
def test_preflight_rejects_overlap_fence_nonfinite_and_keepout(position):
    experiment = parse_experiment(config())
    homes = dict(experiment.fleet.homes, px4_2=position)
    fleet = replace(experiment.fleet, homes=homes)
    with pytest.raises(ValueError):
        validate_layout(fleet, experiment.vehicles)


def test_cli_validates_without_creating_artifacts_or_launching(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **k: pytest.fail("must not launch processes"))
    output = tmp_path / "must-not-exist"
    path = tmp_path / "config.json"
    path.write_text(json.dumps(dict(config(), output_dir=str(output))))
    assert main([str(path)]) == 0
    assert json.loads(capsys.readouterr().out)["stage"] == "preflight_only"
    assert not output.exists()
    path.write_text(json.dumps(dict(config(), fleet_size=99)))
    with pytest.raises(SystemExit) as exc:
        main([str(path)])
    assert exc.value.code == 2
    assert not output.exists()


def test_committed_example_is_accepted():
    root = Path(__file__).resolve().parents[3]
    example = root / "config/fleet_experiments/three_vehicle_dropout.json"
    assert len(parse_experiment(json.loads(example.read_text())).vehicles) == 3
