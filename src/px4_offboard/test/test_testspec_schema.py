"""Test-spec schema: everything malformed is rejected before anything runs."""
from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from px4_offboard.testspec import actions, assertions
from px4_offboard.testspec.schema import (
    ACTIONS,
    ADAPTERS,
    ASSERTIONS,
    SpecError,
    load_spec,
    parse_spec_text,
)
from px4_offboard.validation.registry import load_registry

ROOT = Path(__file__).resolve().parents[3]
EXAMPLE = ROOT / "config" / "test_specs" / "basic_mission.yaml"
KNOWN = {r.id for r in load_registry().requirements}


def _doc() -> dict:
    return yaml.safe_load(EXAMPLE.read_text())


def _parse(doc: dict):
    return parse_spec_text(yaml.safe_dump(doc), KNOWN)


def _rejects(doc: dict, *fragments: str) -> str:
    with pytest.raises(SpecError) as exc:
        _parse(doc)
    for f in fragments:
        assert f in str(exc.value), str(exc.value)
    return str(exc.value)


def test_every_shipped_spec_is_valid():
    specs = sorted((ROOT / "config" / "test_specs").glob("*.yaml"))
    assert specs
    for path in specs:
        load_spec(path, KNOWN)


def test_example_parses_with_defaults_filled_in():
    spec = load_spec(EXAMPLE, KNOWN)
    assert spec.vehicle.adapter == "mock"
    assert [s.action for s in spec.mission] == ["arm", "takeoff", "goto", "goto", "hold", "goto", "land", "disarm"]
    assert spec.mission[0].params == {"timeout_s": 5.0}
    assert spec.mission[2].params["tolerance_m"] == 0.3
    assert spec.requirement_ids == ("REQ-GEOFENCE-007",)
    assert len(spec.sha256) == 64


def test_catalogues_match_what_the_runner_can_execute_and_evaluate():
    assert set(ACTIONS) == set(actions.EXECUTORS)
    assert set(ASSERTIONS) == set(assertions.EVALUATORS)
    assert set(ADAPTERS) == {"mock"}


def test_unknown_top_level_key_is_rejected():
    doc = _doc()
    doc["setup_script"] = "rm -rf /"
    _rejects(doc, "unknown key", "setup_script")


@pytest.mark.parametrize("schema", [None, 2, True, "1"])
def test_schema_version_must_be_exactly_one(schema):
    doc = _doc()
    if schema is None:
        del doc["schema"]
    else:
        doc["schema"] = schema
    _rejects(doc, "schema must be 1")


def test_unknown_action_is_rejected():
    doc = _doc()
    doc["mission"][2]["action"] = "teleport"
    _rejects(doc, "mission[2]", "unknown action 'teleport'")


def test_unknown_assertion_type_is_rejected():
    doc = _doc()
    doc["assertions"][0]["type"] = "python"
    _rejects(doc, "assertions[0]", "unknown assertion type 'python'")


def test_unsupported_adapter_is_rejected():
    doc = _doc()
    doc["vehicle"]["adapter"] = "px4_sitl"
    _rejects(doc, "unsupported adapter 'px4_sitl'")


@pytest.mark.parametrize("fault", [{"type": "comm_disconnect", "start_s": 1, "duration_s": 1}, {"type": "made_up"}, "x"])
def test_any_fault_is_unsupported_by_the_mock_adapter(fault):
    doc = _doc()
    doc["faults"] = [fault]
    _rejects(doc, "faults[0]", "unsupported fault", "supported: none")


def test_faults_must_be_a_list():
    doc = _doc()
    doc["faults"] = {"type": "comm_disconnect"}
    _rejects(doc, "faults must be a list")


@pytest.mark.parametrize(
    "index, key, value, fragment",
    [
        (1, "altitude_m", 500.0, "within [0.5, 120.0]"),
        (1, "altitude_m", True, "must be a number"),
        (1, "altitude_m", "5", "must be a number"),
        (2, "north_m", float("nan"), "must be finite"),
        (2, "north_m", float("inf"), "must be finite"),
        (2, "timeout_s", 0, "within"),
        (2, "timeout_s", -1, "within"),
        (2, "colour", "red", "unknown key"),
        (4, "timeout_s", 5, "unknown key"),  # hold is bounded by its duration
    ],
)
def test_invalid_action_params_are_rejected(index, key, value, fragment):
    doc = _doc()
    doc["mission"][index][key] = value
    _rejects(doc, f"mission[{index}]", fragment)


def test_missing_required_param_is_rejected():
    doc = _doc()
    del doc["mission"][2]["east_m"]
    _rejects(doc, "mission[2] (goto): east_m is required")


@pytest.mark.parametrize(
    "mission, fragment",
    [
        ([{"action": "takeoff", "altitude_m": 5}], "mission[0] (takeoff): not allowed while disarmed"),
        ([{"action": "arm"}, {"action": "goto", "north_m": 1, "east_m": 1, "altitude_m": 2}], "on the ground"),
        ([{"action": "arm"}, {"action": "takeoff", "altitude_m": 5}, {"action": "disarm"}], "airborne"),
        ([{"action": "arm"}, {"action": "arm"}], "mission[1] (arm)"),
        ([], "non-empty"),
    ],
)
def test_impossible_action_orders_are_rejected(mission, fragment):
    doc = _doc()
    doc["mission"] = mission
    _rejects(doc, fragment)


def test_duplicate_step_ids_are_rejected():
    doc = _doc()
    doc["mission"][3]["id"] = "leg_north"
    _rejects(doc, "duplicate step id")


@pytest.mark.parametrize(
    "mutate, fragment",
    [
        (lambda a: a.pop("id"), "id must match"),
        (lambda a: a.update(id="Bad Id"), "id must match"),
        (lambda a: a.update(requirements=["GEOFENCE-7"]), "requirements must be a list"),
        (lambda a: a.update(requirements="REQ-GEOFENCE-007"), "requirements must be a list"),
        (lambda a: a.update(requirements=["REQ-NOPE-999"]), "not in requirements/registry.yaml"),
        (lambda a: a.update(requirements=["REQ-GEOFENCE-007", "REQ-GEOFENCE-007"]), "duplicate requirement"),
        (lambda a: a.pop("polygon_ned"), "polygon_ned is required"),
        (lambda a: a.update(polygon_ned=[[0, 0], [1, 1]]), "at least 3"),
        (lambda a: a.update(polygon_ned=[[0, 0], [1], [1, 1]]), "pair"),
        (lambda a: a.update(polygon_ned=[[0, 0], [10, 10], [10, 0], [0, 10]]), "invalid keep-in"),
        (lambda a: a.update(min_altitude_m=20.0), "invalid keep-in"),
        (lambda a: a.update(expression="lambda s: True"), "unknown key"),
        (lambda a: a.update(note=3), "note must be a string"),
    ],
)
def test_malformed_assertions_are_rejected(mutate, fragment):
    doc = _doc()
    mutate(doc["assertions"][0])
    _rejects(doc, "assertions[0]", fragment)


def test_max_speed_needs_at_least_one_limit_and_ids_are_unique():
    doc = _doc()
    doc["assertions"][2] = {"id": "speed_envelope", "type": "max_speed"}
    _rejects(doc, "horizontal_mps, vertical_mps or both")
    doc = _doc()
    doc["assertions"][1]["id"] = "stays_in_keep_in"
    _rejects(doc, "duplicate assertion id")


def test_a_spec_without_assertions_cannot_be_valid():
    doc = _doc()
    doc["assertions"] = []
    _rejects(doc, "cannot pass")


def test_duplicate_yaml_keys_are_rejected_not_silently_overwritten():
    text = EXAMPLE.read_text().replace("schema: 1\n", "schema: 1\nname: shadow\n", 1)
    with pytest.raises(SpecError, match="duplicate key 'name'"):
        parse_spec_text(text, KNOWN)


def test_python_object_tags_are_not_constructed():
    text = "!!python/object/apply:os.system ['echo pwned']\n"
    with pytest.raises(SpecError, match="invalid YAML"):
        parse_spec_text(text, KNOWN)


def test_invalid_yaml_and_non_mapping_are_rejected():
    with pytest.raises(SpecError, match="invalid YAML"):
        parse_spec_text("mission: [unclosed", KNOWN)
    with pytest.raises(SpecError, match="must be a mapping"):
        parse_spec_text("- just\n- a list\n", KNOWN)


def test_adapter_params_are_validated():
    doc = _doc()
    doc["vehicle"]["rate_hz"] = 0.0
    _rejects(doc, "vehicle (mock): rate_hz")
    doc = _doc()
    doc["vehicle"]["px4_param"] = 1
    _rejects(doc, "unknown key")
