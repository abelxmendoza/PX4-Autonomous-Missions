"""Declarative fault scenarios: parsing and validation."""
from __future__ import annotations

import pytest

from px4_offboard.faults.schema import FAULT_TYPES, ScenarioError, load_scenario_text

EXAMPLE = """
name: example
duration_s: 60
seed: 3
faults:
  - type: vo_dropout
    start_s: 20
    duration_s: 5
  - type: mavlink_packet_loss
    start_s: 40
    duration_s: 3
    probability: 0.30
"""


def test_the_example_scenario_parses():
    sc = load_scenario_text(EXAMPLE)
    assert sc.name == "example" and sc.duration_s == 60 and sc.seed == 3
    assert [f.type for f in sc.faults] == ["vo_dropout", "mavlink_packet_loss"]
    assert sc.faults[1].params["probability"] == pytest.approx(0.30)
    assert (sc.faults[0].start_s, sc.faults[0].end_s) == (20, 25)


def test_defaults_are_filled_in_from_the_fault_type():
    sc = load_scenario_text("faults:\n  - {type: dropped_messages, start_s: 1, duration_s: 2, probability: 0.5}\n")
    assert sc.faults[0].params["sensor"] == "vo"
    assert sc.duration_s == 60.0


def test_every_fault_type_documents_its_domain_and_expected_behaviour():
    assert {"vo_dropout", "dropped_messages", "delayed_messages", "timestamp_jitter",
            "corrupted_measurement", "frozen_sensor", "estimator_reset",
            "mavlink_packet_loss", "comm_disconnect"} <= set(FAULT_TYPES)
    for name, info in FAULT_TYPES.items():
        assert info.domain in ("sensor", "link"), name
        assert info.expected_text, name


@pytest.mark.parametrize(
    "body,fragment",
    [
        ("faults:\n  - {type: gremlins, start_s: 1, duration_s: 1}", "unknown fault type"),
        ("faults:\n  - {type: vo_dropout, duration_s: 1}", "start_s"),
        ("faults:\n  - {type: vo_dropout, start_s: -1, duration_s: 1}", "start_s"),
        ("faults:\n  - {type: vo_dropout, start_s: 1, duration_s: 0}", "duration_s"),
        ("faults:\n  - {type: mavlink_packet_loss, start_s: 1, duration_s: 1}", "probability"),
        ("faults:\n  - {type: mavlink_packet_loss, start_s: 1, duration_s: 1, probability: 1.5}", "probability"),
        ("faults:\n  - {type: vo_dropout, start_s: 1, duration_s: 1, colour: red}", "unknown parameter"),
        ("faults:\n  - {type: dropped_messages, start_s: 1, duration_s: 1, probability: 0.1, sensor: lidar}", "sensor"),
        ("duration_s: 10\nfaults:\n  - {type: vo_dropout, start_s: 9, duration_s: 5}", "ends after"),
        ("faults: []", "at least one fault"),
        ("faults: 3", "list"),
        ("name: x\n", "faults"),
    ],
)
def test_invalid_scenarios_are_rejected_with_a_useful_message(body, fragment):
    with pytest.raises(ScenarioError) as exc:
        load_scenario_text(body)
    assert fragment in str(exc.value)


def test_expectation_overrides_are_validated_and_merged():
    sc = load_scenario_text(
        "faults:\n  - type: vo_dropout\n    start_s: 5\n    duration_s: 2\n    expect: {recover_within_s: 4.0}\n"
    )
    assert sc.faults[0].expect["recover_within_s"] == 4.0
    with pytest.raises(ScenarioError):
        load_scenario_text(
            "faults:\n  - type: vo_dropout\n    start_s: 5\n    duration_s: 2\n    expect: {bogus: 1}\n"
        )


def test_estimator_reset_is_instantaneous_and_needs_no_duration():
    sc = load_scenario_text("faults:\n  - {type: estimator_reset, start_s: 10}\n")
    assert sc.faults[0].duration_s == 0.0


def test_known_gap_marker_is_carried_through():
    sc = load_scenario_text(
        "faults:\n  - {type: frozen_sensor, start_s: 5, duration_s: 3, known_gap: 'BUG-099'}\n"
    )
    assert sc.faults[0].known_gap == "BUG-099"


def test_background_faults_are_parsed_separately_from_the_faults_under_test():
    sc = load_scenario_text(
        "background:\n  - {type: vo_dropout, start_s: 5, duration_s: 20}\n"
        "faults:\n  - {type: frozen_sensor, sensor: imu, start_s: 10, duration_s: 5}\n"
    )
    assert [f.type for f in sc.background] == ["vo_dropout"]
    assert [f.type for f in sc.faults] == ["frozen_sensor"]


def test_a_scenario_with_only_background_is_rejected():
    with pytest.raises(ScenarioError):
        load_scenario_text("background:\n  - {type: vo_dropout, start_s: 5, duration_s: 2}\nfaults: []\n")
