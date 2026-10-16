"""Kinematic scenarios for the five release gates. These are not PX4 flights."""

from px4_offboard.flight_management_scenarios import (
    scenario_airspace_during_release,
    scenario_airspace_fails,
    scenario_airspace_outranks_comms,
    scenario_all_gates_pass,
    scenario_altitude_fails,
    scenario_altitude_outranks_traffic,
    scenario_clearance_expires_before_release,
    scenario_clearance_expires_during_dispense,
    scenario_conflict_during_dispense,
    scenario_converging_before_release,
    scenario_nominal_precision_landing,
    scenario_stale_authorization,
    scenario_traffic_provider_unavailable,
    scenario_weather_fails,
)


def test_scenario_all_gates_pass():
    evidence = scenario_all_gates_pass()
    assert evidence["mission_outcome"]["phase"] == "complete"
    assert evidence["mission_outcome"]["sample_count"] > 0
    assert evidence["operational_approval"] is False
    opened = [item for item in evidence["weather_decisions"] if item["dispenser_open"]]
    assert opened
    assert all(item["gates_authorized"] for item in opened)


def test_scenario_altitude_failure_is_the_primary_reason():
    evidence = scenario_altitude_fails()
    assert evidence["mission"]["sample_count"] == 0
    assert evidence["mission"]["reason"].startswith("local altitude")
    assert evidence["mission"]["phase"] == "return"


def test_scenario_weather_failure():
    evidence = scenario_weather_fails()
    assert evidence["mission"]["reason"] == "wind_above_limit"
    assert evidence["mission"]["sample_count"] == 0


def test_scenario_airspace_failure():
    evidence = scenario_airspace_fails()
    assert evidence["mission"]["reason"] == "inside_prohibited_airspace"
    assert evidence["mission"]["sample_count"] == 0


def test_scenario_airspace_appearing_during_release_keeps_the_sample():
    evidence = scenario_airspace_during_release()
    assert evidence["mission"]["reason"] == "inside_prohibited_airspace"
    assert evidence["mission"]["sample_count"] == 1
    assert evidence["airspace_decisions"][-1]["dispenser_open"] is False


def test_scenario_stale_traffic_denies_release():
    evidence = scenario_stale_authorization()
    assert evidence["mission"]["reason"] == "traffic_data_stale"
    assert evidence["mission"]["sample_count"] == 0


def test_scenario_converging_traffic_denies_before_release():
    evidence = scenario_converging_before_release()
    assert evidence["mission"]["reason"] == "traffic_predicted_conflict"
    assert evidence["mission"]["sample_count"] == 0


def test_scenario_conflict_during_dispense_closes_the_valve():
    evidence = scenario_conflict_during_dispense()
    assert evidence["mission"]["reason"] == "traffic_conflict_detected"
    assert evidence["mission"]["sample_count"] == 1
    assert evidence["traffic_decisions"][-1]["authorized"] is True
    assert evidence["traffic_decisions"][-1]["dispenser_open"] is False


def test_scenario_expired_clearance_before_and_during_release():
    before = scenario_clearance_expires_before_release()
    assert before["mission"]["reason"] == "simulated_clearance_expired"
    assert before["mission"]["sample_count"] == 0
    during = scenario_clearance_expires_during_dispense()
    assert during["mission"]["reason"] == "simulated_clearance_expired"
    assert during["mission"]["sample_count"] == 1
    assert during["communications_decisions"][-1]["operational_approval"] is False


def test_scenario_gate_priority_and_provider_loss():
    altitude = scenario_altitude_outranks_traffic()
    assert altitude["mission"]["reason"].startswith("local altitude")
    assert altitude["traffic_decisions"][-1]["authorized"] is False
    airspace = scenario_airspace_outranks_comms()
    assert airspace["mission"]["reason"] == "inside_prohibited_airspace"
    assert airspace["communications_decisions"][-1]["reason_code"] == "simulated_clearance_missing"
    missing = scenario_traffic_provider_unavailable()
    assert missing["mission"]["reason"] == "traffic_source_unavailable"
    assert missing["mission"]["sample_count"] == 0


def test_scenario_nominal_sortie_reaches_precision_landing_in_kinematics():
    evidence = scenario_nominal_precision_landing()
    assert evidence["mission_result"] == "completed"
    assert evidence["altitude_evidence"]["operational_approval"] is False
    assert evidence["traffic_evidence"]["consulted"] is True
    assert evidence["communications_evidence"]["operational_approval"] is False
    assert evidence["precision_landing_evidence"]["failed"] is False
