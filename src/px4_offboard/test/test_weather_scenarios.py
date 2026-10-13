"""Reproducible seeding weather scenarios. They do not use the network."""

from __future__ import annotations

import json

from px4_offboard.weather import Reason
from px4_offboard.weather_scenarios import (
    scenario_high_wind_before_release,
    scenario_nominal,
    scenario_stale_feed,
    scenario_surface_weather_at_15000_ft,
    scenario_weather_invalid_during_dispense,
)


def test_scenario_nominal_completes_with_a_simulated_feed():
    evidence = scenario_nominal()
    assert evidence["mission"]["phase"] == "complete"
    assert evidence["mission"]["violations"] == []
    assert evidence["mission"]["sample_count"] > 0
    assert evidence["operational_approval"] is False
    opened = [item for item in evidence["weather_decisions"] if item["dispenser_open"]]
    assert opened
    assert all(item["gates_authorized"] and item["provenance"] == "simulated" for item in opened)
    json.dumps(evidence, allow_nan=False)


def test_scenario_high_wind_returns_without_a_release():
    evidence = scenario_high_wind_before_release()
    assert evidence["mission"]["phase"] == "abort"
    assert evidence["mission"]["reason"] == Reason.WIND_ABOVE_LIMIT
    assert evidence["mission"]["sample_count"] == 0
    assert all(item["dispenser_open"] is False for item in evidence["weather_decisions"])


def test_scenario_wind_during_dispense_keeps_the_earlier_sample():
    evidence = scenario_weather_invalid_during_dispense()
    assert evidence["mission"]["phase"] == "abort"
    assert evidence["mission"]["reason"] == Reason.WIND_ABOVE_LIMIT
    assert evidence["mission"]["sample_count"] == 1
    assert evidence["mission"]["violations"] == []
    opened = [item for item in evidence["weather_decisions"] if item["dispenser_open"]]
    assert len(opened) == 1
    closed = evidence["weather_decisions"][-2]
    assert closed["dispenser_open"] is False
    assert closed["reason_code"] == Reason.WIND_ABOVE_LIMIT


def test_scenario_stale_feed_names_the_age_failure():
    evidence = scenario_stale_feed()
    assert evidence["mission"]["phase"] == "abort"
    assert evidence["mission"]["reason"] == Reason.STALE_OBSERVATION
    assert evidence["mission"]["sample_count"] == 0
    assert evidence["weather_decisions"][-2]["status"] == "stale"


def test_scenario_surface_weather_does_not_cover_15000_ft():
    evidence = scenario_surface_weather_at_15000_ft()
    assert evidence["mission"]["phase"] == "abort"
    assert evidence["mission"]["reason"] == Reason.SURFACE_OBSERVATION_INSUFFICIENT
    assert evidence["mission"]["sample_count"] == 0
    denial = evidence["weather_decisions"][-2]
    assert denial["applicability"] == "surface"
    assert denial["release_altitude_amsl_m"] == 4572.0
    assert denial["provenance"] == "simulated"
    assert evidence["decisions"][-2]["authorized"] is True
