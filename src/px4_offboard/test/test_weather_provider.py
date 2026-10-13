"""AWC METAR parsing and providers. No live HTTP call."""

from __future__ import annotations

import json
from datetime import datetime, timezone

import pytest

from px4_offboard.weather import (
    Applicability,
    ClockDomain,
    Provenance,
    Reason,
    WeatherFetch,
    simulation_observation,
)
from px4_offboard.weather_provider import (
    HPA_TO_PA,
    KNOT_TO_MPS,
    CachedWeatherProvider,
    SimulatedWeatherProvider,
    StaticWeatherProvider,
    failure_segment,
    fetch_awc_metar,
    observation_from_awc_metar,
)


KMCI = {
    "icaoId": "KMCI",
    "receiptTime": "2023-11-03 21:54:03.000Z",
    "obsTime": 1699048260,
    "temp": 14.4,
    "wdir": 180,
    "wspd": 6,
    "wgst": 12,
    "altim": 1013.0,
    "slp": 1016.2,
    "lat": 39.2976,
    "lon": -94.7139,
    "elev": 313,
}


def test_metar_units_are_converted_and_the_report_stays_a_surface_observation():
    observation = observation_from_awc_metar(KMCI)
    assert observation.source == "awc-metar:KMCI"
    assert observation.provenance is Provenance.RECORDED
    assert observation.clock is ClockDomain.UNIX
    assert observation.applicability is Applicability.SURFACE
    assert observation.observed_at_s == pytest.approx(1699048260.0)
    assert observation.received_at_s == pytest.approx(
        datetime(2023, 11, 3, 21, 54, 3, tzinfo=timezone.utc).timestamp()
    )
    assert observation.wind_speed_mps == pytest.approx(6 * KNOT_TO_MPS)
    assert observation.wind_gust_mps == pytest.approx(12 * KNOT_TO_MPS)
    assert observation.wind_direction_deg == pytest.approx(180.0)
    assert observation.temperature_c == pytest.approx(14.4)
    assert observation.pressure_pa == pytest.approx(1016.2 * HPA_TO_PA)
    assert observation.elevation_m == pytest.approx(313.0)
    assert observation.relative_humidity_pct is None


def test_a_fixture_is_not_labeled_as_a_live_report():
    assert observation_from_awc_metar([KMCI]).provenance is Provenance.RECORDED


def test_variable_wind_direction_stays_missing():
    payload = dict(KMCI, wdir="VRB")
    assert observation_from_awc_metar(payload).wind_direction_deg is None


def test_a_forecast_payload_is_rejected():
    with pytest.raises(Exception) as caught:
        observation_from_awc_metar({"icaoId": "KMCI", "fcsts": []})
    assert caught.value.reason_code == Reason.UNSUPPORTED_PRODUCT


def test_fetch_uses_the_injected_transport_and_labels_the_report_live():
    calls = []

    class Response:
        status = 200

        def read(self):
            return json.dumps([KMCI]).encode()

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

    def transport(url, timeout=10.0):
        calls.append((url, timeout))
        return Response()

    fetch = fetch_awc_metar("KMCI", urlopen_fn=transport, timeout_s=2.0)
    assert calls == [("https://aviationweather.gov/api/data/metar?ids=KMCI&format=json", 2.0)]
    assert fetch.reason_code == ""
    assert fetch.observation is not None
    assert fetch.observation.provenance is Provenance.LIVE


def test_an_empty_metar_response_is_a_missing_observation():
    class Response:
        status = 204

        def read(self):
            return b""

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

    fetch = fetch_awc_metar("KMCI", urlopen_fn=lambda url, timeout=10.0: Response())
    assert fetch.reason_code == Reason.MISSING_OBSERVATION
    assert fetch.observation is None


def test_http_failure_is_a_provider_failure():
    class Response:
        status = 503

        def read(self):
            return b"busy"

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

    fetch = fetch_awc_metar("KMCI", urlopen_fn=lambda url, timeout=10.0: Response())
    assert fetch.reason_code == Reason.PROVIDER_FAILURE


def test_simulated_provider_changes_with_mission_time_and_static_does_not():
    calm = simulation_observation(wind_speed_mps=3.0, wind_gust_mps=5.0)
    windy = simulation_observation(wind_speed_mps=20.0, wind_gust_mps=25.0)
    simulated = SimulatedWeatherProvider([(0.0, calm), (5.0, windy), failure_segment(9.0, "timeout")])
    assert simulated.latest(4.9).observation.wind_speed_mps == pytest.approx(3.0)
    assert simulated.latest(5.0).observation.wind_speed_mps == pytest.approx(20.0)
    assert simulated.latest(9.0).reason_code == Reason.PROVIDER_FAILURE

    static = StaticWeatherProvider(calm)
    assert static.latest(0.0).observation is static.latest(100.0).observation


def test_cached_provider_returns_the_stored_fetch_without_refetching():
    cache = CachedWeatherProvider()
    assert cache.latest(0.0).reason_code == Reason.MISSING_OBSERVATION
    stored = observation_from_awc_metar(KMCI, provenance=Provenance.LIVE)
    cache.store(WeatherFetch(stored, ""))
    assert cache.latest(50.0).observation.provenance is Provenance.LIVE
    assert cache.latest(50.0).observation.source == "awc-metar:KMCI"
