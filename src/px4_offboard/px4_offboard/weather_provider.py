"""Weather providers. ``latest`` reads a cache. Only ``fetch_awc_metar`` uses the network.

Aviation Weather Center METAR JSON, as documented in their OpenAPI schema
(wind in knots, temperature in Celsius, altimeter and sea-level pressure in
hectopascals, station elevation in metres, ``obsTime`` as Unix seconds):

https://aviationweather.gov/api/data/metar
http://testbed.aviationweather.gov/data/schema/openapi-v4.0-beta.yaml

A parsed METAR is a surface observation on the Unix clock. It is not a
sounding, and this module does not label a simulation feed as a live report.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Callable, Sequence
from urllib.request import Request, urlopen

from .weather import (
    Applicability,
    ClockDomain,
    Provenance,
    Reason,
    WeatherFetch,
    WeatherObservation,
)

KNOT_TO_MPS = 1852.0 / 3600.0
HPA_TO_PA = 100.0
AWC_METAR_URL = "https://aviationweather.gov/api/data/metar"
USER_AGENT = "px4-autonomous-mission/weather (research; not for operational dispatch)"


class WeatherParseError(ValueError):
    def __init__(self, reason_code: str, detail: str) -> None:
        super().__init__(detail)
        self.reason_code = reason_code


@dataclass(frozen=True)
class _Segment:
    start_s: float
    observation: WeatherObservation | None = None
    failure: str | None = None


class StaticWeatherProvider:
    """One fixed fetch result for deterministic tests."""

    def __init__(
        self,
        observation: WeatherObservation | None = None,
        *,
        failure: str | None = None,
    ) -> None:
        self._observation = observation
        self._failure = failure

    def latest(self, now_s: float) -> WeatherFetch:
        del now_s
        if self._failure:
            return WeatherFetch(None, Reason.PROVIDER_FAILURE, self._failure)
        if self._observation is None:
            return WeatherFetch(None, Reason.MISSING_OBSERVATION, "no observation")
        return WeatherFetch(self._observation, "")


class SimulatedWeatherProvider:
    """The segment with the latest ``start_s`` at or before ``now_s`` is current."""

    def __init__(self, segments: Sequence[tuple[float, WeatherObservation | None] | _Segment]) -> None:
        parsed = tuple(sorted((_as_segment(item) for item in segments), key=lambda segment: segment.start_s))
        if not parsed:
            raise ValueError("simulated weather needs at least one segment")
        self._segments = parsed

    def latest(self, now_s: float) -> WeatherFetch:
        active: _Segment | None = None
        for segment in self._segments:
            if segment.start_s <= now_s:
                active = segment
        if active is None:
            return WeatherFetch(None, Reason.MISSING_OBSERVATION, "no observation yet")
        if active.failure:
            return WeatherFetch(None, Reason.PROVIDER_FAILURE, active.failure)
        if active.observation is None:
            return WeatherFetch(None, Reason.MISSING_OBSERVATION, "no observation")
        return WeatherFetch(active.observation, "")


class CachedWeatherProvider:
    """Holds whatever an external fetch last stored. ``latest`` does not fetch."""

    def __init__(self, fetch: WeatherFetch | None = None) -> None:
        self._fetch = fetch or WeatherFetch(None, Reason.MISSING_OBSERVATION, "cache empty")

    def store(self, fetch: WeatherFetch) -> None:
        self._fetch = fetch

    def latest(self, now_s: float) -> WeatherFetch:
        del now_s
        return self._fetch


def observation_from_awc_metar(
    payload: object,
    *,
    provenance: Provenance = Provenance.RECORDED,
) -> WeatherObservation:
    """Convert one AWC METAR JSON value into a surface observation.

    The default provenance is ``RECORDED``. The live client passes ``LIVE``.
    Relative humidity is left missing: this product does not report it.
    """
    item = _metar_object(payload)
    if "fcsts" in item:
        raise WeatherParseError(Reason.UNSUPPORTED_PRODUCT, "forecast product")
    station = item.get("icaoId")
    source = f"awc-metar:{station}" if isinstance(station, str) and station else "awc-metar"
    pressure = _optional_number(item.get("slp") if "slp" in item else item.get("altim"), "pressure")
    return WeatherObservation(
        source=source,
        provenance=provenance,
        clock=ClockDomain.UNIX,
        applicability=Applicability.SURFACE,
        observed_at_s=_optional_number(item.get("obsTime"), "obsTime"),
        received_at_s=_receipt_unix(item.get("receiptTime")),
        latitude_deg=_optional_number(item.get("lat"), "lat"),
        longitude_deg=_optional_number(item.get("lon"), "lon"),
        elevation_m=_optional_number(item.get("elev"), "elev") if "elev" in item else None,
        wind_speed_mps=_knots(item.get("wspd")),
        wind_direction_deg=_direction(item.get("wdir")),
        wind_gust_mps=_knots(item.get("wgst")),
        temperature_c=_optional_number(item.get("temp"), "temp"),
        pressure_pa=None if pressure is None else pressure * HPA_TO_PA,
        relative_humidity_pct=None,
    )


def fetch_awc_metar(
    station: str,
    *,
    urlopen_fn: Callable[..., object] | None = None,
    timeout_s: float = 10.0,
) -> WeatherFetch:
    """HTTP GET of one station. ``SeedingSupervisor`` does not call this.

    ``urlopen_fn`` is the transport, so tests can supply a fake response.
    A live call is not part of the unit tests.
    """
    if len(station) != 4 or not station.isalnum():
        return WeatherFetch(None, Reason.MALFORMED_RECORD, "station id")
    url = f"{AWC_METAR_URL}?ids={station}&format=json"
    transport = urlopen_fn or _open
    try:
        with transport(url, timeout=timeout_s) as response:
            status = int(getattr(response, "status", 200))
            body = response.read()
    except (OSError, TimeoutError, ValueError) as exc:
        return WeatherFetch(None, Reason.PROVIDER_FAILURE, type(exc).__name__)
    if status == 204 or body in (b"", None):
        return WeatherFetch(None, Reason.MISSING_OBSERVATION, "no data")
    if status != 200:
        return WeatherFetch(None, Reason.PROVIDER_FAILURE, f"http {status}")
    try:
        payload = json.loads(body.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError):
        return WeatherFetch(None, Reason.MALFORMED_RECORD, "body")
    if payload == []:
        return WeatherFetch(None, Reason.MISSING_OBSERVATION, "no data")
    try:
        observation = observation_from_awc_metar(payload, provenance=Provenance.LIVE)
    except WeatherParseError as exc:
        return WeatherFetch(None, exc.reason_code, str(exc))
    return WeatherFetch(observation, "")


def _open(url: str, timeout: float = 10.0):
    request = Request(url, headers={"User-Agent": USER_AGENT, "Accept": "application/json"})
    return urlopen(request, timeout=timeout)


def _as_segment(item: tuple[float, WeatherObservation | None] | _Segment) -> _Segment:
    if isinstance(item, _Segment):
        return item
    start_s, observation = item
    return _Segment(start_s, observation)


def failure_segment(start_s: float, detail: str) -> _Segment:
    return _Segment(start_s, failure=detail)


def _metar_object(payload: object) -> dict:
    if isinstance(payload, list):
        if len(payload) != 1:
            raise WeatherParseError(Reason.MALFORMED_RECORD, "expected one METAR object")
        payload = payload[0]
    if not isinstance(payload, dict):
        raise WeatherParseError(Reason.MALFORMED_RECORD, "METAR object")
    return payload


def _knots(value: object) -> float | None:
    number = _optional_number(value, "wind")
    if number is None:
        return None
    return number * KNOT_TO_MPS


def _direction(value: object) -> float | None:
    if value is None:
        return None
    if isinstance(value, str):
        if value.upper() == "VRB":
            return None
        raise WeatherParseError(Reason.MALFORMED_RECORD, "wind direction")
    return _optional_number(value, "wind direction")


def _optional_number(value: object, field: str) -> float | None:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise WeatherParseError(Reason.MALFORMED_RECORD, field)
    return float(value)


def _receipt_unix(value: object) -> float | None:
    if value is None:
        return None
    if not isinstance(value, str):
        raise WeatherParseError(Reason.MALFORMED_RECORD, "receiptTime")
    text = value.strip()
    if text.endswith("Z"):
        text = text[:-1]
    for fmt in ("%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S"):
        try:
            parsed = datetime.strptime(text, fmt).replace(tzinfo=timezone.utc)
        except ValueError:
            continue
        return parsed.timestamp()
    return None
