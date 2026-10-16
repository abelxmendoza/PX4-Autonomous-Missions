"""Release authorization for a seeding mission.

The mission controller still flies climb, hold, dispense, and return in the
local frame. This supervisor reads altitude, weather, airspace, traffic, and
simulated communications. The dispenser opens only when all five authorize it
and the mission itself is in the release phase. The primary denial is the
first gate in that order. A denial closes the valve and uses the mission's
existing return and abort path. ``evidence`` keeps each decision.
``operational_approval`` stays false. A simulated clearance is not an ATC
authorization, and a traffic decision is not a collision-avoidance command.
"""

from __future__ import annotations

import math
from typing import Sequence

from .airspace import AirspaceAuthorizer
from .altitude_reference import (
    AltitudeBand,
    AltitudeDecision,
    AltitudeMonitor,
    Px4AltitudeSnapshot,
)
from .simulated_comms import CommsAuthorizer
from .seeding_mission import SeedingCommand, SeedingMission, SeedingProfile
from .traffic import TrafficAuthorizer
from .weather import WeatherAuthorizer


class SeedingSupervisor:
    def __init__(
        self,
        mission: SeedingMission,
        monitor: AltitudeMonitor,
        band: AltitudeBand,
        weather: WeatherAuthorizer,
        airspace: AirspaceAuthorizer,
        traffic: TrafficAuthorizer,
        comms: CommsAuthorizer,
    ) -> None:
        self.mission = mission
        self.monitor = monitor
        self.band = band
        self.weather = weather
        self.airspace = airspace
        self.traffic = traffic
        self.comms = comms
        self.decisions: list[AltitudeDecision] = []
        self.weather_decisions: list[dict] = []
        self.airspace_decisions: list[dict] = []
        self.traffic_decisions: list[dict] = []
        self.comms_decisions: list[dict] = []

    def step(
        self,
        position_ned: Sequence[float],
        time_s: float,
        snapshot: Px4AltitudeSnapshot,
        *,
        landed: bool = False,
        dt_s: float = 0.0,
        velocity_ned: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> SeedingCommand:
        report = self.monitor.update(snapshot, time_s)
        altitude = self.band.decide(report)
        release_amsl = release_amsl_m(self.mission.profile, snapshot)
        weather = self.weather.evaluate(time_s, release_amsl)
        release_north = self.mission.profile.release_ned[0]
        release_east = self.mission.profile.release_ned[1]
        airspace = self.airspace.evaluate(time_s, release_north, release_east, release_amsl)
        north, east, down = position_ned
        traffic = self.traffic.evaluate(time_s, north, east, down, velocity_ned, snapshot.ref_alt_m)
        comms = self.comms.evaluate(time_s, release_north, release_east, release_amsl)
        if not altitude.authorized:
            authorized = False
            denial = altitude.reason
        elif not weather.authorized:
            authorized = False
            denial = weather.reason_code
        elif not airspace.authorized:
            authorized = False
            denial = airspace.reason_code
        elif not traffic.authorized:
            authorized = False
            denial = traffic.reason_code
        elif not comms.authorized:
            authorized = False
            denial = comms.reason_code
        else:
            authorized = True
            denial = ""
        command = self.mission.step(
            position_ned,
            time_s,
            landed=landed,
            dt_s=dt_s,
            release_authorized=authorized,
            release_denial=denial,
        )
        self.decisions.append(altitude)
        gates = (
            altitude.authorized
            and weather.authorized
            and airspace.authorized
            and traffic.authorized
            and comms.authorized
        )
        context = {
            "altitude_authorized": altitude.authorized,
            "weather_authorized": weather.authorized,
            "airspace_authorized": airspace.authorized,
            "traffic_authorized": traffic.authorized,
            "communications_authorized": comms.authorized,
            "gates_authorized": gates,
            "dispenser_open": command.dispense,
            "mission_phase": command.phase.value,
            "primary_reason": denial,
        }
        record = weather.as_dict()
        record.update(context)
        self.weather_decisions.append(record)
        airspace_record = airspace.as_dict()
        airspace_record.update(context)
        self.airspace_decisions.append(airspace_record)
        traffic_record = traffic.as_dict()
        traffic_record.update(context)
        self.traffic_decisions.append(traffic_record)
        comms_record = comms.as_dict()
        comms_record.update(context)
        self.comms_decisions.append(comms_record)
        return command

    def evidence(self) -> dict:
        return {
            "schema": 4,
            "kind": "seeding_release_authorization",
            "operational_approval": False,
            "frame": self.band.frame.value,
            "floor_m": self.band.floor_m,
            "ceiling_m": self.band.ceiling_m,
            "agreement_tolerance_m": self.band.agreement_tolerance_m,
            "decisions": [decision.as_dict() for decision in self.decisions],
            "weather_decisions": self.weather_decisions,
            "airspace_decisions": self.airspace_decisions,
            "traffic_decisions": self.traffic_decisions,
            "traffic_snapshots": _latest_snapshots(self.traffic_decisions),
            "traffic_conflicts": _bounded_conflicts(self.traffic_decisions),
            "traffic_source_metadata": _traffic_source(self.traffic_decisions),
            "communications_events": self.comms.broker.events(),
            "communications_decisions": self.comms_decisions,
            "simulated_clearance_records": _clearance_records(self.comms_decisions),
            "authorization_summary": _authorization_summary(self),
            "mission_outcome": self.mission.report(),
            "mission": self.mission.report(),
            "non_operational_communications": True,
            "surveillance_complete": False,
            "collision_avoidance": False,
        }


def release_amsl_m(profile: SeedingProfile, snapshot: Px4AltitudeSnapshot) -> float | None:
    """AMSL of the release point: local-origin AMSL plus the release height.

    The vehicle's current GPS altitude is a different number while it is still
    climbing, so this does not read ``gps_alt_amsl_m``.
    """
    if snapshot.ref_alt_m is None or not math.isfinite(snapshot.ref_alt_m):
        return None
    return snapshot.ref_alt_m + (-profile.release_ned[2])


def _latest_snapshots(records: list[dict]) -> list[dict]:
    for record in reversed(records):
        snapshot = record.get("snapshot")
        if snapshot is not None:
            return [snapshot]
    return []


def _bounded_conflicts(records: list[dict], limit: int = 256) -> list[dict]:
    found: list[dict] = []
    for record in records:
        for conflict in record.get("conflicts") or ():
            found.append(conflict)
    return found[-limit:]


def _traffic_source(records: list[dict]) -> dict:
    snapshot = _latest_snapshots(records)
    if not snapshot:
        return {"source": None, "provenance": None, "source_quality": None, "surveillance_complete": False}
    item = snapshot[0]
    return {
        "source": item.get("source"),
        "provenance": item.get("provenance"),
        "source_quality": item.get("source_quality"),
        "surveillance_complete": False,
    }


def _clearance_records(records: list[dict]) -> list[dict]:
    found: list[dict] = []
    seen: set[str] = set()
    for record in records:
        clearance = record.get("clearance")
        if not clearance:
            continue
        message_id = clearance.get("message_id")
        if message_id in seen:
            continue
        seen.add(message_id)
        found.append(clearance)
    return found


def _authorization_summary(supervisor: SeedingSupervisor) -> dict:
    last_traffic = supervisor.traffic_decisions[-1] if supervisor.traffic_decisions else {}
    last_comms = supervisor.comms_decisions[-1] if supervisor.comms_decisions else {}
    last_weather = supervisor.weather_decisions[-1] if supervisor.weather_decisions else {}
    last_airspace = supervisor.airspace_decisions[-1] if supervisor.airspace_decisions else {}
    last_altitude = supervisor.decisions[-1].as_dict() if supervisor.decisions else {}
    return {
        "altitude": {"authorized": last_altitude.get("authorized"), "reason_code": last_altitude.get("reason")},
        "weather": {"authorized": last_weather.get("authorized"), "reason_code": last_weather.get("reason_code")},
        "airspace": {"authorized": last_airspace.get("authorized"), "reason_code": last_airspace.get("reason_code")},
        "traffic": {
            "authorized": last_traffic.get("authorized"),
            "reason_code": last_traffic.get("reason_code"),
            "conflict_status": last_traffic.get("conflict_status"),
            "surveillance_complete": False,
        },
        "simulated_communications": {
            "authorized": last_comms.get("authorized"),
            "reason_code": last_comms.get("reason_code"),
            "non_operational": True,
            "operational_approval": False,
        },
        "gates_authorized": bool(last_weather.get("gates_authorized")),
        "primary_reason": last_weather.get("primary_reason", ""),
        "operational_approval": False,
    }
