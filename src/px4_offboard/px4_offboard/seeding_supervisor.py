"""Release authorization for a seeding mission.

The mission controller still flies climb, hold, dispense, and return in the
local frame. This supervisor reads the altitude monitor, the weather
authorizer, and the airspace authorizer. The dispenser opens only when all
three authorize it and the mission itself is in the release phase. An altitude
denial is kept ahead of a weather denial, and both are kept ahead of an
airspace denial. A denial closes the valve and uses the mission's existing
return and abort path. ``evidence`` keeps each decision.
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
from .seeding_mission import SeedingCommand, SeedingMission, SeedingProfile
from .weather import WeatherAuthorizer


class SeedingSupervisor:
    def __init__(
        self,
        mission: SeedingMission,
        monitor: AltitudeMonitor,
        band: AltitudeBand,
        weather: WeatherAuthorizer,
        airspace: AirspaceAuthorizer,
    ) -> None:
        self.mission = mission
        self.monitor = monitor
        self.band = band
        self.weather = weather
        self.airspace = airspace
        self.decisions: list[AltitudeDecision] = []
        self.weather_decisions: list[dict] = []
        self.airspace_decisions: list[dict] = []

    def step(
        self,
        position_ned: Sequence[float],
        time_s: float,
        snapshot: Px4AltitudeSnapshot,
        *,
        landed: bool = False,
        dt_s: float = 0.0,
    ) -> SeedingCommand:
        report = self.monitor.update(snapshot, time_s)
        altitude = self.band.decide(report)
        release_amsl = release_amsl_m(self.mission.profile, snapshot)
        weather = self.weather.evaluate(time_s, release_amsl)
        airspace = self.airspace.evaluate(
            time_s,
            self.mission.profile.release_ned[0],
            self.mission.profile.release_ned[1],
            release_amsl,
        )
        if not altitude.authorized:
            authorized = False
            denial = altitude.reason
        elif not weather.authorized:
            authorized = False
            denial = weather.reason_code
        elif not airspace.authorized:
            authorized = False
            denial = airspace.reason_code
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
        gates = altitude.authorized and weather.authorized and airspace.authorized
        record = weather.as_dict()
        record["altitude_authorized"] = altitude.authorized
        record["airspace_authorized"] = airspace.authorized
        record["gates_authorized"] = gates
        record["dispenser_open"] = command.dispense
        record["mission_phase"] = command.phase.value
        self.weather_decisions.append(record)
        airspace_record = airspace.as_dict()
        airspace_record["altitude_authorized"] = altitude.authorized
        airspace_record["weather_authorized"] = weather.authorized
        airspace_record["gates_authorized"] = gates
        airspace_record["dispenser_open"] = command.dispense
        airspace_record["mission_phase"] = command.phase.value
        self.airspace_decisions.append(airspace_record)
        return command

    def evidence(self) -> dict:
        return {
            "schema": 3,
            "kind": "seeding_release_authorization",
            "operational_approval": False,
            "frame": self.band.frame.value,
            "floor_m": self.band.floor_m,
            "ceiling_m": self.band.ceiling_m,
            "agreement_tolerance_m": self.band.agreement_tolerance_m,
            "decisions": [decision.as_dict() for decision in self.decisions],
            "weather_decisions": self.weather_decisions,
            "airspace_decisions": self.airspace_decisions,
            "mission": self.mission.report(),
        }


def release_amsl_m(profile: SeedingProfile, snapshot: Px4AltitudeSnapshot) -> float | None:
    """AMSL of the release point: local-origin AMSL plus the release height.

    The vehicle's current GPS altitude is a different number while it is still
    climbing, so this does not read ``gps_alt_amsl_m``.
    """
    if snapshot.ref_alt_m is None or not math.isfinite(snapshot.ref_alt_m):
        return None
    return snapshot.ref_alt_m + (-profile.release_ned[2])
