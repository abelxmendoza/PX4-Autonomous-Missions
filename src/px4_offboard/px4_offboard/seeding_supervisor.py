"""Release authorization for a seeding mission.

The mission controller still flies climb, hold, dispense, and return in the
local frame. This supervisor reads the altitude monitor and the weather
authorizer. The dispenser opens only when both authorize it and the mission
itself is in the release phase. An altitude denial is kept even when the
weather decision would allow the release. A denial closes the valve and uses
the mission's existing return and abort path. ``evidence`` keeps the altitude
records and adds the weather records.
"""

from __future__ import annotations

import math
from typing import Sequence

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
    ) -> None:
        self.mission = mission
        self.monitor = monitor
        self.band = band
        self.weather = weather
        self.decisions: list[AltitudeDecision] = []
        self.weather_decisions: list[dict] = []

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
        weather = self.weather.evaluate(time_s, release_amsl_m(self.mission.profile, snapshot))
        if not altitude.authorized:
            authorized = False
            denial = altitude.reason
        elif not weather.authorized:
            authorized = False
            denial = weather.reason_code
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
        record = weather.as_dict()
        record["altitude_authorized"] = altitude.authorized
        record["gates_authorized"] = altitude.authorized and weather.authorized
        record["dispenser_open"] = command.dispense
        record["mission_phase"] = command.phase.value
        self.weather_decisions.append(record)
        return command

    def evidence(self) -> dict:
        return {
            "schema": 2,
            "kind": "seeding_release_authorization",
            "operational_approval": False,
            "frame": self.band.frame.value,
            "floor_m": self.band.floor_m,
            "ceiling_m": self.band.ceiling_m,
            "agreement_tolerance_m": self.band.agreement_tolerance_m,
            "decisions": [decision.as_dict() for decision in self.decisions],
            "weather_decisions": self.weather_decisions,
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
