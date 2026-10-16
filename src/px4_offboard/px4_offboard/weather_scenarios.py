"""Kinematic weather scenarios for the seeding supervisor. No network access."""

from __future__ import annotations

import math

from .airspace import simulation_authorizer
from .simulated_comms import simulation_comms_authorizer
from .traffic import simulation_traffic_authorizer
from .altitude_reference import AltitudeBand, AltitudeFrame, AltitudeMonitor, Px4AltitudeSnapshot
from .seeding_mission import CommandMode, SeedingMission, SeedingPhase, profile_at_15000_ft, sim_profile
from .seeding_supervisor import SeedingSupervisor
from .weather import WeatherAuthorizer, simulation_observation, simulation_policy
from .weather_provider import SimulatedWeatherProvider, StaticWeatherProvider


def scenario_nominal() -> dict:
    """Favorable simulated weather. The sortie completes and the valve opens."""
    return _fly(sim_profile(), StaticWeatherProvider(simulation_observation()), simulation_policy())


def scenario_high_wind_before_release() -> dict:
    """Wind is already over the simulation limit when the hold ends."""
    profile = sim_profile()
    windy = simulation_observation(wind_speed_mps=20.0, wind_gust_mps=25.0)
    supervisor = _supervisor(profile, StaticWeatherProvider(windy), simulation_policy())
    release = profile.release_ned
    supervisor.step(release, 0.0, _snapshot(0.0, release), dt_s=0.1)
    supervisor.step(release, profile.hold_s, _snapshot(profile.hold_s, release), dt_s=0.1)
    supervisor.step(profile.home_ned, profile.hold_s + 0.1, _snapshot(profile.hold_s + 0.1, profile.home_ned), landed=True, dt_s=0.1)
    return supervisor.evidence()


def scenario_weather_invalid_during_dispense() -> dict:
    """The wind limit is crossed after the first release sample."""
    profile = sim_profile()
    calm = simulation_observation()
    windy = simulation_observation(wind_speed_mps=20.0, wind_gust_mps=25.0)
    provider = SimulatedWeatherProvider([(0.0, calm), (profile.hold_s + 0.05, windy)])
    supervisor = _supervisor(profile, provider, simulation_policy())
    release = profile.release_ned
    supervisor.step(release, 0.0, _snapshot(0.0, release), dt_s=0.1)
    supervisor.step(release, profile.hold_s, _snapshot(profile.hold_s, release), dt_s=0.1)
    supervisor.step(release, profile.hold_s + 0.1, _snapshot(profile.hold_s + 0.1, release), dt_s=0.1)
    supervisor.step(
        profile.home_ned,
        profile.hold_s + 0.2,
        _snapshot(profile.hold_s + 0.2, profile.home_ned),
        landed=True,
        dt_s=0.1,
    )
    return supervisor.evidence()


def scenario_stale_feed() -> dict:
    """The observation is older than the policy allows by the time release would start."""
    profile = sim_profile()
    policy = simulation_policy(max_age_s=5.0)
    supervisor = _supervisor(profile, StaticWeatherProvider(simulation_observation(observed_at_s=0.0, received_at_s=0.0)), policy)
    release = profile.release_ned
    supervisor.step(release, 0.0, _snapshot(0.0, release), dt_s=0.1)
    supervisor.step(release, 6.0, _snapshot(6.0, release), dt_s=0.1)
    supervisor.step(profile.home_ned, 6.1, _snapshot(6.1, profile.home_ned), landed=True, dt_s=0.1)
    return supervisor.evidence()


def scenario_surface_weather_at_15000_ft() -> dict:
    """A surface report against the 15,000 ft release point."""
    profile = profile_at_15000_ft()
    supervisor = _supervisor(profile, StaticWeatherProvider(simulation_observation()), simulation_policy())
    release = profile.release_ned
    supervisor.step(release, 0.0, _snapshot(0.0, release), dt_s=0.1)
    supervisor.step(release, profile.hold_s, _snapshot(profile.hold_s, release), dt_s=0.1)
    supervisor.step(
        profile.home_ned,
        profile.hold_s + 0.1,
        _snapshot(profile.hold_s + 0.1, profile.home_ned),
        landed=True,
        dt_s=0.1,
    )
    return supervisor.evidence()


def _supervisor(profile, provider, policy) -> SeedingSupervisor:
    return SeedingSupervisor(
        SeedingMission(profile),
        AltitudeMonitor(max_age_s=0.5),
        AltitudeBand(AltitudeFrame.LOCAL, profile.volume.altitude_floor_m, profile.volume.altitude_ceiling_m),
        WeatherAuthorizer(provider, policy),
        simulation_authorizer(),
        simulation_traffic_authorizer(),
        simulation_comms_authorizer(),
    )


def _snapshot(time_s: float, position, **overrides) -> Px4AltitudeSnapshot:
    values = dict(
        time_s=time_s,
        baro_alt_m=-position[2],
        local_z_m=position[2],
        local_vz_mps=0.0,
        z_valid=True,
        vz_valid=True,
        ref_alt_m=0.0,
        baro_fault=False,
    )
    values.update(overrides)
    return Px4AltitudeSnapshot(**values)


def _fly(profile, provider, policy) -> dict:
    supervisor = _supervisor(profile, provider, policy)
    position = list(profile.home_ned)
    landed = True
    time_s = 0.0
    for _ in range(600):
        command = supervisor.step(tuple(position), time_s, _snapshot(time_s, position), landed=landed, dt_s=0.1)
        if command.phase in (SeedingPhase.COMPLETE, SeedingPhase.ABORT):
            return supervisor.evidence()
        landed = False
        if command.mode is CommandMode.LAND:
            position[2] = min(0.0, position[2] + 0.5)
            if position[2] >= -0.05:
                position[2] = 0.0
                landed = True
        else:
            position[:] = _toward(position, command.position_ned, 5.0, 0.1)
        time_s = round(time_s + 0.1, 10)
    raise RuntimeError(supervisor.evidence()["mission"])


def _toward(position, target, speed, dt):
    dist = math.dist(position, target)
    if dist <= speed * dt:
        return [target[0], target[1], target[2]]
    scale = speed * dt / dist
    return [position[i] + (target[i] - position[i]) * scale for i in range(3)]
