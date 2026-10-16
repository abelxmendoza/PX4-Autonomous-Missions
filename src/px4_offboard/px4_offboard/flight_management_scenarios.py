"""Kinematic release scenarios. No PX4 process and no network access.

These exercise the supervisor gates. A passing run is not a SITL flight and
not an approval to operate.
"""

from __future__ import annotations

import math

from .airspace import (
    AirspaceAuthorizer,
    AirspaceKind,
    AirspaceVolume,
    SimulatedAirspaceProvider,
    simulation_authorizer,
    simulation_dataset,
)
from .altitude_reference import AltitudeBand, AltitudeFrame, AltitudeMonitor, Px4AltitudeSnapshot
from .simulated_comms import SimCommsBroker, simulation_clearance, simulation_comms_authorizer
from .mission_orchestrator import MissionOrchestrator, MissionState, OrchestratorConfig, VehicleInput
from .precision_land import CameraTag, PrecisionLand, PrecisionLandParams
from .seeding_mission import CommandMode, SeedingMission, SeedingPhase, sim_profile
from .seeding_supervisor import SeedingSupervisor
from .traffic import simulation_snapshot, simulation_traffic_authorizer, simulation_traffic_policy, track_at_ned
from .traffic_provider import SimulatedTrafficProvider, StaticTrafficProvider
from .weather import WeatherAuthorizer, simulation_observation, simulation_policy
from .weather_provider import StaticWeatherProvider


PX4_LAND_SPEED_MPS = 0.7  # PX4 default MPC_LAND_SPEED


def scenario_all_gates_pass() -> dict:
    return _fly(_supervisor())


def scenario_altitude_fails() -> dict:
    profile = sim_profile()
    supervisor = _supervisor()
    release = profile.release_ned
    supervisor.step(release, 0.0, _snapshot(0.0, release, local_z_m=-10.0, baro_alt_m=10.0), dt_s=0.1)
    supervisor.step(release, profile.hold_s, _snapshot(profile.hold_s, release, local_z_m=-10.0, baro_alt_m=10.0), dt_s=0.1)
    return supervisor.evidence()


def scenario_weather_fails() -> dict:
    windy = simulation_observation(wind_speed_mps=20.0, wind_gust_mps=25.0)
    return _denied_at_release(_supervisor(observation=windy))


def scenario_airspace_fails() -> dict:
    volume = _prohibited()
    return _denied_at_release(_supervisor(airspace=simulation_authorizer(simulation_dataset(volumes=(volume,)))))


def scenario_airspace_during_release() -> dict:
    profile = sim_profile()
    clear = simulation_dataset()
    blocked = simulation_dataset(volumes=(_prohibited(),))
    provider = SimulatedAirspaceProvider([(0.0, clear), (profile.hold_s + 0.05, blocked), (profile.hold_s + 1.0, clear)])
    supervisor = _supervisor(airspace=AirspaceAuthorizer(provider))
    return _open_then_close(supervisor)


def scenario_stale_authorization() -> dict:
    profile = sim_profile()
    traffic = simulation_traffic_authorizer(policy=simulation_traffic_policy(max_age_s=5.0))
    supervisor = _supervisor(traffic=traffic)
    release = profile.release_ned
    supervisor.step(release, 0.0, _snapshot(0.0, release), dt_s=0.1)
    supervisor.step(release, 6.0, _snapshot(6.0, release), dt_s=0.1)
    return supervisor.evidence()


def scenario_converging_before_release() -> dict:
    track = track_at_ned(230.0, 10.0, 15.0, ground_speed_mps=20.0, heading_deg=180.0)
    traffic = simulation_traffic_authorizer(simulation_snapshot(tracks=(track,)))
    return _denied_at_release(_supervisor(traffic=traffic))


def scenario_conflict_during_dispense() -> dict:
    profile = sim_profile()
    clear = simulation_snapshot()
    blocked = simulation_snapshot(tracks=(track_at_ned(30.0, 10.0, 15.0, track_id="AC-NEAR"),))
    provider = SimulatedTrafficProvider([(0.0, clear), (profile.hold_s + 0.05, blocked), (profile.hold_s + 1.0, clear)])
    supervisor = _supervisor(traffic=simulation_traffic_authorizer_from(provider))
    return _open_then_close(supervisor)


def scenario_clearance_expires_before_release() -> dict:
    comms = simulation_comms_authorizer(clearance=simulation_clearance(expires_at_s=1.0, message_id="CLR-EARLY"))
    return _denied_at_release(_supervisor(comms=comms))


def scenario_clearance_expires_during_dispense() -> dict:
    profile = sim_profile()
    comms = simulation_comms_authorizer(
        clearance=simulation_clearance(expires_at_s=profile.hold_s + 0.05, message_id="CLR-MID")
    )
    return _open_then_close(_supervisor(comms=comms))


def scenario_altitude_outranks_traffic() -> dict:
    profile = sim_profile()
    track = track_at_ned(30.0, 10.0, 15.0, track_id="AC-LOW")
    supervisor = _supervisor(traffic=simulation_traffic_authorizer(simulation_snapshot(tracks=(track,))))
    release = profile.release_ned
    supervisor.step(release, 0.0, _snapshot(0.0, release, local_z_m=-10.0, baro_alt_m=10.0), dt_s=0.1)
    supervisor.step(release, profile.hold_s, _snapshot(profile.hold_s, release, local_z_m=-10.0, baro_alt_m=10.0), dt_s=0.1)
    return supervisor.evidence()


def scenario_airspace_outranks_comms() -> dict:
    broker = SimCommsBroker()
    supervisor = _supervisor(
        airspace=simulation_authorizer(simulation_dataset(volumes=(_prohibited(),))),
        comms=simulation_comms_authorizer(broker=broker),
    )
    return _denied_at_release(supervisor)


def scenario_traffic_provider_unavailable() -> dict:
    traffic = simulation_traffic_authorizer_from(StaticTrafficProvider(failure="down"))
    return _denied_at_release(_supervisor(traffic=traffic))


def scenario_nominal_precision_landing() -> dict:
    """Kinematic orchestrator loop. This does not start PX4."""
    profile = sim_profile()
    orchestrator = MissionOrchestrator(
        _supervisor(),
        PrecisionLand(PrecisionLandParams(delta_velocity=0.5)),
        OrchestratorConfig(),
    )
    position = list(profile.home_ned)
    velocity = (0.0, 0.0, 0.0)
    landed = False
    orchestrator.start(0.0)
    time_s = 0.0
    dt = 0.1
    for _ in range(800):
        tag = None
        if orchestrator.state is MissionState.PRECISION_LANDING:
            tag = CameraTag((0.0, 0.0, max(0.2, -position[2] - 0.05)), time_s=time_s)
        output = orchestrator.step(
            VehicleInput(
                time_s=time_s,
                position_ned=tuple(position),
                velocity_ned=velocity,
                attitude_wxyz=(1.0, 0.0, 0.0, 0.0),
                landed=landed,
                armed=True,
                offboard=True,
                altitude=_snapshot(time_s, position),
                tag=tag,
                dt_s=dt,
            )
        )
        if output.state in (MissionState.LANDED, MissionState.ABORTED, MissionState.FAILED):
            return orchestrator.evidence()
        landed = False
        if output.use_position and output.position_ned is not None:
            position, velocity = _toward(position, output.position_ned, 8.0, dt)
            if math.dist(position, output.position_ned) < 0.05:
                velocity = (0.0, 0.0, 0.0)
        elif output.request_nav_land:
            # PX4 land mode (requested on pad contact): straight down at MPC_LAND_SPEED.
            velocity = (0.0, 0.0, PX4_LAND_SPEED_MPS)
            position = [position[0], position[1], position[2] + PX4_LAND_SPEED_MPS * dt]
        elif output.velocity_ned is not None and all(math.isfinite(item) for item in output.velocity_ned):
            velocity = output.velocity_ned
            position = [position[index] + velocity[index] * dt for index in range(3)]
        if position[2] >= -0.05:
            position[2] = 0.0
            landed = True
            velocity = (0.0, 0.0, 0.0)
        time_s += dt
    raise RuntimeError(orchestrator.state.value)


def simulation_traffic_authorizer_from(provider) -> object:
    from .traffic import TrafficAuthorizer

    return TrafficAuthorizer(provider, simulation_traffic_policy())


def _prohibited() -> AirspaceVolume:
    return AirspaceVolume("P-1", AirspaceKind.PROHIBITED, 46.9, 47.1, -122.2, -121.8, 0.0, 20_000.0)


def _supervisor(observation=None, airspace=None, traffic=None, comms=None) -> SeedingSupervisor:
    profile = sim_profile()
    return SeedingSupervisor(
        SeedingMission(profile),
        AltitudeMonitor(max_age_s=0.5),
        AltitudeBand(AltitudeFrame.LOCAL, profile.volume.altitude_floor_m, profile.volume.altitude_ceiling_m),
        WeatherAuthorizer(StaticWeatherProvider(observation or simulation_observation()), simulation_policy()),
        airspace or simulation_authorizer(),
        traffic or simulation_traffic_authorizer(),
        comms or simulation_comms_authorizer(),
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


def _denied_at_release(supervisor: SeedingSupervisor) -> dict:
    profile = sim_profile()
    release = profile.release_ned
    supervisor.step(release, 0.0, _snapshot(0.0, release), dt_s=0.1)
    supervisor.step(release, profile.hold_s, _snapshot(profile.hold_s, release), dt_s=0.1)
    return supervisor.evidence()


def _open_then_close(supervisor: SeedingSupervisor) -> dict:
    profile = sim_profile()
    release = profile.release_ned
    supervisor.step(release, 0.0, _snapshot(0.0, release), dt_s=0.1)
    supervisor.step(release, profile.hold_s, _snapshot(profile.hold_s, release), dt_s=0.1)
    supervisor.step(release, profile.hold_s + 0.1, _snapshot(profile.hold_s + 0.1, release), dt_s=0.1)
    supervisor.step(release, profile.hold_s + 1.0, _snapshot(profile.hold_s + 1.0, release), dt_s=0.1)
    return supervisor.evidence()


def _fly(supervisor: SeedingSupervisor) -> dict:
    profile = supervisor.mission.profile
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
            position[:] = _move(position, command.position_ned, 5.0, 0.1)
        time_s = round(time_s + 0.1, 10)
    raise RuntimeError(supervisor.evidence()["mission"])


def _move(position, target, speed, dt):
    dist = math.dist(position, target)
    if dist <= speed * dt:
        return [target[0], target[1], target[2]]
    scale = speed * dt / dist
    return [position[i] + (target[i] - position[i]) * scale for i in range(3)]


def _toward(position, target, speed, dt):
    dist = math.dist(position, target)
    if dist <= speed * dt or dist == 0.0:
        return [target[0], target[1], target[2]], (0.0, 0.0, 0.0)
    scale = speed * dt / dist
    new = [position[i] + (target[i] - position[i]) * scale for i in range(3)]
    velocity = tuple((new[i] - position[i]) / dt for i in range(3))
    return new, velocity
