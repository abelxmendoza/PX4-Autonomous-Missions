"""Climb, hold, dispense, and return. Release is legal only inside the volume."""

from __future__ import annotations

import math

import pytest

from px4_offboard.seeding_mission import (
    RELEASE_ALTITUDE_15000_FT_M,
    CommandMode,
    ReleaseSample,
    ReleaseVolume,
    SeedingMission,
    SeedingPhase,
    SeedingProfile,
    profile_at_15000_ft,
    release_violations,
    sim_profile,
)


def _toward(position, target, speed, dt):
    dist = math.dist(position, target)
    if dist <= speed * dt:
        return [target[0], target[1], target[2]]
    scale = speed * dt / dist
    return [position[i] + (target[i] - position[i]) * scale for i in range(3)]


def _fly(profile, speed=5.0, dt=0.1, max_ticks=600):
    mission = SeedingMission(profile)
    position = list(profile.home_ned)
    landed = True
    seen = []
    time_s = 0.0
    for _ in range(max_ticks):
        command = mission.step(tuple(position), time_s, landed=landed, dt_s=dt)
        seen.append((command, tuple(position)))
        if command.phase in (SeedingPhase.COMPLETE, SeedingPhase.ABORT):
            return mission, seen
        landed = False
        if command.mode is CommandMode.LAND:
            position[2] = min(0.0, position[2] + speed * dt)
            if position[2] >= -0.05:
                position[2] = 0.0
                landed = True
        else:
            position[:] = _toward(position, command.position_ned, speed, dt)
        time_s += dt
    raise AssertionError(f"mission did not finish: {mission.report()}")


def test_sim_profile_climbs_holds_dispenses_and_returns():
    profile = sim_profile()
    mission, seen = _fly(profile)

    phases = []
    for command, position in seen:
        if not phases or phases[-1] is not command.phase:
            phases.append(command.phase)
        if command.dispense:
            assert command.phase is SeedingPhase.DISPENSE
            assert profile.volume.contains(position)
            assert -position[2] >= profile.volume.altitude_floor_m

    assert phases == [
        SeedingPhase.CLIMB,
        SeedingPhase.HOLD,
        SeedingPhase.DISPENSE,
        SeedingPhase.RETURN,
        SeedingPhase.COMPLETE,
    ]
    report = mission.report()
    assert report["reason"] == ""
    assert report["violations"] == []
    assert report["release_s"] == pytest.approx(profile.dispense_s)
    assert report["amount_kg"] == pytest.approx(profile.rate_kg_s * profile.dispense_s)
    assert report["sample_count"] > 0
    assert math.dist(seen[-1][1], profile.home_ned) <= profile.arrival_m


def test_climb_through_the_volume_does_not_dispense():
    profile = sim_profile()
    mission = SeedingMission(profile)
    # On the line from the pad to the release point, 12 m up is already inside the box.
    fraction = 12.0 / 15.0
    inside = tuple(fraction * v for v in profile.release_ned)
    assert profile.volume.contains(inside)
    command = mission.step(inside, 0.0, dt_s=0.1)
    assert command.phase is SeedingPhase.CLIMB
    assert command.dispense is False
    assert mission.samples == ()


def test_a_point_below_the_floor_is_not_a_release():
    profile = sim_profile()
    below = (30.0, 10.0, -11.0)
    assert not profile.volume.contains(below)
    command = SeedingMission(profile).step(below, 0.0, dt_s=0.1)
    assert command.dispense is False
    assert command.phase is SeedingPhase.CLIMB


def test_hold_restarts_if_the_vehicle_leaves_the_release_point():
    profile = sim_profile()
    mission = SeedingMission(profile)
    release = profile.release_ned
    command = mission.step(release, 0.0, dt_s=0.1)
    assert command.phase is SeedingPhase.HOLD

    command = mission.step(release, 1.0, dt_s=0.1)
    assert command.phase is SeedingPhase.HOLD

    command = mission.step((0.0, 0.0, release[2]), 1.1, dt_s=0.1)
    assert command.phase is SeedingPhase.CLIMB
    assert command.dispense is False

    command = mission.step(release, 1.2, dt_s=0.1)
    assert command.phase is SeedingPhase.HOLD
    command = mission.step(release, 3.1, dt_s=0.1)
    assert command.phase is SeedingPhase.HOLD
    command = mission.step(release, 3.2, dt_s=0.1)
    assert command.phase is SeedingPhase.DISPENSE
    assert command.dispense is True


def test_leaving_the_volume_closes_the_dispenser_and_aborts_after_landing():
    profile = sim_profile()
    mission = SeedingMission(profile)
    release = profile.release_ned
    mission.step(release, 0.0, dt_s=0.1)
    command = mission.step(release, profile.hold_s, dt_s=0.1)
    assert command.dispense is True
    assert mission.samples

    command = mission.step((0.0, 0.0, release[2]), profile.hold_s + 0.1, dt_s=0.1)
    assert command.phase is SeedingPhase.RETURN
    assert command.dispense is False
    assert command.position_ned == profile.home_ned
    assert mission.reason == "left release volume"
    assert release_violations(mission.samples, profile.volume) == []
    assert mission.report()["amount_kg"] < profile.rate_kg_s * profile.dispense_s

    command = mission.step(profile.home_ned, profile.hold_s + 0.2, landed=True, dt_s=0.1)
    assert command.phase is SeedingPhase.ABORT
    assert command.dispense is False
    assert mission.report()["violations"] == []


def test_timeout_before_release_returns_home_with_the_valve_closed():
    profile = sim_profile()
    mission = SeedingMission(profile)
    command = mission.step(profile.home_ned, 0.0, landed=True, dt_s=0.1)
    assert command.phase is SeedingPhase.CLIMB
    assert command.dispense is False

    command = mission.step(profile.home_ned, profile.timeout_s, landed=True, dt_s=0.1)
    assert command.phase is SeedingPhase.ABORT
    assert command.dispense is False
    assert mission.reason == "mission timeout"
    assert mission.samples == ()


def test_release_check_rejects_samples_outside_the_volume():
    volume = sim_profile().volume
    below = ReleaseSample(1.0, 30.0, 10.0, -10.0, 10.0, 0.01)
    above = ReleaseSample(2.0, 30.0, 10.0, -20.0, 20.0, 0.01)
    aside = ReleaseSample(3.0, 0.0, 0.0, -15.0, 15.0, 0.01)
    mismatched = ReleaseSample(4.0, 30.0, 10.0, -15.0, 10.0, 0.01)

    assert "floor" in release_violations([below], volume)[0]
    assert "ceiling" in release_violations([above], volume)[0]
    assert "release area" in release_violations([aside], volume)[0]
    assert "does not match" in release_violations([mismatched], volume)[0]

    legal = ReleaseSample(1.0, 30.0, 10.0, -15.0, 15.0, 0.02)
    assert release_violations([legal], volume) == []


def test_a_denied_release_closes_the_dispenser_without_a_sample():
    profile = sim_profile()
    mission = SeedingMission(profile)
    release = profile.release_ned
    mission.step(release, 0.0, dt_s=0.1)
    command = mission.step(
        release,
        profile.hold_s,
        dt_s=0.1,
        release_authorized=False,
        release_denial="barometric altitude is stale",
    )
    assert command.dispense is False
    assert command.phase is SeedingPhase.RETURN
    assert mission.reason == "barometric altitude is stale"
    assert mission.samples == ()


def test_profile_rejects_a_release_point_outside_the_volume():
    volume = sim_profile().volume
    with pytest.raises(ValueError, match="outside the release volume"):
        SeedingProfile(
            home_ned=(0.0, 0.0, 0.0),
            release_ned=(0.0, 0.0, -15.0),
            volume=volume,
            hold_s=1.0,
            dispense_s=1.0,
            rate_kg_s=0.01,
            arrival_m=0.5,
            timeout_s=30.0,
        )


def test_profile_rejects_a_floor_at_the_ground():
    with pytest.raises(ValueError, match="altitude_floor_m"):
        ReleaseVolume(0.0, 10.0, 0.0, 10.0, 0.0, 20.0)


def test_15000_ft_profile_holds_the_release_at_that_altitude():
    profile = profile_at_15000_ft()
    assert -profile.release_ned[2] == pytest.approx(RELEASE_ALTITUDE_15000_FT_M)
    assert profile.volume.contains(profile.release_ned)
    assert not profile.volume.contains((0.0, 0.0, -15.0))

    command = SeedingMission(profile).step((0.0, 0.0, 0.0), 0.0, landed=True, dt_s=0.1)
    assert command.phase is SeedingPhase.CLIMB
    assert command.position_ned[2] == pytest.approx(-RELEASE_ALTITUDE_15000_FT_M)
    assert command.dispense is False
