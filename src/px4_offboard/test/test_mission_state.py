import pytest

from px4_offboard.mission_state import (
    FailsafeInputs,
    InvalidTransition,
    MissionStateMachine,
    State,
    arming_complete,
    failsafe_reason,
    hover_complete,
    should_start_arming,
    takeoff_complete,
)


def nominal_inputs(**overrides):
    values = dict(
        state=State.MOVE,
        now_s=100.0,
        have_position=True,
        position_stamp_s=100.0,
        position_timeout_s=1.5,
        mission_elapsed_s=30.0,
        mission_timeout_s=180.0,
        avoidance_started_s=0.0,
        waypoint_progress_s=100.0,
        avoidance_stuck_s=12.0,
        geofence_enabled=True,
        inside_fence=True,
        north_m=10.0,
        east_m=2.0,
        down_m=-5.0,
    )
    values.update(overrides)
    return FailsafeInputs(**values)


def test_nominal_transition_sequence():
    machine = MissionStateMachine()

    for expected in (
        State.ARMING,
        State.TAKEOFF,
        State.HOVER,
        State.MOVE,
        State.LANDING,
    ):
        machine.transition(expected)
        assert machine.state is expected


def test_failsafe_is_reachable_from_each_active_flight_state():
    for state in (State.ARMING, State.TAKEOFF, State.HOVER, State.MOVE):
        machine = MissionStateMachine()
        path = [State.ARMING, State.TAKEOFF, State.HOVER, State.MOVE]
        for next_state in path[: path.index(state) + 1]:
            machine.transition(next_state)
        machine.transition(State.FAILSAFE)
        assert machine.state is State.FAILSAFE


def test_illegal_transition_is_rejected():
    machine = MissionStateMachine()
    with pytest.raises(InvalidTransition):
        machine.transition(State.MOVE)


def test_terminal_states_cannot_restart():
    machine = MissionStateMachine()
    machine.transition(State.ARMING)
    machine.transition(State.FAILSAFE)
    with pytest.raises(InvalidTransition):
        machine.transition(State.PREFLIGHT)


def test_preflight_guard_occurs_at_configured_cycle():
    assert not should_start_arming(24, 20)
    assert should_start_arming(25, 20)


def test_arming_requires_both_arm_and_offboard_confirmation():
    assert not arming_complete(True, -1, False, -1, False, 0.0, 30)
    assert arming_complete(True, -1, True, -1, False, 0.0, 30)
    assert arming_complete(False, 2, False, 14, False, 0.0, 30)


def test_arming_airborne_fallback_requires_position_height_and_delay():
    assert not arming_complete(False, -1, False, -1, True, -2.0, 50)
    assert arming_complete(False, -1, False, -1, True, -2.0, 51)


def test_takeoff_and_hover_guards_respect_strict_and_inclusive_boundaries():
    assert takeoff_complete(-4.8, 5.0, 0.25)
    assert not takeoff_complete(-4.75, 5.0, 0.25)
    assert hover_complete(3.0, 3.0)
    assert not hover_complete(2.9, 3.0)


def test_position_timeout_boundary_and_priority():
    assert failsafe_reason(nominal_inputs(now_s=101.5)) is None
    reason = failsafe_reason(
        nominal_inputs(now_s=101.51, mission_elapsed_s=181.0)
    )
    assert reason == "position timeout (XRCE / EKF)"


def test_mission_timeout_only_applies_during_move():
    assert failsafe_reason(nominal_inputs(mission_elapsed_s=180.0)) is None
    assert failsafe_reason(nominal_inputs(mission_elapsed_s=180.1)) == "mission timeout"
    assert failsafe_reason(
        nominal_inputs(state=State.HOVER, mission_elapsed_s=181.0)
    ) is None


def test_avoidance_timeout_requires_both_stale_avoidance_and_progress():
    assert failsafe_reason(
        nominal_inputs(
            now_s=20.1,
            position_stamp_s=20.1,
            avoidance_started_s=8.0,
            waypoint_progress_s=7.9,
        )
    ) == "stuck in avoidance without waypoint progress"
    assert failsafe_reason(
        nominal_inputs(
            now_s=20.1,
            position_stamp_s=20.1,
            avoidance_started_s=8.0,
            waypoint_progress_s=20.0,
        )
    ) is None


def test_geofence_is_enforced_only_in_airborne_states():
    reason = failsafe_reason(nominal_inputs(inside_fence=False))
    assert reason == "geofence breach at N=10.0 E=2.0 alt=5.0"
    assert failsafe_reason(
        nominal_inputs(state=State.ARMING, inside_fence=False)
    ) is None
    assert failsafe_reason(
        nominal_inputs(inside_fence=False, geofence_enabled=False)
    ) is None


def test_terminal_states_ignore_failsafe_inputs():
    for state in (State.PREFLIGHT, State.LANDING, State.FAILSAFE):
        assert failsafe_reason(
            nominal_inputs(
                state=state,
                now_s=1000.0,
                inside_fence=False,
                mission_elapsed_s=1000.0,
            )
        ) is None
