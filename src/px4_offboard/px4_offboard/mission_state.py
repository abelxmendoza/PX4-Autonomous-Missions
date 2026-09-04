"""Deterministic state-transition policy for an offboard PX4 mission."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto


class State(Enum):
    PREFLIGHT = auto()
    ARMING = auto()
    TAKEOFF = auto()
    HOVER = auto()
    MOVE = auto()
    LANDING = auto()
    FAILSAFE = auto()


LEGAL_TRANSITIONS = {
    State.PREFLIGHT: frozenset((State.ARMING,)),
    State.ARMING: frozenset((State.TAKEOFF, State.FAILSAFE)),
    State.TAKEOFF: frozenset((State.HOVER, State.FAILSAFE)),
    State.HOVER: frozenset((State.MOVE, State.FAILSAFE)),
    State.MOVE: frozenset((State.LANDING, State.FAILSAFE)),
    State.LANDING: frozenset(),
    State.FAILSAFE: frozenset(),
}


class InvalidTransition(ValueError):
    """Raised when code attempts a transition excluded by mission design."""


class MissionStateMachine:
    def __init__(self) -> None:
        self.state = State.PREFLIGHT

    def transition(self, new_state: State) -> None:
        if new_state not in LEGAL_TRANSITIONS[self.state]:
            raise InvalidTransition(
                f"illegal mission transition: {self.state.name} -> {new_state.name}"
            )
        self.state = new_state


@dataclass(frozen=True)
class FailsafeInputs:
    state: State
    now_s: float
    have_position: bool
    position_stamp_s: float
    position_timeout_s: float
    mission_elapsed_s: float
    mission_timeout_s: float
    avoidance_started_s: float
    waypoint_progress_s: float
    avoidance_stuck_s: float
    geofence_enabled: bool
    inside_fence: bool
    north_m: float
    east_m: float
    down_m: float


def should_start_arming(counter: int, preflight_cycles: int) -> bool:
    """Setpoints have streamed long enough to issue ARM and enter ARMING."""
    return counter == preflight_cycles + 5


def arming_complete(
    flag_armed: bool,
    arming_state: int,
    flag_offboard: bool,
    navigation_state: int,
    have_position: bool,
    down_m: float,
    counter: int,
) -> bool:
    armed = flag_armed or arming_state == 2
    offboard = flag_offboard or navigation_state == 14
    confirmed = armed and offboard
    airborne_fallback = have_position and abs(down_m) > 1.0 and counter > 50
    return confirmed or airborne_fallback


def takeoff_complete(down_m: float, hover_altitude_m: float, tolerance_m: float) -> bool:
    return abs(down_m + hover_altitude_m) < tolerance_m


def hover_complete(elapsed_s: float, required_s: float) -> bool:
    return elapsed_s >= required_s


def failsafe_reason(inputs: FailsafeInputs) -> str | None:
    """Return the highest-priority active failure, or ``None`` when healthy."""
    if inputs.state in (State.PREFLIGHT, State.LANDING, State.FAILSAFE):
        return None

    if (
        inputs.have_position
        and inputs.now_s - inputs.position_stamp_s > inputs.position_timeout_s
    ):
        return "position timeout (XRCE / EKF)"

    if (
        inputs.state == State.MOVE
        and inputs.mission_elapsed_s > inputs.mission_timeout_s
    ):
        return "mission timeout"

    avoidance_is_stuck = (
        inputs.state == State.MOVE
        and inputs.avoidance_started_s > 0.0
        and inputs.now_s - inputs.avoidance_started_s > inputs.avoidance_stuck_s
        and inputs.now_s - inputs.waypoint_progress_s > inputs.avoidance_stuck_s
    )
    if avoidance_is_stuck:
        return "stuck in avoidance without waypoint progress"

    if (
        inputs.geofence_enabled
        and inputs.have_position
        and inputs.state in (State.TAKEOFF, State.HOVER, State.MOVE)
        and not inputs.inside_fence
    ):
        return (
            f"geofence breach at N={inputs.north_m:.1f} E={inputs.east_m:.1f} "
            f"alt={-inputs.down_m:.1f}"
        )

    return None
