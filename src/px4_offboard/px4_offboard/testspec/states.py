"""Run lifecycle. Every transition is explicit and recorded; an illegal one raises.

    CREATED -> VALIDATED -> INITIALIZING -> READY -> RUNNING -> EVALUATING -> PASSED | FAILED

ERROR (infrastructure: invalid spec, adapter failure, no telemetry, wall-clock
budget exceeded) and ABORTED (cancellation) are reachable from every
non-terminal state. PASSED is reachable only from EVALUATING, so a run that
never evaluated cannot pass.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum


class RunState(Enum):
    CREATED = "CREATED"
    VALIDATED = "VALIDATED"
    INITIALIZING = "INITIALIZING"
    READY = "READY"
    RUNNING = "RUNNING"
    EVALUATING = "EVALUATING"
    PASSED = "PASSED"
    FAILED = "FAILED"
    ERROR = "ERROR"
    ABORTED = "ABORTED"


TERMINAL = frozenset({RunState.PASSED, RunState.FAILED, RunState.ERROR, RunState.ABORTED})

_FORWARD = {
    RunState.CREATED: {RunState.VALIDATED},
    RunState.VALIDATED: {RunState.INITIALIZING},
    RunState.INITIALIZING: {RunState.READY},
    RunState.READY: {RunState.RUNNING},
    RunState.RUNNING: {RunState.EVALUATING},
    RunState.EVALUATING: {RunState.PASSED, RunState.FAILED},
}

ALLOWED: dict[RunState, frozenset[RunState]] = {
    state: frozenset(_FORWARD.get(state, set()) | ({RunState.ERROR, RunState.ABORTED} if state not in TERMINAL else set()))
    for state in RunState
}


class IllegalTransition(RuntimeError):
    pass


@dataclass(frozen=True)
class Transition:
    state: RunState
    at_utc: str
    reason: str


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


class RunStateMachine:
    def __init__(self, on_transition=None) -> None:
        self._state = RunState.CREATED
        self._on_transition = on_transition
        self.history: list[Transition] = [Transition(RunState.CREATED, utc_now(), "run created")]

    @property
    def state(self) -> RunState:
        return self._state

    @property
    def terminal(self) -> bool:
        return self._state in TERMINAL

    def to(self, new: RunState, reason: str = "") -> None:
        if new not in ALLOWED[self._state]:
            raise IllegalTransition(f"{self._state.value} -> {new.value} is not allowed")
        old, self._state = self._state, new
        self.history.append(Transition(new, utc_now(), reason))
        if self._on_transition is not None:
            self._on_transition(old, new, reason)
