"""Run lifecycle: only the documented transitions exist, and PASSED needs EVALUATING."""
from __future__ import annotations

import pytest

from px4_offboard.testspec.states import ALLOWED, TERMINAL, IllegalTransition, RunState, RunStateMachine

HAPPY = [RunState.VALIDATED, RunState.INITIALIZING, RunState.READY, RunState.RUNNING, RunState.EVALUATING]


def test_happy_path_records_every_transition_in_order():
    seen = []
    sm = RunStateMachine(lambda old, new, reason: seen.append((old, new, reason)))
    for state in HAPPY:
        sm.to(state, f"to {state.value}")
    sm.to(RunState.PASSED, "done")
    assert [t.state for t in sm.history] == [RunState.CREATED, *HAPPY, RunState.PASSED]
    assert seen[-1] == (RunState.EVALUATING, RunState.PASSED, "done")
    assert all(t.at_utc.endswith("+00:00") for t in sm.history)
    assert sm.terminal


@pytest.mark.parametrize("state", [s for s in RunState if s not in (RunState.EVALUATING,)])
def test_passed_and_failed_are_reachable_only_from_evaluating(state):
    assert RunState.PASSED not in ALLOWED[state]
    assert RunState.FAILED not in ALLOWED[state]


@pytest.mark.parametrize("state", [s for s in RunState if s not in TERMINAL])
def test_error_and_aborted_are_reachable_from_every_non_terminal_state(state):
    assert {RunState.ERROR, RunState.ABORTED} <= ALLOWED[state]


@pytest.mark.parametrize("state", sorted(TERMINAL, key=lambda s: s.value))
def test_terminal_states_have_no_way_out(state):
    assert ALLOWED[state] == frozenset()


def _walk_to(target: RunState) -> RunStateMachine:
    sm = RunStateMachine()
    for state in HAPPY:
        if sm.state == target:
            break
        sm.to(state)
    return sm


@pytest.mark.parametrize(
    "start, illegal",
    [
        (RunState.CREATED, RunState.RUNNING),
        (RunState.CREATED, RunState.PASSED),
        (RunState.VALIDATED, RunState.READY),
        (RunState.READY, RunState.EVALUATING),
        (RunState.RUNNING, RunState.PASSED),
        (RunState.RUNNING, RunState.FAILED),
        (RunState.EVALUATING, RunState.RUNNING),
    ],
)
def test_skipping_or_reversing_a_phase_raises(start, illegal):
    sm = _walk_to(start)
    assert sm.state == start
    with pytest.raises(IllegalTransition):
        sm.to(illegal)
    assert sm.state == start


def test_a_finished_run_cannot_be_reopened_or_relabelled():
    sm = RunStateMachine()
    sm.to(RunState.ERROR, "spec invalid")
    for state in RunState:
        with pytest.raises(IllegalTransition):
            sm.to(state)
    assert sm.state == RunState.ERROR
