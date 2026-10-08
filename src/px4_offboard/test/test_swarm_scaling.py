"""1, 2, 3, 5 and 10 drones on generated layouts (kinematic, seeded; not PX4).

What is established here, and what is not:

* SAFETY holds at every size: invariants #2, #3, #5 are checked after every step
  and no two vehicles come within the 2.5 m minimum separation.
* LIVENESS without a dropout holds at 1, 2, 3, 5 and 10 drones (BUG-021).
* A dropout can still end in the 330 s timeout, or in a reported circular wait.
  Those rates are measured in bugs/BUG-021.md. An abort must not be a separation
  or prediction trip.
"""
from __future__ import annotations

import pytest

import test_swarm_invariants as H
from px4_offboard.swarm_fleet_layout import make_fleet

pytestmark = pytest.mark.integration

SIZES = (1, 2, 3, 5, 10)
SEEDS = range(15)  # every third seed is a no-dropout run


def _invariants():
    return [H.assert_every_task_has_one_valid_owner, H.CompletedTasksStayCompleted(),
            H.CompleteMeansEveryTaskWasVisited()]


def _run(n: int, seed: int) -> H.RunResult:
    fleet = make_fleet(n)
    return H.run_scenario(H.Scenario.for_fleet(seed, fleet), _invariants(), fleet=fleet)


@pytest.fixture(scope="module")
def runs() -> dict[int, list[H.RunResult]]:
    # run_scenario raises (with seed, tick and phase) on any invariant violation.
    return {n: [_run(n, s) for s in SEEDS] for n in SIZES}


@pytest.mark.parametrize("n", SIZES)
def test_safety_holds_for_every_fleet_size(runs, n):
    results = runs[n]
    assert len(results) == len(SEEDS)
    assert all(r.checks_run == r.steps * 3 for r in results)
    if n > 1:
        closest = min(r.min_separation_m for r in results)
        assert closest >= 2.5, f"{n} drones came within {closest:.2f} m"


@pytest.mark.parametrize("n", SIZES)
def test_dropouts_really_happen_at_every_size(runs, n):
    fired = sum(r.dropout_fired for r in runs[n])
    assert fired >= 8, f"only {fired} of {len(SEEDS)} runs at N={n} actually lost a drone"


@pytest.mark.parametrize("n", (1, 2, 3, 5))
def test_missions_without_failures_complete_up_to_five_drones(runs, n):
    clean = [r for r in runs[n] if r.scenario.dropout_vehicle is None]
    assert clean and all(r.final_phase == "COMPLETE" for r in clean), \
        [(r.scenario.seed, r.final_phase, r.coordinator.reason) for r in clean]


def test_ten_drones_complete_without_failures(runs):
    clean = [r for r in runs[10] if r.scenario.dropout_vehicle is None]
    assert clean and all(r.final_phase == "COMPLETE" for r in clean), \
        [(r.scenario.seed, r.final_phase, r.coordinator.reason) for r in clean]


def test_ten_drone_aborts_are_not_safety_trips(runs):
    # A dropout may still time out, or stop on a reported circular wait.
    # It must not be a separation breach or a predicted-separation abort.
    reasons = {r.coordinator.reason for r in runs[10] if r.final_phase == "ABORTED"}
    for reason in reasons:
        assert reason == "mission timeout / blocked route" or reason.startswith("deadlock:"), reason


def test_one_drone_cannot_recover_from_its_own_failure(runs):
    # By design: with nobody to hand tasks to, the coordinator aborts.
    lost = [r for r in runs[1] if r.dropout_fired]
    assert lost and all(r.final_phase == "ABORTED" and r.coordinator.reason == "no available vehicles"
                        for r in lost)
