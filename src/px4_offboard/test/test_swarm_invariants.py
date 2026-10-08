"""Invariant testing for the two-vehicle SurveyCoordinator (kinematic, seeded).

The runner is the same kinematic loop as ``simulate()`` in test_swarm_logic.py
(commands move a point mass toward its target; telemetry is fed back), made
configurable so a *property* can be checked after every coordinator step across
many scenarios instead of one fixed recovery.

This is a kinematic fixture around the real coordinator, not PX4 flight evidence.

Invariants implemented so far:
  #2  every one of the four tasks has exactly one owner, the owner is px4_1 or
      px4_2, and report()["assignments"] covers all four tasks.
  #3  a completed task stays completed and its owner never changes.
  #5  COMPLETE means every task was actually visited (armed, within 0.6 m),
      by the same rule as the independent verifier.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field, replace
from typing import Callable

import pytest

from px4_offboard.swarm_logic import HOMES, SurveyCoordinator, Telemetry

EXPECTED_TASKS = ("gate_1", "land_1", "gate_2", "land_2")
SEEDS = range(50)
TICK_S = 0.1
MAX_TICKS = 3500  # same budget as simulate(): 350 s against the coordinator's 330 s timeout

Invariant = Callable[[SurveyCoordinator], None]


def _sample(vehicle: str, now: float = 0.0, **kwargs) -> Telemetry:
    return replace(
        Telemetry(vehicle, vehicle, int(now * 1000), now, HOMES[vehicle],
                  (0, 0, 0), "WAITING", True, False, True),
        **kwargs,
    )


@dataclass(frozen=True)
class Scenario:
    seed: int
    dropout_vehicle: str | None
    dropout_time_s: float
    speed_mps: float

    @staticmethod
    def from_seed(seed: int) -> "Scenario":
        """Deterministic: seeds cycle through none / px4_1 / px4_2, so every
        group of three seeds covers all three cases regardless of the RNG."""
        rng = random.Random(seed)
        return Scenario(
            seed=seed,
            dropout_vehicle=(None, "px4_1", "px4_2")[seed % 3],
            dropout_time_s=rng.uniform(0.0, 40.0),
            speed_mps=rng.choice([0.45, 0.7, 1.0, 1.5]),
        )


@dataclass
class RunResult:
    scenario: Scenario
    final_phase: str
    dropout_fired: bool
    reassignments: int
    steps: int
    checks_run: int
    min_separation_m: float
    # The live coordinator, for characterization tests that inspect final state.
    # Excluded from equality so two runs of one seed still compare equal.
    coordinator: SurveyCoordinator | None = field(default=None, compare=False, repr=False)


def run_scenario(
    scenario: Scenario,
    invariants: list[Invariant],
    mutate: Callable[[SurveyCoordinator, int], None] | None = None,
    observe: Callable[[SurveyCoordinator, int, float], None] | None = None,
) -> RunResult:
    """Run the real coordinator against a point-mass model, calling every
    invariant after every ``step()``. ``mutate`` (tests only) may corrupt the
    coordinator after a step to prove the checks can fail. ``observe`` is a
    read-only hook called after every step with (coordinator, tick, now)."""
    c = SurveyCoordinator()
    positions = dict(HOMES)
    velocities = {v: (0, 0, 0) for v in HOMES}
    states = {v: "WAITING" for v in HOMES}
    faults = {v: "" for v in HOMES}
    minimum = math.inf
    dropout_fired = False
    steps = checks = 0

    for tick in range(MAX_TICKS):
        now = tick * TICK_S
        v_drop = scenario.dropout_vehicle
        if (v_drop and not faults[v_drop] and now >= scenario.dropout_time_s
                and states[v_drop] in {"READY", "MOVING"}):
            faults[v_drop] = "test fault"
            states[v_drop] = "LANDING"
            dropout_fired = True
        for v in HOMES:
            c.update(_sample(v, now, position=positions[v], velocity=velocities[v], state=states[v],
                             armed=states[v] not in {"WAITING", "LANDED"},
                             landed=states[v] in {"WAITING", "LANDED"}, fault=faults[v]), now)
        commands = c.step(now)
        steps += 1
        if mutate is not None:
            mutate(c, tick)
        if observe is not None:
            observe(c, tick, now)
        for invariant in invariants:
            try:
                invariant(c)
            except AssertionError as exc:
                raise AssertionError(
                    f"seed={scenario.seed} tick={tick} t={now:.1f}s phase={c.phase} "
                    f"scenario={scenario}: {exc}"
                ) from exc
            checks += 1
        if c.phase in {"COMPLETE", "ABORTED"}:
            break
        for v, cmd in commands.items():
            action = cmd["action"]
            if states[v] == "LANDED":
                continue
            if faults[v] or action == "land":
                states[v] = "LANDING"
                goal = (*positions[v][:2], 0)
            elif action == "takeoff":
                states[v] = "TAKEOFF"
                goal = (*HOMES[v][:2], -3)
            elif action == "move":
                states[v] = "MOVING"
                goal = cmd["target"]
            else:
                goal = positions[v]
            distance = math.dist(positions[v], goal)
            old = positions[v]
            positions[v] = (
                tuple(old[i] + (goal[i] - old[i]) * min(1, scenario.speed_mps * TICK_S / distance)
                      for i in range(3))
                if distance else old
            )
            velocities[v] = tuple((positions[v][i] - old[i]) / TICK_S for i in range(3))
            if states[v] == "TAKEOFF" and math.dist(positions[v], goal) < 0.01:
                states[v] = "READY"
            if states[v] == "LANDING" and positions[v][2] == 0:
                states[v] = "LANDED"
        minimum = min(minimum, math.dist(*[p[:2] for p in positions.values()]))

    return RunResult(scenario, c.phase, dropout_fired, c.reassignments, steps, checks, minimum, c)


# --- Invariant #2 ----------------------------------------------------------------

def assert_every_task_has_one_valid_owner(c: SurveyCoordinator) -> None:
    names = [t.name for t in c.tasks]
    assert sorted(names) == sorted(EXPECTED_TASKS), f"task list is {names}, expected {list(EXPECTED_TASKS)}"
    for task in c.tasks:
        assert type(task.owner) is str, f"{task.name} owner is {task.owner!r}, not a single vehicle id"
        assert task.owner in HOMES, f"{task.name} owner {task.owner!r} is not one of {sorted(HOMES)}"
    assignments = c.report()["assignments"]
    assert set(assignments) == set(EXPECTED_TASKS), \
        f"report()['assignments'] covers {sorted(assignments)}, expected {list(EXPECTED_TASKS)}"
    for task in c.tasks:
        assert assignments[task.name] == task.owner, \
            f"report says {task.name} -> {assignments[task.name]!r}, coordinator state says {task.owner!r}"


# --- Invariant #3 ----------------------------------------------------------------

class CompletedTasksStayCompleted:
    """Once a task is completed it stays completed, and its owner never changes.

    Needs the previous step, so it is stateful; it resets itself when it is
    handed a different coordinator, so state cannot leak between scenarios."""

    def __init__(self) -> None:
        self._coordinator: SurveyCoordinator | None = None
        self._done: dict[str, str] = {}  # task name -> owner at completion

    def __call__(self, c: SurveyCoordinator) -> None:
        if c is not self._coordinator:
            self._coordinator, self._done = c, {}
        for task in c.tasks:
            if task.name in self._done:
                assert task.completed, f"{task.name} was completed and is now incomplete again"
                assert task.owner == self._done[task.name], (
                    f"{task.name} changed owner after completion: "
                    f"{self._done[task.name]!r} -> {task.owner!r}")
            elif task.completed:
                self._done[task.name] = task.owner


# --- Invariant #5 ----------------------------------------------------------------

VISIT_RADIUS_M = 0.6  # the independent verifier's rule (swarm_verify.verify)


class CompleteMeansEveryTaskWasVisited:
    """COMPLETE means all four tasks are completed AND each one was actually
    visited: some armed vehicle's reported position came within 0.6 m of the task
    point at some step. Same visit rule as swarm_verify.verify (any vehicle,
    armed, < 0.6 m); the tasks here have start == end, so that is one point.

    This is the case-study bug as a rule: the coordinator once accepted a task at
    1.0 m, so it could report COMPLETE for points never visited within 0.6 m."""

    def __init__(self) -> None:
        self._coordinator: SurveyCoordinator | None = None
        self._visited: set[str] = set()
        self._closest: dict[str, float] = {}

    def __call__(self, c: SurveyCoordinator) -> None:
        if c is not self._coordinator:
            self._coordinator, self._visited, self._closest = c, set(), {}
        for task in c.tasks:
            for t in c.telemetry.values():
                d = math.dist(t.position, task.end)
                self._closest[task.name] = min(self._closest.get(task.name, math.inf), d)
                if t.armed and d < VISIT_RADIUS_M:
                    self._visited.add(task.name)
        if c.phase == "COMPLETE":
            incomplete = [t.name for t in c.tasks if not t.completed]
            assert not incomplete, f"COMPLETE with incomplete tasks {incomplete}"
            unvisited = {t.name: round(self._closest.get(t.name, math.inf), 3)
                         for t in c.tasks if t.name not in self._visited}
            assert not unvisited, (
                f"COMPLETE but never visited within {VISIT_RADIUS_M} m "
                f"(closest approach per task, m): {unvisited}")


INVARIANTS: list[Invariant] = [
    assert_every_task_has_one_valid_owner,  # 2
    CompletedTasksStayCompleted(),          # 3
    CompleteMeansEveryTaskWasVisited(),     # 5
]


# --- The invariant on real scenarios ------------------------------------------------

@pytest.fixture(scope="module")
def results() -> list[RunResult]:
    return [run_scenario(Scenario.from_seed(s), INVARIANTS) for s in SEEDS]


def test_invariants_2_3_5_hold_across_50_seeded_scenarios(results):
    # run_scenario raised with the failing seed if any step broke the invariant.
    assert len(results) == 50
    assert all(r.checks_run == r.steps * len(INVARIANTS) for r in results)


def test_the_scenarios_actually_exercise_dropouts_of_each_vehicle_and_no_dropout(results):
    fired = {v: sum(1 for r in results if r.scenario.dropout_vehicle == v and r.dropout_fired)
             for v in HOMES}
    planned_none = [r for r in results if r.scenario.dropout_vehicle is None]
    reassigned = {v: sum(1 for r in results if r.scenario.dropout_vehicle == v and r.reassignments > 0)
                  for v in HOMES}
    assert len(planned_none) >= 15 and all(r.reassignments == 0 and not r.dropout_fired for r in planned_none)
    for v in HOMES:
        assert fired[v] >= 10, f"only {fired[v]} runs actually dropped {v}"
        assert reassigned[v] >= 5, f"only {reassigned[v]} runs reached a reassignment after {v} dropped"


def test_runs_are_reproducible_from_the_seed():
    a = run_scenario(Scenario.from_seed(7), INVARIANTS)
    b = run_scenario(Scenario.from_seed(7), INVARIANTS)
    assert a == b


# --- The checks must be able to fail -------------------------------------------------

def _coordinator() -> SurveyCoordinator:
    return SurveyCoordinator()


def test_invariant_2_passes_on_a_fresh_coordinator():
    assert_every_task_has_one_valid_owner(_coordinator())


@pytest.mark.parametrize(
    "corrupt, fragment",
    [
        (lambda c: setattr(c.tasks[0], "owner", None), "not a single vehicle id"),
        (lambda c: setattr(c.tasks[1], "owner", "px4_3"), "not one of"),
        (lambda c: setattr(c.tasks[2], "owner", ["px4_1", "px4_2"]), "not a single vehicle id"),
        (lambda c: c.tasks.pop(), "task list is"),
        (lambda c: setattr(c.tasks[3], "name", "gate_1"), "task list is"),
    ],
    ids=["no-owner", "unknown-owner", "two-owners", "missing-task", "duplicate-task"],
)
def test_invariant_2_detects_corrupted_ownership(corrupt, fragment):
    c = _coordinator()
    corrupt(c)
    with pytest.raises(AssertionError, match=fragment):
        assert_every_task_has_one_valid_owner(c)


def test_invariant_2_detects_a_report_that_disagrees_with_the_tasks():
    c = _coordinator()
    original = c.report
    c.report = lambda: {**original(), "assignments": {k: v for k, v in original()["assignments"].items()
                                                       if k != "land_2"}}
    with pytest.raises(AssertionError, match="covers"):
        assert_every_task_has_one_valid_owner(c)


def test_invariants_3_and_5_are_not_vacuous_on_these_scenarios(results):
    # #3 only bites on completed tasks and #5 only on COMPLETE runs: make sure both happen a lot.
    completed = sum(t.completed for r in results for t in r.coordinator.tasks)
    complete_runs = sum(r.final_phase == "COMPLETE" for r in results)
    assert completed >= 190, f"only {completed} task completions across 50 runs"
    assert complete_runs >= 45, f"only {complete_runs} runs reached COMPLETE"


def _run_with(seed: int, mutate) -> None:
    run_scenario(Scenario.from_seed(seed), [CompletedTasksStayCompleted(), CompleteMeansEveryTaskWasVisited()],
                 mutate=mutate)


def test_invariant_3_detects_a_completed_task_becoming_incomplete():
    def uncomplete(c, tick):
        done = [t for t in c.tasks if t.completed]
        if done and tick % 50 == 0:
            done[0].completed = False

    with pytest.raises(AssertionError, match="now incomplete again"):
        _run_with(0, uncomplete)


def test_invariant_3_detects_ownership_moving_after_completion():
    def steal(c, tick):
        for t in c.tasks:
            if t.completed:
                t.owner = "px4_2" if t.owner == "px4_1" else "px4_1"
                return

    with pytest.raises(AssertionError, match="changed owner after completion"):
        _run_with(0, steal)


def test_invariant_5_detects_a_forged_complete():
    def forge(c, tick):
        if tick == 30:
            for t in c.tasks:
                t.completed = True
            c.phase = "COMPLETE"

    with pytest.raises(AssertionError, match="never visited within 0.6 m"):
        _run_with(1, forge)


def _accept_task_endpoints_at_1m(c: SurveyCoordinator, tick: int) -> None:
    """Re-creates the pre-fix coordinator rule from the case study -- a task
    endpoint accepted at 1.0 m instead of 0.5 m -- from outside, so
    swarm_logic.py is untouched."""
    for v in HOMES:
        t, name = c.telemetry.get(v), c.active.get(v)
        if v in c.retired or t is None or name is None or len(c.routes[v]) != 1:
            continue
        if math.dist(t.position, c.routes[v][0]) < 1.0:
            c.routes[v].pop(0)
            next(task for task in c.tasks if task.name == name).completed = True
            c.active[v] = None


def test_invariant_5_catches_the_original_case_study_bug():
    caught = []
    for seed in range(12):
        try:
            _run_with(seed, _accept_task_endpoints_at_1m)
        except AssertionError as exc:
            if "never visited" in str(exc):
                caught.append(seed)
    # With the old 1.0 m acceptance, runs reach COMPLETE without real visits, and #5 says so.
    assert len(caught) >= 6, f"invariant 5 caught the 1.0 m acceptance bug in only {caught} of 12 seeds"


def test_the_runner_checks_after_every_step_and_reports_the_failing_seed():
    def corrupt_after_50_steps(c: SurveyCoordinator, tick: int) -> None:
        if tick == 50:
            c.tasks[2].owner = "px4_9"

    with pytest.raises(AssertionError, match=r"seed=4 tick=50 .*px4_9"):
        run_scenario(Scenario.from_seed(4), INVARIANTS, mutate=corrupt_after_50_steps)


# === Characterization: seed 10 ====================================================
#
# NOT a statement that this behaviour is right. It pins down what the coordinator
# does today in the one seeded scenario (of 50) that ends ABORTED, so that the
# decision "bug or intended safe abort?" is made from a reproducible record, and
# so that any future change to this behaviour is noticed and has to be deliberate.
#
# Sequence (px4_1 drops out at 22.9 s, flying at 1.5 m/s):
#   px4_1 lands 1.8 m from gate_1's task point -> retired as a grounded obstacle;
#   (gate_1 is unreachable for two independent reasons: the planner's 3.5 m keep-out
#   around a retired vehicle, and the 2.5 m minimum-separation rule itself);
#   gate_1 and land_1 are reassigned to px4_2; px4_2 finishes everything it can
#   reach; gate_1 is never routable; the coordinator idles until its mission
#   timeout and aborts.

SEED_10 = 10


@dataclass
class Seed10Record:
    retire_t: float | None = None
    owners_at_retire: dict | None = None
    gate_1_owner_history: list | None = None
    px4_2_active_ever: set | None = None
    last_completion_t: float = 0.0
    abort_t: float | None = None
    completed_count: int = 0

    def __post_init__(self) -> None:
        self.gate_1_owner_history = []
        self.px4_2_active_ever = set()


def _run_seed_10() -> tuple[RunResult, Seed10Record]:
    rec = Seed10Record()

    def observe(c: SurveyCoordinator, tick: int, now: float) -> None:
        owner = c.tasks[0].owner
        if not rec.gate_1_owner_history or rec.gate_1_owner_history[-1] != owner:
            rec.gate_1_owner_history.append(owner)
        if rec.retire_t is None and "px4_1" in c.retired:
            rec.retire_t = now
            rec.owners_at_retire = {t.name: t.owner for t in c.tasks}
        if c.active["px4_2"]:
            rec.px4_2_active_ever.add(c.active["px4_2"])
        done = sum(t.completed for t in c.tasks)
        if done != rec.completed_count:
            rec.completed_count, rec.last_completion_t = done, now
        if rec.abort_t is None and c.phase == "ABORTED":
            rec.abort_t = now

    return run_scenario(Scenario.from_seed(SEED_10), INVARIANTS, observe=observe), rec


def _geometry(c: SurveyCoordinator) -> dict:
    """Everything needed to see why gate_1 is unreachable, for assertion messages."""
    from px4_offboard.swarm_logic import segment_distance

    grounded = c.telemetry["px4_1"].position
    gate = c.tasks[0]
    return {
        "grounded_px4_1_ned": tuple(round(x, 2) for x in grounded),
        "gate_1_endpoint_ned": gate.end,
        "horizontal_distance_m": round(math.dist(grounded[:2], gate.end[:2]), 2),
        "transect_to_grounded_m": round(segment_distance(gate.start, gate.end, grounded, grounded), 2),
        "reservation_m": c.reservation,
        "minimum_separation_m": c.minimum_separation,
        # A task counts as visited within 0.5 m of its endpoint, so the closest a
        # vehicle that completes gate_1 gets to the grounded one is distance - 0.5.
        "closest_visit_approach_m": round(math.dist(grounded[:2], gate.end[:2]) - 0.5, 2),
        "obstacle_clearance_m": c.obstacle_clearance,
        # _route() turns a retired vehicle into a box of half-size reservation-clearance
        # and plans with `clearance` on top, so it blocks +-reservation around it.
        "blocked_half_extent_m": (c.reservation - c.obstacle_clearance) + c.obstacle_clearance,
        "north_offset_m": round(abs(gate.end[0] - grounded[0]), 2),
        "east_offset_m": round(abs(gate.end[1] - grounded[1]), 2),
    }


def test_seed_10_the_scenario_is_the_one_we_characterized():
    sc = Scenario.from_seed(SEED_10)
    assert (sc.dropout_vehicle, sc.speed_mps) == ("px4_1", 1.5)
    assert sc.dropout_time_s == pytest.approx(22.86, abs=0.01)


def test_seed_10_px4_1_drops_lands_and_is_retired():
    result, rec = _run_seed_10()
    c = result.coordinator
    assert result.dropout_fired
    assert "px4_1" in c.retired
    grounded = c.telemetry["px4_1"]
    assert grounded.state == "LANDED" and grounded.landed and not grounded.armed
    assert grounded.position[2] == 0  # on the ground
    # Retirement needs 1 s of confirmed landed+disarmed evidence after the dropout.
    assert rec.retire_t is not None and rec.retire_t == pytest.approx(26.0, abs=0.6)


def test_seed_10_gate_1_is_reassigned_to_px4_2_but_never_completed():
    result, rec = _run_seed_10()
    c = result.coordinator
    gate_1 = c.tasks[0]
    assert gate_1.name == "gate_1"
    assert rec.gate_1_owner_history == ["px4_1", "px4_2"]
    assert rec.owners_at_retire["gate_1"] == "px4_2" and rec.owners_at_retire["land_1"] == "px4_2"
    assert result.reassignments == 2
    assert not gate_1.completed
    # Everything else was reachable and was done.
    assert sorted(t.name for t in c.tasks if t.completed) == ["gate_2", "land_1", "land_2"]


def test_seed_10_the_grounded_vehicle_blocks_the_goal_px4_2_would_need():
    result, rec = _run_seed_10()
    c = result.coordinator
    g = _geometry(c)
    gate = c.tasks[0]
    # 1. The task point sits inside the grounded vehicle's keep-out zone.
    assert g["horizontal_distance_m"] < c.reservation, g
    assert g["north_offset_m"] <= g["blocked_half_extent_m"] and g["east_offset_m"] <= g["blocked_half_extent_m"], g
    # 2. So the planner refuses the goal (this is the exception the coordinator swallows)...
    with pytest.raises(ValueError, match="outside free planning space"):
        c._route("px4_2", gate.start)
    # 3. ...and the coordinator's second guard (transect vs retired vehicle) would skip it too.
    assert g["transect_to_grounded_m"] <= c.reservation, g
    # 3b. Independently of routing: visiting gate_1 means passing within 2.5 m of the
    #     grounded vehicle, which the coordinator's own minimum-separation rule forbids.
    #     (Throwaway experiment, not asserted here: with the reservation relaxed the
    #     planner accepts the goal but the run aborts ~6 s into the approach with
    #     "predicted separation breach".)
    assert g["closest_visit_approach_m"] < g["minimum_separation_m"], g
    # 4. px4_2 therefore never even tried gate_1.
    assert "gate_1" not in rec.px4_2_active_ever, rec.px4_2_active_ever
    assert gate.owner == "px4_2" and not gate.completed


def test_seed_10_ends_aborted_by_the_mission_timeout_not_by_a_safety_trip():
    result, rec = _run_seed_10()
    c = result.coordinator
    assert c.phase == "ABORTED"
    assert c.reason == "mission timeout / blocked route"
    # The abort fires the first step after `timeout` seconds since start...
    assert rec.abort_t == pytest.approx(c.started + c.timeout, abs=0.3)
    # ...and nothing safety-related went wrong on the way: vehicles stayed far apart.
    assert result.min_separation_m > 2.5 * 2
    # The coordinator knew nothing more useful after the last reachable task finished:
    # it then idled for most of the mission budget.
    assert rec.last_completion_t == pytest.approx(46.2, abs=1.0)
    idle_s = rec.abort_t - rec.last_completion_t
    assert idle_s > 250, f"idle for only {idle_s:.0f} s"


def test_seed_10_reproduces_identically_every_time():
    a, ra = _run_seed_10()
    b, rb = _run_seed_10()
    assert a == b and ra == rb
    assert _geometry(a.coordinator) == _geometry(b.coordinator)
