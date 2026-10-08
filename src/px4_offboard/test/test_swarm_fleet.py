"""N-vehicle coordinator: fleet description, all-pairs separation, reassignment.

The two-vehicle behaviour must not change: test_two_vehicle_behaviour_is_unchanged
replays the 50 seeded scenarios from test_swarm_invariants.py and compares every
outcome with the snapshot taken before the refactor (data/swarm_two_vehicle_baseline.json).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from px4_offboard.swarm_logic import (
    DEFAULT_FLEET,
    HOMES,
    LANDINGS,
    Fleet,
    FleetTask,
    SurveyCoordinator,
    Telemetry,
)

BASELINE = Path(__file__).parent / "data" / "swarm_two_vehicle_baseline.json"


def tel(vehicle, now=0.0, position=(0.0, 0.0, 0.0), velocity=(0.0, 0.0, 0.0), state="WAITING",
        armed=False, landed=True, fault=""):
    return Telemetry(vehicle, vehicle, int(now * 1000), now, position, velocity, state, True,
                     armed, landed, fault)


def three(homes, tasks=None):
    tasks = tasks or (FleetTask("t_a", "a", (20.0, 0.0, -3.0), (20.0, 0.0, -3.0)),)
    return Fleet(homes=homes, landings={v: (50.0, p[1], 0.0) for v, p in homes.items()}, tasks=tuple(tasks))


# --- the default fleet is exactly the old hard-coded one ------------------------------

def test_default_fleet_matches_the_original_two_vehicle_constants():
    assert DEFAULT_FLEET.homes == HOMES and DEFAULT_FLEET.landings == LANDINGS
    c = SurveyCoordinator()
    assert [(t.name, t.owner, t.start, t.end) for t in c.tasks] == [
        ("gate_1", "px4_1", (28.0, 11.0, -3.0), (28.0, 11.0, -3.0)),
        ("land_1", "px4_1", (50.0, -2.0, -3.0), (50.0, -2.0, -3.0)),
        ("gate_2", "px4_2", (28.0, 16.5, -3.0), (28.0, 16.5, -3.0)),
        ("land_2", "px4_2", (50.0, 5.0, -3.0), (50.0, 5.0, -3.0)),
    ]


def test_two_vehicle_behaviour_is_unchanged_on_all_50_seeds():
    import test_swarm_invariants as harness

    expected = json.loads(BASELINE.read_text())
    for row in expected:
        seed = row[0]
        r = harness.run_scenario(harness.Scenario.from_seed(seed), [])
        c = r.coordinator
        got = [seed, r.final_phase, c.reason, r.reassignments, r.steps, round(r.min_separation_m, 6),
               sorted(t.name for t in c.tasks if t.completed), {t.name: t.owner for t in c.tasks}]
        assert got == row, f"seed {seed} changed:\n  before {row}\n  after  {got}"


# --- fleet validation ------------------------------------------------------------------

@pytest.mark.parametrize(
    "kwargs, fragment",
    [
        (dict(homes={}), "at least one vehicle"),
        (dict(landings={"a": (50, 0, 0)}), "landing for every vehicle"),
        (dict(tasks=(FleetTask("x", "zz", (1, 1, -3), (1, 1, -3)),)), "unknown owner"),
        (dict(tasks=(FleetTask("x", "a", (1, 1, -3), (1, 1, -3)),) * 2), "duplicate task"),
    ],
)
def test_fleet_rejects_inconsistent_descriptions(kwargs, fragment):
    base = dict(homes={"a": (0, 0, 0), "b": (0, 6, 0)},
                landings={"a": (50, 0, 0), "b": (50, 6, 0)},
                tasks=(FleetTask("x", "a", (1, 1, -3), (1, 1, -3)),))
    base.update(kwargs)
    with pytest.raises(ValueError, match=fragment):
        Fleet(**base)


def test_telemetry_parse_accepts_vehicles_of_a_custom_fleet():
    data = {"vehicle": "c", "session": "s", "seq": 1, "sent": 0.0, "position": [0, 0, 0],
            "velocity": [0, 0, 0], "state": "WAITING", "valid": True, "armed": False, "landed": True}
    with pytest.raises(ValueError):
        Telemetry.parse(data)  # default fleet: px4_1 / px4_2 only
    assert Telemetry.parse(data, vehicles={"a", "b", "c"}).vehicle == "c"


# --- separation is checked for every pair, not just the first two -------------------

def test_separation_breach_between_the_first_and_third_vehicle_aborts():
    homes = {"a": (0.0, 0.0, 0.0), "b": (0.0, 10.0, 0.0), "c": (0.0, 2.0, 0.0)}  # only a-c too close
    c = SurveyCoordinator(fleet=three(homes))
    for v, p in homes.items():
        c.update(tel(v, position=p), 0.0)
    c.step(0.0)
    assert c.phase == "ABORTED" and c.reason == "minimum horizontal separation breached"


def test_predicted_breach_between_the_first_and_third_vehicle_aborts():
    homes = {"a": (0.0, 0.0, 0.0), "b": (0.0, 12.0, 0.0), "c": (0.0, 3.5, 0.0)}
    c = SurveyCoordinator(fleet=three(homes))
    c.update(tel("a", position=homes["a"], velocity=(0.0, 2.0, 0.0)), 0.0)  # a and c closing at 4 m/s
    c.update(tel("b", position=homes["b"]), 0.0)
    c.update(tel("c", position=homes["c"], velocity=(0.0, -2.0, 0.0)), 0.0)
    c.step(0.0)
    assert c.phase == "ABORTED" and c.reason == "predicted separation breach"


def test_three_well_separated_vehicles_do_not_abort():
    homes = {"a": (0.0, -3.0, 0.0), "b": (0.0, 4.0, 0.0), "c": (0.0, 11.0, 0.0)}
    c = SurveyCoordinator(fleet=three(homes))
    for v, p in homes.items():
        c.update(tel(v, position=p), 0.0)
    c.step(0.0)
    assert c.phase == "STARTING"


# --- reassignment: distance + workload, healthy vehicles only ------------------------

def _retire(c: SurveyCoordinator, failed: str, positions: dict, faulted_too: tuple = ()) -> None:
    """Start everyone landed (WAITING), then have `failed` report landed + disarmed + fault
    for longer than the coordinator's 1 s retirement dwell while the others fly."""
    for v, p in positions.items():
        c.update(tel(v, 0.0, position=p), 0.0)
    c.step(0.0)
    for tick in range(1, 16):  # 0.1 .. 1.5 s
        now = tick / 10
        for v, p in positions.items():
            if v == failed:
                c.update(tel(v, now, position=p, state="LANDED", landed=True, fault="test"), now)
            else:
                c.update(tel(v, now, position=p, state="READY", armed=True, landed=False,
                             fault="test" if v in faulted_too else ""), now)
        c.step(now)


def _fleet(positions, tasks):
    return Fleet(homes={v: (p[0], p[1], 0.0) for v, p in positions.items()},
                 landings={v: (50.0, p[1], 0.0) for v, p in positions.items()},
                 tasks=tuple(FleetTask(n, o, p, p) for n, o, p in tasks))


def test_released_tasks_go_to_the_nearest_healthy_vehicle_when_workloads_are_equal():
    pos = {"a": (0.0, 5.0, -3.0), "b": (10.0, 0.0, -3.0), "c": (10.0, 15.0, -3.0)}
    fleet = _fleet(pos, [("near_b", "a", (14.0, 0.0, -3.0)), ("near_c", "a", (14.0, 15.0, -3.0)),
                         ("b_own", "b", (30.0, 0.0, -3.0)), ("c_own", "c", (30.0, 15.0, -3.0))])
    c = SurveyCoordinator(fleet=fleet)
    _retire(c, "a", pos)
    owners = {t.name: t.owner for t in c.tasks}
    assert "a" in c.retired
    assert owners["near_b"] == "b" and owners["near_c"] == "c"
    assert c.reassignments == 2


def test_workload_outweighs_a_small_distance_advantage():
    # c is 2 m closer to the released task but already has three tasks; b has none.
    pos = {"a": (0.0, 5.0, -3.0), "b": (10.0, 0.0, -3.0), "c": (10.0, 4.0, -3.0)}
    fleet = _fleet(pos, [("x", "a", (14.0, 3.0, -3.0)),
                         ("c1", "c", (30.0, 10.0, -3.0)), ("c2", "c", (35.0, 10.0, -3.0)),
                         ("c3", "c", (40.0, 10.0, -3.0))])
    c = SurveyCoordinator(fleet=fleet)
    _retire(c, "a", pos)
    assert {t.name: t.owner for t in c.tasks}["x"] == "b"


def test_a_faulted_vehicle_never_receives_released_tasks():
    pos = {"a": (0.0, 5.0, -3.0), "b": (10.0, 0.0, -3.0), "c": (10.0, 15.0, -3.0)}
    fleet = _fleet(pos, [("near_c", "a", (14.0, 15.0, -3.0)), ("b_own", "b", (30.0, 0.0, -3.0))])
    c = SurveyCoordinator(fleet=fleet)
    _retire(c, "a", pos, faulted_too=("c",))  # c has faulted but is still airborne
    assert {t.name: t.owner for t in c.tasks}["near_c"] == "b"


def test_single_vehicle_fleet_has_no_pairs_to_check_and_starts():
    c = SurveyCoordinator(fleet=_fleet({"solo": (0.0, 0.0, -3.0)}, [("t", "solo", (10.0, 0.0, -3.0))]))
    c.update(tel("solo"), 0.0)
    c.step(0.0)
    assert c.phase == "STARTING"
