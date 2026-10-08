"""Regression tests for BUG-021: reservation holds that never resolve.

Every scenario below is the real ``SurveyCoordinator`` flown by the kinematic
harness (``run_scenario``). Nothing here pokes routes or phases in by hand.
The layouts are chosen so the current planner builds crossing polylines; the
reservation check then parks every drone, which is the gridlock.
"""
from __future__ import annotations

import pytest

import test_swarm_invariants as H
from px4_offboard.swarm_fleet_layout import make_fleet
from px4_offboard.swarm_logic import Fleet, FleetTask

pytestmark = pytest.mark.integration


def _head_on() -> Fleet:
    """Two drones, each one's only gate sitting on the other's launch pad.

    Both routes are the same north-south line, so each next segment passes
    through the other drone and both hold.
    """
    return Fleet(
        homes={"px4_1": (8.0, 14.0, 0.0), "px4_2": (40.0, 14.0, 0.0)},
        landings={"px4_1": (46.0, 16.0, 0.0), "px4_2": (4.0, 12.0, 0.0)},
        tasks=(
            FleetTask("north_gate", "px4_1", (40.0, 14.0, -3.0), (40.0, 14.0, -3.0)),
            FleetTask("south_gate", "px4_2", (8.0, 14.0, -3.0), (8.0, 14.0, -3.0)),
        ),
    )


def _triangle() -> Fleet:
    """Three drones whose opening legs form one waiting loop.

    px4_3 sits on the northern goals of px4_1 and px4_2, and px4_3's own goal
    is back through the pair on the south edge. Nobody's first segment is clear.
    """
    return Fleet(
        homes={
            "px4_1": (6.0, 12.0, 0.0),
            "px4_2": (6.0, 16.0, 0.0),
            "px4_3": (36.0, 14.0, 0.0),
        },
        landings={
            "px4_1": (48.0, 12.0, 0.0),
            "px4_2": (48.0, 16.0, 0.0),
            "px4_3": (48.0, 8.0, 0.0),
        },
        tasks=(
            FleetTask("g1", "px4_1", (36.0, 14.0, -3.0), (36.0, 14.0, -3.0)),
            FleetTask("g2", "px4_2", (36.0, 16.0, -3.0), (36.0, 16.0, -3.0)),
            FleetTask("g3", "px4_3", (6.0, 14.0, -3.0), (6.0, 14.0, -3.0)),
        ),
    )


def _quiet(seed: int, fleet: Fleet, speed: float = 1.0) -> H.Scenario:
    return H.Scenario(seed=seed, dropout_vehicle=None, dropout_time_s=0.0, speed_mps=speed)


def _assert_finished(result: H.RunResult) -> None:
    c = result.coordinator
    assert result.final_phase == "COMPLETE", (c.phase, c.reason, c.report())
    assert result.min_separation_m >= 2.5
    assert all(t.completed for t in c.tasks)
    assert c.deadlock is None


def test_two_drones_blocking_each_other_both_finish():
    fleet = _head_on()
    result = H.run_scenario(_quiet(0, fleet), [], fleet=fleet)
    _assert_finished(result)


def test_three_drones_in_a_circular_wait_all_finish():
    fleet = _triangle()
    result = H.run_scenario(_quiet(0, fleet), [], fleet=fleet)
    _assert_finished(result)


@pytest.mark.parametrize("seed", [0, 3, 6, 9, 12, 15, 18, 21, 24, 27])
def test_ten_drones_do_not_stay_stuck(seed):
    """The ten no-dropout seeds of the 30-seed sweep. Unfixed, every one aborts
    at the 330 s timeout with drones still holding for each other."""
    fleet = make_fleet(10)
    result = H.run_scenario(H.Scenario.for_fleet(seed, fleet), [], fleet=fleet)
    assert result.scenario.dropout_vehicle is None
    _assert_finished(result)


def test_a_drone_failing_while_others_are_waiting_does_not_freeze_the_swarm():
    """px4_6 drops at 55 s on the seed-0 ten-drone layout, while other drones
    are holding with a route. The survivors finish every task, including the
    ones px4_6 releases, and stay outside 2.5 m.

    px4_4 at 80 s is not this case: it comes to rest 2.49 m from land_4, inside
    the safety floor, so that task is unreachable the same way two-vehicle
    seed 10 is. Right-of-way cannot finish a point it is forbidden to visit.
    """
    fleet = make_fleet(10)
    scenario = H.Scenario(seed=0, dropout_vehicle="px4_6", dropout_time_s=55.0, speed_mps=1.0)
    waiting_holds = {"n": 0}

    def observe(c, tick, now):
        if 40.0 <= now < 55.0:
            holds = sum(
                1 for v, cmd in c.last_commands.items()
                if cmd.get("action") == "hold" and c.routes.get(v) and v not in c.retired
            )
            waiting_holds["n"] = max(waiting_holds["n"], holds)

    result = H.run_scenario(scenario, [], fleet=fleet, observe=observe)
    c = result.coordinator
    assert result.dropout_fired
    assert "px4_6" in c.retired
    # The fault has to land in the actual wait, not before the swarm has met.
    assert waiting_holds["n"] >= 2, waiting_holds
    _assert_finished(result)


def test_an_unresolvable_stall_is_reported_before_the_timeout():
    """Two-vehicle seed 10 is a physical infeasibility: the grounded drone sits
    on a task point, so right-of-way cannot finish the mission. The coordinator
    must say so well before the 330 s timeout, and must still abort with the
    timeout rather than flying into the grounded drone."""
    seen: dict = {}

    def observe(c, tick, now):
        if c.deadlock and "at" not in seen:
            seen["at"] = now
            seen["text"] = c.deadlock

    result = H.run_scenario(H.Scenario.from_seed(10), [], observe=observe)
    c = result.coordinator
    assert result.final_phase == "ABORTED"
    assert c.reason == "mission timeout / blocked route"
    assert seen.get("text"), "stall was never reported"
    assert seen["at"] < c.started + c.timeout - 60
    assert result.min_separation_m > 2.5
