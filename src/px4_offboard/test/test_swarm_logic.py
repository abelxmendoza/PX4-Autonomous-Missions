from dataclasses import replace
import json
import math

import pytest

from px4_offboard.mission_logic import DEFAULT_OBSTACLE_COURSE, segment_hits_expanded_aabb
from px4_offboard.swarm_logic import (HOMES, SurveyCoordinator, Telemetry, enu_to_ned,
                                     local_to_world, world_to_local, segment_distance)
from px4_offboard.swarm_verify import verify


def sample(vehicle, now=0.0, **kwargs):
    return replace(Telemetry(vehicle, vehicle, int(now * 1000), now, HOMES[vehicle],
                             (0, 0, 0), "WAITING", True, False, True), **kwargs)


def test_frame_roundtrip_with_offset_and_calibration():
    origin = enu_to_ned((8, 2, 0))
    local, calibration = (5, 4, -3), (1, 2, 0.2)
    world = local_to_world(local, origin, calibration)
    assert world == (6, 10, -3.2)
    assert world_to_local(world, origin, calibration) == pytest.approx(local)


@pytest.mark.parametrize("a,b,c,d,distance", [
    ((0, 0, 0), (4, 4, 0), (0, 4, 0), (4, 0, 0), 0),
    ((0, 0, 0), (4, 0, 0), (0, 3, 0), (4, 3, 0), 3),
    ((0, 0, 0), (0, 0, 0), (2, 0, 0), (2, 0, 0), 2),
    ((0, 0, 0), (4, 0, 0), (2, 0, 0), (6, 0, 0), 0),
])
def test_segment_distance(a, b, c, d, distance):
    assert segment_distance(a, b, c, d) == pytest.approx(distance)


def test_stale_telemetry_aborts_without_reassigning():
    c = SurveyCoordinator()
    for v in HOMES:
        c.update(sample(v), 0)
    c.step(0)
    assert c.phase == "STARTING"
    commands = c.step(1)
    assert all(cmd["action"] == "land" for cmd in commands.values())
    assert c.phase == "ABORTED"
    assert c.reassignments == 0
    assert not c.retired


def test_replayed_telemetry_does_not_renew_freshness():
    c = SurveyCoordinator()
    assert c.update(sample("px4_1"), 0)
    assert not c.update(sample("px4_1"), 0.1)
    assert not c.update(sample("px4_2", now=-2), 0)


def test_estimator_invalidity_blocks_start():
    c = SurveyCoordinator()
    c.update(sample("px4_1", valid=False), 0)
    c.update(sample("px4_2"), 0)
    assert all(cmd["action"] == "hold" for cmd in c.step(0).values())
    assert c.phase == "WAITING"


def test_process_restart_aborts():
    c = SurveyCoordinator()
    for v in HOMES:
        c.update(sample(v), 0)
    c.step(0)
    c.update(sample("px4_1", 0.1, session="new"), 0.1)
    assert c.phase == "ABORTED"


def test_route_avoids_the_physical_obstacle_course():
    # A straight line at north=10 from east=2 to east=17 (both inside FENCE)
    # runs directly through OB2's clearance zone in obstacle_world.sdf. The
    # commute-leg planner must detour around it now that the physical course
    # is always avoided, not just retired-vehicle placeholders.
    c = SurveyCoordinator()
    c.update(sample("px4_1", position=(10.0, 2.0, -3.0)), 0)
    c.update(sample("px4_2"), 0)
    route = c._route("px4_1", (10.0, 17.0, -3.0))
    points = [(10.0, 2.0), *[(n, e) for n, e, _ in route]]
    for obstacle in DEFAULT_OBSTACLE_COURSE:
        for start, end in zip(points, points[1:]):
            hit, *_ = segment_hits_expanded_aabb(
                [start[0], start[1], 0.0], [end[0], end[1], 0.0], obstacle, 0.5
            )
            assert not hit, f"route leg {start}->{end} crosses {obstacle}"
    assert route[-1] == (10.0, 17.0, -3.0)
    assert len(route) > 1, "a valid detour needs more than a single direct hop"


def test_airborne_fault_does_not_release_tasks():
    c = SurveyCoordinator()
    for v in HOMES:
        c.update(sample(v), 0)
    c.step(0)
    c.update(sample("px4_1", 0.1, state="READY", landed=False, armed=True), 0.1)
    c.update(sample("px4_2", 0.1, state="LANDING", fault="abort", armed=True, landed=False), 0.1)
    assert c.step(0.1)["px4_1"]["action"] == "hold"
    assert c.reassignments == 0


def simulate(path, dropout=False, speed=1.0):
    """Kinematic integration of the real coordinator; not PX4 flight evidence."""
    from dataclasses import asdict
    c = SurveyCoordinator()
    positions = dict(HOMES)
    velocities = {v: (0, 0, 0) for v in HOMES}
    states = {v: "WAITING" for v in HOMES}
    faults = {v: "" for v in HOMES}
    minimum = math.inf
    with path.open("w") as log:
        log.write(json.dumps({"type": "header", "schema": 1, "homes_ned": HOMES,
                              "expected_tasks": [t.name for t in c.tasks], "minimum_separation_m": 2.5}) + "\n")
        for tick in range(3500):
            now = tick * 0.1
            if dropout and states["px4_2"] == "READY" and not faults["px4_2"]:
                faults["px4_2"] = "test fault"
                states["px4_2"] = "LANDING"
            for v in HOMES:
                c.update(sample(v, now, position=positions[v], velocity=velocities[v], state=states[v],
                                armed=states[v] not in {"WAITING", "LANDED"},
                                landed=states[v] in {"WAITING", "LANDED"}, fault=faults[v]), now)
            commands = c.step(now)
            log.write(json.dumps({"type": "sample", "time": now, **c.report(),
                                  "vehicles": {v: asdict(t) for v, t in c.telemetry.items()}}) + "\n")
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
                positions[v] = tuple(old[i] + (goal[i] - old[i]) * min(1, speed * 0.1 / distance) for i in range(3)) if distance else old
                velocities[v] = tuple((positions[v][i] - old[i]) / 0.1 for i in range(3))
                if states[v] == "TAKEOFF" and math.dist(positions[v], goal) < 0.01:
                    states[v] = "READY"
                if states[v] == "LANDING" and positions[v][2] == 0:
                    states[v] = "LANDED"
            minimum = min(minimum, math.dist(*[p[:2] for p in positions.values()]))
    return c, minimum


@pytest.mark.parametrize("dropout", [False, True])
def test_complete_survey_and_confirmed_dropout_reassignment(tmp_path, dropout):
    path = tmp_path / "evidence.jsonl"
    coordinator, minimum = simulate(path, dropout)
    assert coordinator.phase == "COMPLETE", coordinator.report()
    assert all(t.completed for t in coordinator.tasks)
    assert minimum >= 2.5
    assert coordinator.reassignments == (2 if dropout else 0)
    assert verify(path)["passed"]


def test_verifier_rejects_missing_and_truncated_evidence(tmp_path):
    path = tmp_path / "bad.jsonl"
    path.write_text("")
    assert not verify(path)["passed"]
    simulate(path)
    lines = path.read_text().splitlines()
    path.write_text("\n".join(lines[:20]))
    assert not verify(path)["passed"]


def test_recovery_budget_allows_measured_sitl_tracking_speed(tmp_path):
    # The initial 180 s budget terminated recovery after three lanes in SITL.
    # Approximate the observed 0.45 m/s tracking pace, not ideal setpoint speed.
    path = tmp_path / "slow_recovery.jsonl"
    coordinator, minimum = simulate(path, dropout=True, speed=0.45)
    assert coordinator.phase == "COMPLETE", coordinator.report()
    assert coordinator.reassignments == 2
    assert minimum >= 2.5
    assert verify(path)["passed"]
