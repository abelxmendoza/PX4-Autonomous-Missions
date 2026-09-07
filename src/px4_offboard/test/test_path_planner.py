import math

import pytest

from px4_offboard.mission_logic import Fence, Obstacle, segment_hits_expanded_aabb
from px4_offboard.path_planner import plan_path, plan_path_via


FENCE = Fence(-5.0, 55.0, -23.0, 17.0, 12.0)


def test_direct_path_has_only_goal():
    assert plan_path((0.0, 0.0), (10.0, 0.0), (), FENCE) == [(10.0, 0.0)]


def test_route_goes_around_inflated_obstacle_and_is_simplified():
    obstacle = Obstacle(0.0, 5.0, 3.0, 3.0, 10.0)
    route = plan_path((0.0, 0.0), (10.0, 0.0), (obstacle,), FENCE, clearance_m=2.0)
    assert route[-1] == (10.0, 0.0)
    assert 2 <= len(route) <= 4
    points = [(0.0, 0.0), *route]
    assert all(
        not segment_hits_expanded_aabb(
            [a[0], a[1], 0.0], [b[0], b[1], 0.0], obstacle, 2.0
        )[0]
        for a, b in zip(points, points[1:])
    )


def test_no_path_raises():
    wall = Obstacle(0.0, 25.0, 100.0, 2.0, 10.0)
    with pytest.raises(RuntimeError):
        plan_path((0.0, 0.0), (50.0, 0.0), (wall,), FENCE)


def test_invalid_resolution_raises():
    with pytest.raises(ValueError):
        plan_path((0.0, 0.0), (10.0, 0.0), (), FENCE, resolution_m=0.0)


def test_route_is_shorter_than_old_hand_authored_course():
    obstacles = (
        Obstacle(-6.0, 10.0, 3.0, 3.0, 11.5),
        Obstacle(10.0, 10.0, 3.0, 3.0, 6.0),
        Obstacle(-8.0, 24.0, 5.0, 3.0, 4.0),
        Obstacle(6.0, 24.0, 3.0, 2.0, 5.0),
        Obstacle(0.0, 38.0, 5.0, 3.0, 11.5),
    )
    route = plan_path((0.0, 0.0), (50.0, 0.0), obstacles, FENCE)
    points = [(0.0, 0.0), *route]
    length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    assert length < 60.0


def test_checkpoint_route_preserves_mission_turns():
    route = plan_path_via(
        (0.0, 0.0), ((15.0, -6.0), (31.0, 0.0), (50.0, 0.0)), (), FENCE
    )
    assert (15.0, -6.0) in route
    assert (31.0, 0.0) in route
    assert route[-1] == (50.0, 0.0)


def test_world_checkpoint_route_has_seven_legs_around_tall_obstacles():
    obstacles = (
        Obstacle(-6.0, 10.0, 3.0, 3.0, 11.5),
        Obstacle(10.0, 10.0, 3.0, 3.0, 6.0),
        Obstacle(-8.0, 24.0, 5.0, 3.0, 4.0),
        Obstacle(6.0, 24.0, 3.0, 2.0, 5.0),
        Obstacle(0.0, 38.0, 5.0, 3.0, 11.5),
    )
    route = plan_path_via(
        (0.0, 0.0),
        ((15.0, -6.0), (31.0, 0.0), (50.0, 0.0)),
        obstacles,
        FENCE,
        clearance_m=2.0,
        fence_margin_m=1.0,
    )
    assert len(route) == 7
    points = [(0.0, 0.0), *route]
    for obstacle in (obstacles[0], obstacles[4]):
        for start, end in zip(points, points[1:]):
            hit = segment_hits_expanded_aabb(
                [start[0], start[1], 0.0],
                [end[0], end[1], 0.0],
                obstacle,
                2.0,
            )[0]
            assert not hit
