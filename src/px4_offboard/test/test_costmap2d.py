"""Layered costmap: graded inflation, SLAM-map and other-drone layers, and cost-aware A*."""
import math

import numpy as np
import pytest

from px4_offboard.costmap2d import INSCRIBED, LETHAL, NO_INFO, Costmap, plan_on_costmap
from px4_offboard.lidar_sim import Box, scan_angles, simulate_scan
from px4_offboard.mission_logic import Obstacle
from px4_offboard.slam2d import OccupancyGrid

BOUNDS = dict(north_min=0.0, north_max=40.0, east_min=-20.0, east_max=20.0)


def _costmap(**kw):
    return Costmap(resolution=0.25, robot_radius_m=0.6, inflation_radius_m=3.0, **BOUNDS, **kw)


def _cells_along(cm, path, start, step=0.05):
    pts = [start] + list(path)
    out = []
    for a, b in zip(pts, pts[1:]):
        n = max(1, int(math.dist(a, b) / step))
        out += [cm.cost_at(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n + 1)]
    return out


def test_inflation_is_lethal_on_the_obstacle_inscribed_near_it_then_decays_to_free():
    cm = _costmap()
    cm.set_obstacles([Obstacle(east=0.0, north=20.0, size_east=2.0, size_north=2.0, height=10.0)])
    assert cm.cost_at(20.0, 0.0) == LETHAL
    assert cm.cost_at(20.0, 1.0 + 0.4) == INSCRIBED            # inside the drone's radius
    ring = [cm.cost_at(20.0, 1.0 + d) for d in (0.9, 1.4, 2.0, 2.6)]
    assert all(0 < c < INSCRIBED for c in ring)
    assert ring == sorted(ring, reverse=True)                  # strictly graded, not binary
    assert len(set(ring)) == len(ring)
    assert cm.cost_at(20.0, 1.0 + 3.5) == 0                    # beyond the inflation radius


def _two_gap_wall():
    # A wall across the field at north 20 with a narrow gap on the direct line (east -1 to 1:
    # passable for a 0.6 m-radius drone, but close to both sides) and a wide gap off to one
    # side (east 4 to 12).
    return [Obstacle(east=-10.5, north=20.0, size_east=19.0, size_north=1.0, height=10.0),
            Obstacle(east=2.5, north=20.0, size_east=3.0, size_north=1.0, height=10.0),
            Obstacle(east=16.0, north=20.0, size_east=8.0, size_north=1.0, height=10.0)]


def test_graded_costs_take_the_wide_gap_where_binary_inflation_squeezes_through_the_narrow_one():
    cm = _costmap()
    cm.set_obstacles(_two_gap_wall())
    start, goal = (5.0, 0.0), (35.0, 0.0)
    binary = plan_on_costmap(cm, start, goal, cost_weight=0.0)
    graded = plan_on_costmap(cm, start, goal, cost_weight=3.0)

    def crossing_east(path):
        pts = [start] + path
        for a, b in zip(pts, pts[1:]):
            if (a[0] - 20.0) * (b[0] - 20.0) <= 0 and a[0] != b[0]:
                return a[1] + (b[1] - a[1]) * (20.0 - a[0]) / (b[0] - a[0])
        raise AssertionError("path never crosses the wall")

    assert abs(crossing_east(binary)) < 0.8          # straight through the narrow gap
    assert 4.0 < crossing_east(graded) < 12.0        # around, through the wide one
    assert max(_cells_along(cm, graded, start)) < max(_cells_along(cm, binary, start))


def test_paths_never_enter_lethal_or_inscribed_cells_and_end_on_the_goal():
    cm = _costmap()
    cm.set_obstacles(_two_gap_wall())
    for weight in (0.0, 3.0):
        path = plan_on_costmap(cm, (5.0, -15.0), (35.0, 15.0), cost_weight=weight)
        assert path[-1] == (35.0, 15.0)
        assert max(_cells_along(cm, path, (5.0, -15.0))) < INSCRIBED


def test_another_drone_is_a_dynamic_obstacle_that_can_be_cleared():
    cm = _costmap()
    start, goal = (5.0, 0.0), (35.0, 0.0)
    assert plan_on_costmap(cm, start, goal) == [goal]                  # open field: straight
    cm.set_dynamic([(20.0, 0.0, 0.6)])                                   # a drone in the way
    path = plan_on_costmap(cm, start, goal)
    pts = [start] + path
    closest = min(math.dist((20.0, 0.0), (a[0] + (b[0] - a[0]) * k / 50, a[1] + (b[1] - a[1]) * k / 50))
                  for a, b in zip(pts, pts[1:]) for k in range(51))
    assert closest > 0.6 + 0.6                                          # its radius + ours
    cm.set_dynamic([])
    assert plan_on_costmap(cm, start, goal) == [goal]


def test_a_slam_map_becomes_lethal_walls_known_free_space_and_unknown():
    grid = OccupancyGrid(resolution=0.25, **BOUNDS)
    wall = [Box(20.0, 0.0, 1.0, 30.0)]
    angles = scan_angles()
    for pose in [(5.0, -5.0, 0.0), (5.0, 5.0, 0.0), (8.0, 0.0, 0.0)]:
        grid.update(pose, angles, simulate_scan(wall, pose, angles))
    cm = _costmap()
    cm.set_occupancy(grid)
    assert cm.cost_at(19.6, 0.0) == LETHAL               # the wall face, as mapped
    assert cm.cost_at(10.0, 0.0) == 0                    # seen free, away from the wall
    assert cm.cost_at(30.0, 0.0) == NO_INFO              # behind the wall: never seen
    # Unknown space is not assumed free unless asked: the only way north is unseen.
    with pytest.raises(RuntimeError):
        plan_on_costmap(cm, (5.0, 0.0), (30.0, 18.0), allow_unknown=False)
    assert plan_on_costmap(cm, (5.0, 0.0), (12.0, 10.0), allow_unknown=False)[-1] == (12.0, 10.0)


def test_layers_combine_and_the_grid_is_exported_for_display():
    cm = _costmap()
    cm.set_obstacles([Obstacle(east=-10.0, north=10.0, size_east=2.0, size_north=2.0, height=10.0)])
    cm.set_dynamic([(30.0, 10.0, 0.6)])
    costs = cm.costs()
    assert costs.dtype == np.uint8 and costs.shape == (160, 160)
    assert cm.cost_at(10.0, -10.0) == LETHAL and cm.cost_at(30.0, 10.0) == LETHAL
    doc = cm.to_dict()
    assert doc["resolution"] == 0.25 and doc["rows"] == 160 and doc["cols"] == 160
    assert len(doc["costs_rle"]) % 2 == 0
    flat = np.repeat(doc["costs_rle"][1::2], doc["costs_rle"][0::2])
    assert np.array_equal(flat.reshape(160, 160), costs)


def test_bad_requests_are_refused():
    cm = _costmap()
    cm.set_obstacles([Obstacle(east=0.0, north=20.0, size_east=2.0, size_north=2.0, height=10.0)])
    with pytest.raises(ValueError):
        plan_on_costmap(cm, (20.0, 0.0), (35.0, 0.0))     # starts inside an obstacle
    with pytest.raises(ValueError):
        plan_on_costmap(cm, (5.0, 0.0), (50.0, 0.0))      # goal off the map
    cm.set_obstacles([Obstacle(east=0.0, north=20.0, size_east=40.0, size_north=1.0, height=10.0)])
    with pytest.raises(RuntimeError):
        plan_on_costmap(cm, (5.0, 0.0), (35.0, 0.0))      # wall from edge to edge
