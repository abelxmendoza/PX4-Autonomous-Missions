"""Generated N-vehicle layouts must be physically valid before any mission runs on them."""
from __future__ import annotations

import itertools
import math

import pytest

from px4_offboard.mission_logic import DEFAULT_OBSTACLE_COURSE
from px4_offboard.swarm_fleet_layout import MAX_VEHICLES, make_fleet
from px4_offboard.swarm_logic import FENCE, GPS_DENIED_ZONE, SurveyCoordinator

SIZES = (1, 2, 3, 5, 10)
CLEARANCE = SurveyCoordinator().obstacle_clearance  # what the planner keeps from obstacles
RESERVATION = SurveyCoordinator().reservation       # what moving vehicles keep from each other


def _blocked(p, margin):
    return any(abs(p[0] - o.north) <= o.size_north / 2 + margin and abs(p[1] - o.east) <= o.size_east / 2 + margin
               for o in list(DEFAULT_OBSTACLE_COURSE) + [GPS_DENIED_ZONE])


def _min_gap(points):
    return min((math.dist(a[:2], b[:2]) for a, b in itertools.combinations(points, 2)), default=math.inf)


@pytest.mark.parametrize("n", SIZES)
def test_layout_has_one_home_pad_gate_and_land_task_per_vehicle(n):
    f = make_fleet(n)
    assert len(f.homes) == len(f.landings) == n
    assert sorted(t.name for t in f.tasks) == sorted([f"gate_{i}" for i in range(1, n + 1)] +
                                                     [f"land_{i}" for i in range(1, n + 1)])
    for i, v in enumerate(sorted(f.homes, key=lambda v: int(v.split("_")[1])), start=1):
        owned = sorted(t.name for t in f.tasks if t.owner == v)
        assert owned == [f"gate_{i}", f"land_{i}"]


@pytest.mark.parametrize("n", SIZES)
def test_every_point_is_inside_the_fence_and_clear_of_obstacles_and_the_gps_keep_out(n):
    f = make_fleet(n)
    points = list(f.homes.values()) + list(f.landings.values()) + [t.end for t in f.tasks]
    for p in points:
        assert FENCE.north_min + 0.5 <= p[0] <= FENCE.north_max - 0.5, p
        assert FENCE.east_min + 0.5 <= p[1] <= FENCE.east_max - 0.5, p
        assert not _blocked(p, CLEARANCE), f"{p} is inside an obstacle or the GPS keep-out (+{CLEARANCE} m)"


@pytest.mark.parametrize("n", SIZES)
def test_spacing_respects_separation_and_reservation(n):
    f = make_fleet(n)
    gates = [t.end for t in f.tasks if t.name.startswith("gate_")]
    # Homes: vehicles sit there together before takeoff -> beyond the reservation.
    assert _min_gap(list(f.homes.values())) > RESERVATION
    # Pads: landed vehicles are passed by others still arriving -> beyond the reservation.
    assert _min_gap(list(f.landings.values())) > RESERVATION
    # Gates may be visited at the same time -> beyond the reservation too.
    assert _min_gap(gates) > RESERVATION
    # The land task is flown to at 3 m above its pad, exactly.
    for t in f.tasks:
        if t.name.startswith("land_"):
            pad = f.landings["px4_" + t.name.split("_")[1]]
            assert t.end == (pad[0], pad[1], -3.0)


def test_layout_is_deterministic_and_bounded():
    assert make_fleet(5) == make_fleet(5)
    assert MAX_VEHICLES >= 10
    with pytest.raises(ValueError):
        make_fleet(0)
    with pytest.raises(ValueError):
        make_fleet(MAX_VEHICLES + 1)


def test_smaller_fleets_are_prefixes_of_larger_ones():
    # So "adding a drone" never moves the drones that were already there.
    small, big = make_fleet(3), make_fleet(10)
    for v in small.homes:
        assert small.homes[v] == big.homes[v] and small.landings[v] == big.landings[v]
