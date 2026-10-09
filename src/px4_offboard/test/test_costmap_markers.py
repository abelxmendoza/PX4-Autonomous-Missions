"""Costmap display: cost bands as merged rectangles (the pure part of the Gazebo markers)."""
import math

import pytest

from px4_offboard.costmap2d import Costmap
from px4_offboard.costmap_markers import BANDS, costmap_bands
from px4_offboard.mission_logic import Obstacle


def _cm():
    cm = Costmap(resolution=0.25, north_min=0.0, north_max=20.0, east_min=-10.0, east_max=10.0,
                 robot_radius_m=0.5, inflation_radius_m=2.0)
    cm.set_obstacles([Obstacle(east=0.0, north=10.0, size_east=2.0, size_north=2.0, height=5.0)])
    return cm


def test_bands_cover_the_obstacle_and_its_inflation_and_nothing_else():
    bands = costmap_bands(_cm(), display_res_m=0.5)
    assert set(bands) == {name for name, *_ in BANDS}

    def covered(name, n, e):
        return any(n0 <= n < n1 and e0 <= e < e1 for n0, e0, n1, e1 in bands[name])

    assert covered("lethal", 10.0, 0.0)
    assert covered("low", 10.0, 2.9) or covered("high", 10.0, 2.9)
    everything = [r for rects in bands.values() for r in rects]
    assert not any(n0 <= 18.0 < n1 and e0 <= 8.0 < e1 for n0, e0, n1, e1 in everything)


def test_runs_are_merged_and_bands_do_not_overlap():
    bands = costmap_bands(_cm(), display_res_m=0.5)
    rects = [r for rs in bands.values() for r in rs]
    area = sum((n1 - n0) * (e1 - e0) for n0, e0, n1, e1 in rects)
    tiles = len(rects)
    # A 2 m square inflated 2 m with rounded corners: 2*2 + 4*(2*2) + pi*2^2 = 32.6 m^2, i.e.
    # about 130 display tiles, drawn with far fewer rectangles.
    assert area == pytest.approx(4.0 + 16.0 + math.pi * 4.0, abs=3.0)
    assert tiles < 60
    for i, a in enumerate(rects):
        for b in rects[i + 1:]:
            assert a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1]
