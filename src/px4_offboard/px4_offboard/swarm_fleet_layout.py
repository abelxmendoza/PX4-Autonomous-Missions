"""Deterministic N-vehicle layouts on the existing obstacle course.

Each vehicle gets a home on the south launch line, one gate to visit and a landing
pad at the far end; its tasks are ``gate_i`` then ``land_i`` (3 m above its pad),
the same shape as the original two-vehicle mission in ``swarm_logic.DEFAULT_FLEET``.

Slots are hand-placed, not optimised, from this geometry (``test_swarm_fleet_layout``
checks every number):

* fence: north -5..55, east -6..18;
* GPS keep-out: east -8..8 between north 15.5 and 31.5, planned around with 1.5 m
  clearance, so the only way north is a corridor at east ~9.5..17.5;
* obstacles OB1 (10,-6), OB2 (10,10), OB4 (24,6) near the south half, OB5 (38,0)
  in front of the pads;
* moving vehicles keep 3.5 m (``SurveyCoordinator.reservation``) from each other,
  so homes, gates and pads are all spaced further apart than that.

Slot lists are ordered so that a small fleet is a prefix of a larger one: adding a
drone never moves the drones already placed.
"""

from __future__ import annotations

from .swarm_logic import Fleet, FleetTask, Point

# South launch line, two rows 4.5 m apart, 4 m between columns.
_HOMES: tuple[tuple[float, float], ...] = (
    (-3.0, -4.0), (-3.0, 12.0), (-3.0, 4.0), (-3.0, 16.0), (-3.0, 0.0), (-3.0, 8.0),
    (1.5, -4.0), (1.5, 12.0), (1.5, 4.0), (1.5, 16.0), (1.5, 0.0), (1.5, 8.0),
)
# Corridor east of the GPS keep-out (two columns), plus a row north of it beside OB5.
_GATES: tuple[tuple[float, float], ...] = (
    (22.0, 16.0), (32.0, 11.5), (17.0, 11.5), (27.0, 16.0), (36.0, 6.5), (22.0, 11.5),
    (32.0, 16.0), (17.0, 16.0), (27.0, 11.5), (36.0, 11.5), (36.0, 16.0),
)
# Far pad field: three rows 5.5 m apart, 6 m between columns, clear of OB5.
_PADS: tuple[tuple[float, float], ...] = (
    (48.5, -4.0), (48.5, 8.0), (48.5, 2.0), (48.5, 14.0),
    (43.0, -4.0), (43.0, 8.0), (43.0, 2.0), (43.0, 14.0),
    (54.0, -4.0), (54.0, 8.0), (54.0, 2.0), (54.0, 14.0),
)

MAX_VEHICLES = min(len(_HOMES), len(_GATES), len(_PADS))
CRUISE_DOWN_M = -3.0  # NED: 3 m above ground, the original mission's task altitude


def make_fleet(n: int) -> Fleet:
    """The first ``n`` slots of each kind; vehicles are ``px4_1`` .. ``px4_n``."""
    if not 1 <= n <= MAX_VEHICLES:
        raise ValueError(f"fleet size must be 1..{MAX_VEHICLES}, got {n}")
    homes: dict[str, Point] = {}
    landings: dict[str, Point] = {}
    tasks: list[FleetTask] = []
    for i in range(1, n + 1):
        v = f"px4_{i}"
        hn, he = _HOMES[i - 1]
        gn, ge = _GATES[i - 1]
        pn, pe = _PADS[i - 1]
        homes[v] = (hn, he, 0.0)
        landings[v] = (pn, pe, 0.0)
        gate = (gn, ge, CRUISE_DOWN_M)
        pad_above = (pn, pe, CRUISE_DOWN_M)
        tasks.append(FleetTask(f"gate_{i}", v, gate, gate))
        tasks.append(FleetTask(f"land_{i}", v, pad_above, pad_above))
    return Fleet(homes=homes, landings=landings, tasks=tuple(tasks))
