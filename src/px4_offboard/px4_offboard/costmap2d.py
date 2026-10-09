"""Layered 2-D costmap with graded inflation, and an A* planner that weighs cost, not just walls.

The repo's original planner (path_planner.py) treats space as binary: a cell is blocked if it
is within ``clearance_m`` of an obstacle, otherwise free, so it will happily thread a gap that
is just wide enough. A costmap grades the space instead (Nav2's convention, values 0-255):

    LETHAL (254)     the cell touches an obstacle
    INSCRIBED (253)  the drone's centre here would put its body into an obstacle
    1..252           inflation: decays exponentially with clearance out to the inflation radius
    FREE (0)         clear of everything
    NO_INFO (255)    never observed (only when a SLAM map layer is in use)

Layers, combined by taking the union of their lethal cells before inflating:
    obstacles  known boxes (the mission's obstacle list)
    occupancy  a SLAM OccupancyGrid: occupied cells lethal, seen-free cells known, the rest unknown
    dynamic    other drones as discs, replaced wholesale on every update

``plan_on_costmap`` runs eight-connected A* where a step costs its length times
``1 + cost_weight * cost / 252``: with ``cost_weight=0`` it reproduces binary inflation; with a
positive weight it trades path length for clearance.
"""

from __future__ import annotations

import heapq
import math
from typing import Iterable, Sequence

import cv2
import numpy as np

FREE = 0
INSCRIBED = 253
LETHAL = 254
NO_INFO = 255
MAX_INFLATED = 252

Point = tuple[float, float]  # north, east


class Costmap:
    def __init__(self, resolution: float, north_min: float, north_max: float, east_min: float,
                 east_max: float, robot_radius_m: float = 0.5, inflation_radius_m: float = 3.0,
                 decay_per_m: float = 1.0) -> None:
        if resolution <= 0 or north_max <= north_min or east_max <= east_min:
            raise ValueError("costmap needs a positive resolution and non-empty bounds")
        self.res = resolution
        self.north_min, self.north_max = north_min, north_max
        self.east_min, self.east_max = east_min, east_max
        self.rows = int(math.ceil((north_max - north_min) / resolution))
        self.cols = int(math.ceil((east_max - east_min) / resolution))
        self.robot_radius_m = robot_radius_m
        self.inflation_radius_m = inflation_radius_m
        self.decay_per_m = decay_per_m
        shape = (self.rows, self.cols)
        self._obstacles = np.zeros(shape, dtype=bool)
        self._occupied = np.zeros(shape, dtype=bool)
        self._known = np.ones(shape, dtype=bool)  # everything counts as known until a SLAM layer says otherwise
        self._dynamic = np.zeros(shape, dtype=bool)
        self._costs: np.ndarray | None = None
        # Cell edges and centres, for rasterising.
        self._n0 = north_min + np.arange(self.rows) * resolution
        self._e0 = east_min + np.arange(self.cols) * resolution

    # --- layers ------------------------------------------------------------------------
    def set_obstacles(self, obstacles: Iterable) -> None:
        """Known axis-aligned boxes (anything with north, east, size_north, size_east)."""
        layer = np.zeros_like(self._obstacles)
        for ob in obstacles:
            rows = (self._n0 < ob.north + ob.size_north / 2) & (self._n0 + self.res > ob.north - ob.size_north / 2)
            cols = (self._e0 < ob.east + ob.size_east / 2) & (self._e0 + self.res > ob.east - ob.size_east / 2)
            layer |= rows[:, None] & cols[None, :]
        self._obstacles = layer
        self._costs = None

    def set_occupancy(self, grid, occupied: float = 0.65, free: float = 0.35) -> None:
        """Take a SLAM OccupancyGrid: occupied -> lethal, seen free -> known, otherwise unknown."""
        cn = self._n0 + self.res / 2
        ce = self._e0 + self.res / 2
        i = np.floor((cn - grid.north_min) / grid.res).astype(np.int64)
        j = np.floor((ce - grid.east_min) / grid.res).astype(np.int64)
        prob = np.full((self.rows, self.cols), 0.5)
        ok_i, ok_j = (i >= 0) & (i < grid.rows), (j >= 0) & (j < grid.cols)
        p = grid.probabilities()
        prob[np.ix_(ok_i, ok_j)] = p[np.ix_(i[ok_i], j[ok_j])]
        self._occupied = prob > occupied
        self._known = self._occupied | (prob < free)
        self._costs = None

    def set_dynamic(self, discs: Iterable[tuple[float, float, float]]) -> None:
        """Other drones as (north, east, radius) discs; replaces the previous set."""
        layer = np.zeros_like(self._dynamic)
        for n, e, r in discs:
            # A cell is lethal if any part of it lies inside the disc.
            dn = np.maximum(np.maximum(self._n0 - n, n - (self._n0 + self.res)), 0.0)
            de = np.maximum(np.maximum(self._e0 - e, e - (self._e0 + self.res)), 0.0)
            layer |= (dn[:, None] ** 2 + de[None, :] ** 2) <= r * r
        self._dynamic = layer
        self._costs = None

    # --- combined costs --------------------------------------------------------------------
    def costs(self) -> np.ndarray:
        if self._costs is None:
            lethal = self._obstacles | self._occupied | self._dynamic
            if lethal.any():
                src = np.where(lethal, 0, 255).astype(np.uint8)
                # Centre-to-centre distance less half a cell: clearance to the lethal cell's edge.
                d = cv2.distanceTransform(src, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * self.res
                d = np.maximum(d - self.res / 2, 0.0)
            else:
                d = np.full(lethal.shape, np.inf)
            graded = MAX_INFLATED * np.exp(-self.decay_per_m * (d - self.robot_radius_m))
            cost = np.where(d <= self.inflation_radius_m, np.clip(np.rint(graded), 1, MAX_INFLATED), FREE)
            cost = np.where(d <= self.robot_radius_m, INSCRIBED, cost)
            cost = np.where(lethal, LETHAL, cost)
            cost = np.where(~self._known & (cost < INSCRIBED), NO_INFO, cost)
            self._costs = cost.astype(np.uint8)
        return self._costs

    def cell(self, north: float, east: float) -> tuple[int, int] | None:
        i = int(math.floor((north - self.north_min) / self.res))
        j = int(math.floor((east - self.east_min) / self.res))
        if 0 <= i < self.rows and 0 <= j < self.cols:
            return i, j
        return None

    def centre(self, i: int, j: int) -> Point:
        return (self.north_min + (i + 0.5) * self.res, self.east_min + (j + 0.5) * self.res)

    def cost_at(self, north: float, east: float) -> int:
        c = self.cell(north, east)
        return NO_INFO if c is None else int(self.costs()[c])

    def to_dict(self) -> dict:
        """Compact form for the web replay: row-major run-length pairs [count, value, ...]."""
        flat = self.costs().reshape(-1)
        edges = np.flatnonzero(np.diff(flat)) + 1
        starts = np.concatenate([[0], edges])
        counts = np.diff(np.concatenate([starts, [flat.size]]))
        rle = np.column_stack([counts, flat[starts]]).reshape(-1)
        return {"resolution": self.res, "north_min": self.north_min, "east_min": self.east_min,
                "rows": self.rows, "cols": self.cols, "robot_radius_m": self.robot_radius_m,
                "inflation_radius_m": self.inflation_radius_m,
                "legend": {"free": FREE, "inscribed": INSCRIBED, "lethal": LETHAL, "no_info": NO_INFO},
                "costs_rle": [int(v) for v in rle]}


def plan_on_costmap(cm: Costmap, start: Point, goal: Point, *, cost_weight: float = 3.0,
                    allow_unknown: bool = True, unknown_cost: int = 50) -> list[Point]:
    """Cheapest eight-connected route from ``start`` to ``goal``; excludes start, ends on goal.

    Cells at INSCRIBED or LETHAL are never entered, and diagonal steps may not cut the corner
    of one. Unknown cells cost ``unknown_cost`` when ``allow_unknown``, otherwise they are
    closed. Raises ValueError for a start or goal off the map or inside an obstacle, and
    RuntimeError when no route exists.
    """
    costs = cm.costs()
    s, g = cm.cell(*start), cm.cell(*goal)
    if s is None or g is None:
        raise ValueError("start or goal lies outside the costmap")
    if INSCRIBED <= costs[s] <= LETHAL or INSCRIBED <= costs[g] <= LETHAL:
        raise ValueError("start or goal lies inside an obstacle's footprint")

    step_cost = costs.astype(np.float64)
    step_cost[costs == NO_INFO] = unknown_cost
    open_ = (costs < INSCRIBED) | ((costs == NO_INFO) & allow_unknown)
    open_[s] = True
    rows, cols = costs.shape
    moves = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))

    best = {s: 0.0}
    parent: dict[tuple[int, int], tuple[int, int]] = {}
    frontier = [(0.0, 0.0, s)]
    while frontier:
        _, so_far, cur = heapq.heappop(frontier)
        if cur == g:
            break
        if so_far > best[cur] + 1e-9:
            continue
        for di, dj in moves:
            ni, nj = cur[0] + di, cur[1] + dj
            if not (0 <= ni < rows and 0 <= nj < cols) or not open_[ni, nj]:
                continue
            if di and dj and not (open_[cur[0] + di, cur[1]] and open_[cur[0], cur[1] + dj]):
                continue
            new = so_far + math.hypot(di, dj) * cm.res * (1.0 + cost_weight * step_cost[ni, nj] / MAX_INFLATED)
            if new >= best.get((ni, nj), math.inf):
                continue
            best[(ni, nj)] = new
            parent[(ni, nj)] = cur
            h = math.hypot(g[0] - ni, g[1] - nj) * cm.res
            heapq.heappush(frontier, (new + h, new, (ni, nj)))
    if g not in best:
        raise RuntimeError("no route through the costmap")

    cells = [g]
    while cells[-1] != s:
        cells.append(parent[cells[-1]])
    cells.reverse()
    points = [start] + [cm.centre(*c) for c in cells[1:-1]] + [goal]
    worst = [int(costs[s])] + [int(costs[c]) for c in cells[1:]]
    return _simplify(cm, points, worst, open_)[1:]


def _simplify(cm: Costmap, points: Sequence[Point], worst: Sequence[int], open_: np.ndarray) -> list[Point]:
    """Drop grid zig-zags: replace a run of points by a straight segment when that segment
    stays in open cells and never sees a higher cost than the run it replaces."""
    costs = cm.costs()

    def segment_ok(a: int, b: int) -> bool:
        limit = max(worst[a:b + 1])
        pa, pb = points[a], points[b]
        n = max(1, int(math.ceil(math.dist(pa, pb) / (cm.res / 8))))
        for k in range(n + 1):
            c = cm.cell(pa[0] + (pb[0] - pa[0]) * k / n, pa[1] + (pb[1] - pa[1]) * k / n)
            if c is None or not open_[c] or costs[c] > limit:
                return False
        return True

    out, anchor = [points[0]], 0
    while anchor < len(points) - 1:
        nxt = len(points) - 1
        while nxt > anchor + 1 and not segment_ok(anchor, nxt):
            nxt -= 1
        out.append(points[nxt])
        anchor = nxt
    return out
