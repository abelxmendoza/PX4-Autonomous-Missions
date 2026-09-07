"""Deterministic 2-D A* planning for the known simulation obstacle map."""

from __future__ import annotations

import heapq
import math
from typing import Sequence

from .mission_logic import Fence, Obstacle, segment_hits_expanded_aabb


Point = tuple[float, float]  # north, east
Cell = tuple[int, int]


def _blocked(point: Point, obstacles: Sequence[Obstacle], clearance_m: float) -> bool:
    north, east = point
    return any(
        abs(north - obstacle.north) <= obstacle.size_north / 2.0 + clearance_m
        and abs(east - obstacle.east) <= obstacle.size_east / 2.0 + clearance_m
        for obstacle in obstacles
    )


def _visible(a: Point, b: Point, obstacles: Sequence[Obstacle], clearance_m: float) -> bool:
    start = [a[0], a[1], 0.0]
    end = [b[0], b[1], 0.0]
    return not any(
        segment_hits_expanded_aabb(start, end, obstacle, clearance_m)[0]
        for obstacle in obstacles
    )


def simplify_path(
    points: Sequence[Point], obstacles: Sequence[Obstacle], clearance_m: float
) -> list[Point]:
    """Remove grid-scale turns while retaining collision-free line segments."""
    if len(points) <= 2:
        return list(points)
    result = [points[0]]
    anchor = 0
    while anchor < len(points) - 1:
        candidate = len(points) - 1
        while candidate > anchor + 1 and not _visible(
            points[anchor], points[candidate], obstacles, clearance_m
        ):
            candidate -= 1
        result.append(points[candidate])
        anchor = candidate
    return result


def plan_path(
    start: Point,
    goal: Point,
    obstacles: Sequence[Obstacle],
    fence: Fence,
    *,
    resolution_m: float = 1.0,
    clearance_m: float = 2.0,
    fence_margin_m: float = 1.0,
) -> list[Point]:
    """Plan and simplify an eight-connected route inside ``fence``.

    The returned path excludes the start and always includes the exact goal.
    Raises ``ValueError`` when inputs are invalid and ``RuntimeError`` when no
    route exists.
    """
    if resolution_m <= 0.0:
        raise ValueError("resolution_m must be positive")
    n_min = fence.north_min + fence_margin_m
    n_max = fence.north_max - fence_margin_m
    e_min = fence.east_min + fence_margin_m
    e_max = fence.east_max - fence_margin_m
    if n_min >= n_max or e_min >= e_max:
        raise ValueError("fence margin leaves no planning area")

    def to_cell(point: Point) -> Cell:
        return (
            round((point[0] - n_min) / resolution_m),
            round((point[1] - e_min) / resolution_m),
        )

    def to_point(cell: Cell) -> Point:
        return (n_min + cell[0] * resolution_m, e_min + cell[1] * resolution_m)

    max_i = math.floor((n_max - n_min) / resolution_m)
    max_j = math.floor((e_max - e_min) / resolution_m)
    start_cell, goal_cell = to_cell(start), to_cell(goal)

    def valid(cell: Cell) -> bool:
        i, j = cell
        return 0 <= i <= max_i and 0 <= j <= max_j and not _blocked(
            to_point(cell), obstacles, clearance_m
        )

    if not valid(start_cell) or not valid(goal_cell):
        raise ValueError("start or goal lies outside free planning space")

    frontier: list[tuple[float, float, Cell]] = [(0.0, 0.0, start_cell)]
    cost = {start_cell: 0.0}
    parent: dict[Cell, Cell] = {}
    moves = ((1, 0), (-1, 0), (0, 1), (0, -1),
             (1, 1), (1, -1), (-1, 1), (-1, -1))

    while frontier:
        _, current_cost, current = heapq.heappop(frontier)
        if current == goal_cell:
            break
        if current_cost > cost[current] + 1e-9:
            continue
        for di, dj in moves:
            nxt = (current[0] + di, current[1] + dj)
            if not valid(nxt):
                continue
            if di and dj:
                # Prevent diagonal corner cutting between inflated obstacles.
                if not valid((current[0] + di, current[1])) or not valid(
                    (current[0], current[1] + dj)
                ):
                    continue
            step = math.hypot(di, dj) * resolution_m
            new_cost = current_cost + step
            if new_cost >= cost.get(nxt, math.inf):
                continue
            cost[nxt] = new_cost
            parent[nxt] = current
            heuristic = math.hypot(goal_cell[0] - nxt[0], goal_cell[1] - nxt[1])
            heapq.heappush(frontier, (new_cost + heuristic * resolution_m, new_cost, nxt))

    if goal_cell not in cost:
        raise RuntimeError("no collision-free path found")

    cells = [goal_cell]
    while cells[-1] != start_cell:
        cells.append(parent[cells[-1]])
    cells.reverse()
    grid_points = [start] + [to_point(cell) for cell in cells[1:-1]] + [goal]
    return simplify_path(grid_points, obstacles, clearance_m)[1:]


def plan_path_via(
    start: Point,
    checkpoints: Sequence[Point],
    obstacles: Sequence[Obstacle],
    fence: Fence,
    **kwargs,
) -> list[Point]:
    """Plan each mission leg while preserving intentional checkpoint turns."""
    route: list[Point] = []
    leg_start = start
    for checkpoint in checkpoints:
        leg = plan_path(leg_start, checkpoint, obstacles, fence, **kwargs)
        route.extend(leg)
        leg_start = checkpoint
    return route
