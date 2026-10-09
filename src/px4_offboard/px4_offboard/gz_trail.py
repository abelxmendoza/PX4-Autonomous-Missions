"""Draws the flown path in the Gazebo window (gz-sim's /marker service).

GUI-only by construction (Marker.GUI): the line appears in the Gazebo window but never
in any sensor image, so it cannot affect detection. Points are the drone's own estimate.
"""

from __future__ import annotations

import math
import os
from typing import Sequence

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

TRAIL_RGB = (1.0, 0.36, 0.64)  # the same pink as the web replay's path


class TrailPoints:
    def __init__(self, min_step_m: float = 0.5) -> None:
        self.min_step_m = min_step_m
        self.points: list[tuple[float, float, float]] = []

    def add(self, ned: Sequence[float]) -> bool:
        p = tuple(float(v) for v in ned)
        if self.points and math.dist(p, self.points[-1]) < self.min_step_m:
            return False
        self.points.append(p)
        return True


def build_trail_marker(points_ned: Sequence[Sequence[float]], marker_id: int = 1,
                       rgb: Sequence[float] = TRAIL_RGB, ns: str = "flight_trail"):
    from gz.msgs10.marker_pb2 import Marker

    if len(points_ned) < 2:
        return None
    m = Marker()
    m.ns = ns
    m.id = marker_id
    m.action = Marker.ADD_MODIFY
    m.type = Marker.LINE_STRIP
    m.visibility = Marker.GUI
    r, g, b = rgb
    for color in (m.material.ambient, m.material.diffuse, m.material.emissive):
        color.r, color.g, color.b, color.a = r, g, b, 1.0
    for n, e, d in points_ned:
        pt = m.point.add()
        pt.x, pt.y, pt.z = float(e), float(n), float(-d)  # NED -> Gazebo ENU
    return m


class GazeboTrail:
    """Keeps one LINE_STRIP marker up to date. Best effort: a failed request is ignored."""

    def __init__(self, min_step_m: float = 0.5) -> None:
        from gz.transport13 import Node

        self._node = Node()
        self.points = TrailPoints(min_step_m)

    def publish(self) -> bool:
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.marker_pb2 import Marker

        m = build_trail_marker(self.points.points)
        if m is None:
            return False
        # The /marker service replies with Empty: asking for Boolean makes every call time
        # out and nothing is drawn (checked against the running GUI with `gz service`).
        ok, _ = self._node.request("/marker", m, Marker, Empty, 500)
        return bool(ok)
