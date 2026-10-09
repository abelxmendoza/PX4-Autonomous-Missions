"""Draw a costmap and a SLAM map in the Gazebo window (gz-sim /marker service).

Like gz_trail.py, every marker is Marker.GUI: visible in the Gazebo window, invisible to
every sensor, so drawing the map cannot feed back into the LiDAR that builds it.

The costmap is shown as flat coloured tiles just above the ground, one marker per cost
band, with each row's runs of equal band merged into one rectangle to keep the message small.
"""

from __future__ import annotations

import os
from typing import Sequence

import numpy as np

from .costmap2d import INSCRIBED, LETHAL, NO_INFO, Costmap

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

# (name, lowest cost, highest cost, RGBA). Same colours as the web replay's costmap layer.
BANDS = (
    ("lethal", LETHAL, LETHAL, (0.86, 0.15, 0.15, 0.85)),
    ("inscribed", INSCRIBED, INSCRIBED, (0.98, 0.45, 0.09, 0.7)),
    ("high", 128, INSCRIBED - 1, (0.98, 0.75, 0.14, 0.5)),
    ("low", 1, 127, (0.13, 0.77, 0.37, 0.3)),
)
MAP_POINT_RGBA = (0.95, 0.95, 0.98, 1.0)


def costmap_bands(cm: Costmap, display_res_m: float = 0.5) -> dict[str, list[tuple[float, float, float, float]]]:
    """Rectangles (north0, east0, north1, east1) per band, on a grid coarsened to
    ``display_res_m`` by taking the highest known cost in each block."""
    k = max(1, int(round(display_res_m / cm.res)))
    costs = cm.costs().astype(np.int32)
    costs = np.where(costs == NO_INFO, 0, costs)  # unknown is not drawn
    rows, cols = -(-cm.rows // k), -(-cm.cols // k)
    padded = np.zeros((rows * k, cols * k), dtype=np.int32)
    padded[:cm.rows, :cm.cols] = costs
    block = padded.reshape(rows, k, cols, k).max(axis=(1, 3))
    step = cm.res * k
    out: dict[str, list[tuple[float, float, float, float]]] = {name: [] for name, *_ in BANDS}
    band_of = np.full(block.shape, -1, dtype=np.int32)
    for idx, (_, lo, hi, _) in enumerate(BANDS):
        band_of[(block >= lo) & (block <= hi)] = idx
    for i in range(rows):
        n0 = cm.north_min + i * step
        row = band_of[i]
        j = 0
        while j < cols:
            b = row[j]
            start = j
            while j < cols and row[j] == b:
                j += 1
            if b >= 0:
                out[BANDS[b][0]].append((n0, cm.east_min + start * step, n0 + step, cm.east_min + j * step))
    return out


def _material(m, rgba: Sequence[float]) -> None:
    for color in (m.material.ambient, m.material.diffuse, m.material.emissive):
        color.r, color.g, color.b, color.a = rgba


def build_costmap_markers(cm: Costmap, height_m: float = 0.06, display_res_m: float = 0.5):
    """One TRIANGLE_LIST Marker per band (NED -> Gazebo ENU: x = east, y = north)."""
    from gz.msgs10.marker_pb2 import Marker

    markers = []
    for idx, (name, rects) in enumerate(costmap_bands(cm, display_res_m).items()):
        m = Marker()
        m.ns, m.id = "costmap", idx + 1
        m.visibility = Marker.GUI
        if not rects:
            m.action = Marker.DELETE_MARKER
            markers.append(m)
            continue
        m.action, m.type = Marker.ADD_MODIFY, Marker.TRIANGLE_LIST
        _material(m, BANDS[idx][3])
        z = height_m + 0.005 * idx  # stack bands slightly apart: no z-fighting
        for n0, e0, n1, e1 in rects:
            for e, n in ((e0, n0), (e1, n0), (e1, n1), (e0, n0), (e1, n1), (e0, n1)):
                p = m.point.add()
                p.x, p.y, p.z = float(e), float(n), z
        markers.append(m)
    return markers


def build_map_points_marker(points_ne: Sequence[tuple[float, float]], height_m: float):
    """The SLAM map's occupied cells as white points at the scan height."""
    from gz.msgs10.marker_pb2 import Marker

    m = Marker()
    m.ns, m.id = "slam_map", 1
    m.visibility = Marker.GUI
    m.action, m.type = Marker.ADD_MODIFY, Marker.POINTS
    _material(m, MAP_POINT_RGBA)
    m.scale.x = m.scale.y = m.scale.z = 0.2
    for n, e in points_ne:
        p = m.point.add()
        p.x, p.y, p.z = float(e), float(n), float(height_m)
    return m


class GazeboMapDisplay:
    """Best-effort publisher: a failed /marker request never stops the flight."""

    def __init__(self) -> None:
        from gz.transport13 import Node

        self._node = Node()

    def _send(self, marker, timeout_ms: int = 1000) -> bool:
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.marker_pb2 import Marker

        ok, _ = self._node.request("/marker", marker, Marker, Empty, timeout_ms)
        return bool(ok)

    def available(self) -> bool:
        """Whether a Gazebo window answers /marker. Headless, the service is still listed
        but nothing replies, and every request would block for its full timeout."""
        from gz.msgs10.marker_pb2 import Marker

        probe = Marker()
        probe.ns, probe.id, probe.action = "costmap_probe", 1, Marker.DELETE_MARKER
        return self._send(probe, timeout_ms=1500)

    def publish(self, cm: Costmap, map_points: Sequence[tuple[float, float]], scan_height_m: float) -> bool:
        ok = all([self._send(m) for m in build_costmap_markers(cm)])
        if map_points:
            ok = self._send(build_map_points_marker(map_points, scan_height_m)) and ok
        return ok
