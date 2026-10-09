"""Records a search flight for the browser replay: the drone's pose (PX4's estimate) at a
fixed rate, every geolocated sighting, and when each target was confirmed.

Contains only what the drone itself knew. Ground truth is added by the viewer, from the
world's truth file, purely for display and scoring.
"""

from __future__ import annotations

import math
from typing import Sequence

SCHEMA = 1


def _yaw_deg(q: Sequence[float]) -> float:
    w, x, y, z = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


class TraceRecorder:
    def __init__(self, rate_hz: float = 10.0) -> None:
        self._period = 1.0 / rate_hz
        self._next_t: float | None = None
        self._last_t = -math.inf
        self.frames: list[dict] = []
        self.sightings: list[dict] = []
        self.confirmations: list[dict] = []

    def pose(self, t: float, position_ned: Sequence[float], q: Sequence[float]) -> None:
        if t < self._last_t:
            raise ValueError(f"time went backwards: {t} < {self._last_t}")
        self._last_t = t
        if self._next_t is not None and t < self._next_t - 1e-9:
            return
        self._next_t = t + self._period
        n, e, d = (round(float(v), 3) for v in position_ned)
        self.frames.append({"t": round(t, 3), "n": n, "e": e, "d": d, "yaw_deg": round(_yaw_deg(q), 2)})

    def sighting(self, t: float, marker_id: int, north: float, east: float) -> None:
        self.sightings.append({"t": round(t, 3), "id": int(marker_id), "n": round(north, 3), "e": round(east, 3)})

    def confirmed(self, t: float, marker_id: int) -> None:
        self.confirmations.append({"t": round(t, 3), "id": int(marker_id)})

    def to_dict(self, world: str = "search_field", **meta) -> dict:
        return {"schema": SCHEMA, "world": world, **meta, "frames": self.frames,
                "sightings": self.sightings, "confirmations": self.confirmations}
