"""Cached traffic providers. ``latest`` does not use the network.

There is no live ADS-B or flight-tracking client. An external feed is not
assumed to be complete, timely, or authoritative.
"""

from __future__ import annotations

from typing import Sequence

from .traffic import Reason, TrafficFetch, TrafficSnapshot
from .weather import Provenance


class StaticTrafficProvider:
    def __init__(self, snapshot: TrafficSnapshot | None = None, *, failure: str | None = None) -> None:
        self._snapshot = snapshot
        self._failure = failure

    def latest(self, now_s: float) -> TrafficFetch:
        del now_s
        if self._failure:
            return TrafficFetch(None, Reason.TRAFFIC_SOURCE_UNAVAILABLE, self._failure)
        if self._snapshot is None:
            return TrafficFetch(None, Reason.TRAFFIC_DATA_MISSING, "no snapshot")
        return TrafficFetch(self._snapshot, "")


class SimulatedTrafficProvider:
    """Segments are ``(start_s, snapshot)``. ``None`` is a missing snapshot."""

    def __init__(self, segments: Sequence[tuple[float, TrafficSnapshot | None]]) -> None:
        if not segments:
            raise ValueError("simulated traffic needs at least one segment")
        self._segments = tuple(sorted(segments, key=lambda item: item[0]))

    def latest(self, now_s: float) -> TrafficFetch:
        active = _active(self._segments, now_s)
        if active is None or active[1] is None:
            return TrafficFetch(None, Reason.TRAFFIC_DATA_MISSING, "no snapshot")
        return TrafficFetch(active[1], "")


class RecordedTrafficReplayProvider:
    """Replays timestamped snapshots. Each snapshot must be provenance ``recorded``."""

    def __init__(self, segments: Sequence[tuple[float, TrafficSnapshot | None]]) -> None:
        if not segments:
            raise ValueError("recorded traffic replay needs at least one segment")
        self._segments = tuple(sorted(segments, key=lambda item: item[0]))

    def latest(self, now_s: float) -> TrafficFetch:
        active = _active(self._segments, now_s)
        if active is None or active[1] is None:
            return TrafficFetch(None, Reason.TRAFFIC_DATA_MISSING, "no snapshot")
        snapshot = active[1]
        if snapshot.provenance is not Provenance.RECORDED:
            return TrafficFetch(None, Reason.TRAFFIC_SOURCE_UNAVAILABLE, "replay requires recorded provenance")
        return TrafficFetch(snapshot, "")


def _active(
    segments: Sequence[tuple[float, TrafficSnapshot | None]],
    now_s: float,
) -> tuple[float, TrafficSnapshot | None] | None:
    chosen: tuple[float, TrafficSnapshot | None] | None = None
    for segment in segments:
        if segment[0] <= now_s:
            chosen = segment
    return chosen
