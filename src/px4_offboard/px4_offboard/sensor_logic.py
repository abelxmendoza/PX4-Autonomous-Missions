"""Pure LiDAR processing helpers shared by the ROS node and unit tests.

Hot path uses precomputed sector index bands so each scan is O(R) mins
without per-beam atan/wrap. Optional EMA smooths sector range spikes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


def _wrap_deg(degrees: float) -> float:
    return (degrees + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class SectorBands:
    """Index membership for front/left/right beams (fixed after geometry settles)."""

    front: tuple[int, ...]
    left: tuple[int, ...]
    right: tuple[int, ...]
    key: tuple

    @classmethod
    def build(
        cls,
        count: int,
        angle_min: float,
        angle_step: float,
        front_deg: float,
        side_deg: float,
    ) -> "SectorBands":
        front: list[int] = []
        left: list[int] = []
        right: list[int] = []
        for i in range(count):
            degrees = _wrap_deg(math.degrees(angle_min + i * angle_step))
            if abs(degrees) <= front_deg:
                front.append(i)
            elif front_deg < degrees <= side_deg:
                left.append(i)
            elif -side_deg <= degrees < -front_deg:
                right.append(i)
        return cls(
            front=tuple(front),
            left=tuple(left),
            right=tuple(right),
            key=(count, angle_min, angle_step, front_deg, side_deg),
        )


_BAND_CACHE: dict[tuple, SectorBands] = {}


def get_sector_bands(
    count: int,
    angle_min: float,
    angle_step: float,
    front_deg: float,
    side_deg: float,
) -> SectorBands:
    key = (count, angle_min, angle_step, front_deg, side_deg)
    bands = _BAND_CACHE.get(key)
    if bands is None:
        bands = SectorBands.build(count, angle_min, angle_step, front_deg, side_deg)
        _BAND_CACHE[key] = bands
    return bands


def _band_min(
    ranges: list[float],
    indices: tuple[int, ...],
    range_min: float,
    range_max: float,
) -> float:
    best = float("inf")
    for i in indices:
        distance = ranges[i]
        if (
            distance < range_min
            or distance > range_max
            or math.isinf(distance)
            or math.isnan(distance)
        ):
            continue
        if distance < best:
            best = distance
    return best


def sector_from_scan(
    ranges: list[float],
    angle_min: float,
    angle_step: float,
    range_min: float,
    range_max: float,
    trigger_m: float,
    front_deg: float,
    side_deg: float,
    side_trigger_m: float | None = None,
) -> tuple[str | None, dict[str, float]]:
    """Return the nearest actionable sector and raw minimum ranges."""
    bands = get_sector_bands(len(ranges), angle_min, angle_step, front_deg, side_deg)
    mins = {
        "front": _band_min(ranges, bands.front, range_min, range_max),
        "left": _band_min(ranges, bands.left, range_min, range_max),
        "right": _band_min(ranges, bands.right, range_min, range_max),
    }

    side_limit = trigger_m if side_trigger_m is None else side_trigger_m
    hits = {
        sector: distance
        for sector, distance in mins.items()
        if distance < (trigger_m if sector == "front" else side_limit)
    }
    if not hits:
        return None, mins
    label = min(
        hits.items(), key=lambda item: (item[1], 0 if item[0] == "front" else 1)
    )[0]
    return label, mins


@dataclass
class SectorEma:
    """First-order low-pass on sector range mins (ignore invalid / inf)."""

    alpha: float = 0.4
    values: dict[str, float] = field(
        default_factory=lambda: {
            "front": float("inf"),
            "left": float("inf"),
            "right": float("inf"),
        }
    )

    def update(self, raw: dict[str, float]) -> dict[str, float]:
        a = min(1.0, max(0.0, self.alpha))
        for key in ("front", "left", "right"):
            sample = raw.get(key, float("inf"))
            if not math.isfinite(sample) or sample >= 1e8:
                continue
            previous = self.values[key]
            if not math.isfinite(previous) or previous >= 1e8:
                self.values[key] = sample
            else:
                self.values[key] = a * sample + (1.0 - a) * previous
        return dict(self.values)
