"""Pure LiDAR processing helpers shared by the ROS node and unit tests."""

from __future__ import annotations

import math


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
    mins = {"front": float("inf"), "left": float("inf"), "right": float("inf")}
    for i, distance in enumerate(ranges):
        if (
            distance < range_min
            or distance > range_max
            or math.isinf(distance)
            or math.isnan(distance)
        ):
            continue
        degrees = math.degrees(angle_min + i * angle_step)
        degrees = (degrees + 180.0) % 360.0 - 180.0
        if abs(degrees) <= front_deg:
            mins["front"] = min(mins["front"], distance)
        elif front_deg < degrees <= side_deg:
            mins["left"] = min(mins["left"], distance)
        elif -side_deg <= degrees < -front_deg:
            mins["right"] = min(mins["right"], distance)

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
