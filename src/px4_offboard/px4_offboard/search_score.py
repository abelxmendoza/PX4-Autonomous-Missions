"""Score a search report against the world's ground truth (verifier side only).

The drones never read the truth file; this runs after the flight.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class SearchScore:
    found: dict[int, float]        # id -> horizontal position error (m)
    missed: list[int]              # in the truth, not reported
    false_ids: list[int]           # reported, not in the truth
    max_error_m: float | None
    mean_error_m: float | None

    def passed(self, max_error_m: float) -> bool:
        return not self.missed and not self.false_ids and (self.max_error_m or 0.0) <= max_error_m


def score(report: dict, truth: dict) -> SearchScore:
    truth_by_id = {t["id"]: (t["north"], t["east"]) for t in truth["targets"]}
    reported = {t["id"]: (t["north"], t["east"]) for t in report["targets"]}
    found = {mid: math.dist(reported[mid], truth_by_id[mid]) for mid in sorted(reported) if mid in truth_by_id}
    errors = list(found.values())
    return SearchScore(
        found=found,
        missed=sorted(set(truth_by_id) - set(reported)),
        false_ids=sorted(set(reported) - set(truth_by_id)),
        max_error_m=max(errors) if errors else None,
        mean_error_m=sum(errors) / len(errors) if errors else None,
    )
