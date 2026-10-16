#!/usr/bin/env python3
"""Summarize mission-verification reports that exist on disk.

Does not reuse evidence/integrated_seeding/sitl_report.json. That file is a
different flight.
"""

from __future__ import annotations

import argparse
import gzip
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "px4_offboard"))
from px4_offboard.mission_verification import summarize_campaign  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign", type=Path, required=True)
    parser.add_argument("--planned", type=Path, default=None, help="JSON object of scenario id to planned count")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    # Committed reports are gzip-compressed (each logs every control tick: ~2.5 MB as JSON).
    paths = sorted(list(args.campaign.glob("*/sitl_report.json")) + list(args.campaign.glob("*/sitl_report.json.gz")))
    reports = []
    for path in paths:
        opener = gzip.open if path.suffix == ".gz" else open
        with opener(path, "rt") as handle:
            reports.append(json.load(handle))
    planned = json.loads(args.planned.read_text()) if args.planned else None
    summary = summarize_campaign(reports, planned)
    summary["evidence_paths"] = [str(path) for path in paths]
    args.out.parent.mkdir(parents=True, exist_ok=True)
    attempted = summary["runs"]
    passed = bool(attempted) and all(item["expectation_met"] for item in attempted)
    outcome = {
        "passed": passed,
        "detail": (
            f"{len(attempted)} SITL reports read. "
            f"{sum(1 for item in attempted if item['expectation_met'])} met the scenario expectation. "
            "Unstarted scenarios are listed in the summary and are not counted as verified. "
            "This is not a physical flight and not a population reliability claim."
        ),
        "run_count": len(attempted),
        "physical_flight_validated": False,
    }
    summary["campaign_outcome"] = outcome
    args.out.write_text(json.dumps(summary, indent=2) + "\n")
    results_path = args.campaign / "campaign_results.json"
    if passed:
        results_path.write_text(json.dumps(outcome, indent=2) + "\n")
    elif results_path.is_file():
        results_path.unlink()
    print(json.dumps({"summary": str(args.out), "passed": passed, "runs": len(attempted)}, indent=2))
    return 0 if reports else 1


if __name__ == "__main__":
    raise SystemExit(main())
