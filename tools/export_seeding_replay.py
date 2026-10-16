#!/usr/bin/env python3
"""Publish recorded seeding sorties to the browser replay.

    python tools/export_seeding_replay.py RUN_DIR [RUN_DIR ...] --stamp 20261016T...Z

Each RUN_DIR is a LOG_DIR from scripts/run_integrated_seeding_sitl.sh holding trace.json
(written with --trace). The traces are copied unchanged to evidence/seeding_replay/<stamp>/
and to web/replay/data/seeding/, with an index.json for the scenario picker. A test keeps the
two copies identical.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# Scenario id -> (picker label, one-line description). Order is the picker order.
SCENARIOS = {
    "nominal": ("Nominal sortie", "Every gate allows: climb, release seed, return, land on the pad."),
    "weather_denial": ("Weather denial", "Wind above the limit: no release, return and land."),
    "airspace_denial": ("Airspace denial", "Release point inside prohibited airspace: no release, return and land."),
    "traffic_conflict": ("Traffic conflict", "Another aircraft at the release point: no release, return and land."),
    "clearance_expired": ("Clearance expired", "The simulated clearance has lapsed: no release, return and land."),
    "altitude_denial": ("Altitude denial", "Outside the allowed altitude band: no release, return and land."),
    "marker_loss": ("Marker lost on descent", "The pad marker is withheld mid-descent: the landing fails safe and holds."),
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--stamp", required=True)
    args = parser.parse_args()

    traces = {}
    for run in args.runs:
        doc = json.loads((run / "trace.json").read_text())
        traces[doc["scenario_id"]] = (run / "trace.json").read_text()
    missing = [s for s in SCENARIOS if s not in traces]
    if missing:
        print(f"missing scenarios: {missing}", file=sys.stderr)
        return 1

    evidence = ROOT / "evidence" / "seeding_replay" / args.stamp
    web = ROOT / "web" / "replay" / "data" / "seeding"
    for out in (evidence, web):
        out.mkdir(parents=True, exist_ok=True)
    entries = []
    for scenario, (label, blurb) in SCENARIOS.items():
        text = traces[scenario]
        doc = json.loads(text)
        name = f"{scenario}.json"
        (evidence / name).write_text(text)
        (web / name).write_text(text)
        outcomes = doc.get("outcomes", {})
        entries.append({
            "id": scenario, "label": label, "blurb": blurb, "file": name,
            "expected_result": doc["expected_result"], "mission_result": doc["mission_result"],
            "result_as_designed": doc["result_as_designed"],
            "release_result": outcomes.get("release_result"), "release_reason": outcomes.get("release_reason_code", ""),
            "touchdown_error_m": doc.get("touchdown_horizontal_error_m"), "marker_id": doc.get("marker_id"),
            "duration_s": doc.get("mission_duration_s"),
        })
    index = {"schema": 1, "campaign": args.stamp, "source": f"evidence/seeding_replay/{args.stamp}/",
             "px4_version": json.loads(traces["nominal"]).get("px4_version"), "scenarios": entries}
    text = json.dumps(index, indent=2) + "\n"
    (evidence / "index.json").write_text(text)
    (web / "index.json").write_text(text)
    for e in entries:
        print(f"{e['id']:18s} {e['mission_result']:18s} as designed: {e['result_as_designed']}  "
              f"touchdown {e['touchdown_error_m']} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
