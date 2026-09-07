#!/usr/bin/env python3
"""Generate real V&V reports (vv_harness.py) as JSON for each mission CSV
committed under web/replay/data/, so the web viewer can display the exact
requirement-by-requirement result this module computes — not a
reimplementation of the checks in JS. Re-run this after adding/replacing a
mission recording there.

Usage: PYTHONPATH=src/px4_offboard python3 scripts/gen_vv_reports.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src" / "px4_offboard"))

from px4_offboard.flight_replay import load_flight_log
from px4_offboard.vv_harness import DEFAULT_FENCE, run_vv

DATA_DIR = Path(__file__).resolve().parent.parent / "web" / "replay" / "data"


def main() -> int:
    csvs = sorted(DATA_DIR.glob("*.csv"))
    if not csvs:
        print(f"no CSVs found under {DATA_DIR}", file=sys.stderr)
        return 1
    for csv_path in csvs:
        trace = load_flight_log(csv_path)
        report = run_vv(trace, fence=DEFAULT_FENCE)
        out_path = csv_path.with_suffix(".vv.json")
        out_path.write_text(json.dumps(report.to_dict(), indent=2) + "\n")
        verdict = "PASS" if report.passed else "FAIL"
        print(f"{csv_path.name} -> {out_path.name}  [{verdict}]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
