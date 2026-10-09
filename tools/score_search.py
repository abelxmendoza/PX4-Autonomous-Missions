#!/usr/bin/env python3
"""Score a search report against the ground truth (the drones never see the truth file).

    python tools/score_search.py /tmp/search_demo/search_report.json [--max-error 2.0]
"""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "px4_offboard"))
from px4_offboard.search_score import score  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("report", type=Path)
    parser.add_argument("--truth", type=Path, default=ROOT / "worlds" / "search_field_targets.json")
    parser.add_argument("--max-error", type=float, default=2.0, help="pass threshold, metres")
    args = parser.parse_args()
    report, truth = json.loads(args.report.read_text()), json.loads(args.truth.read_text())
    s = score(report, truth)
    truth_by_id = {t["id"]: t for t in truth["targets"]}
    rep_by_id = {t["id"]: t for t in report["targets"]}
    print(f"{'id':>4} {'reported N,E':>18} {'truth N,E':>18} {'error m':>8}")
    for mid, err in s.found.items():
        r, t = rep_by_id[mid], truth_by_id[mid]
        print(f"{mid:>4} {r['north']:>8.2f},{r['east']:>8.2f} {t['north']:>8.2f},{t['east']:>8.2f} {err:>8.2f}")
    print(f"found {len(s.found)}/{len(truth_by_id)} | missed {s.missed} | false ids {s.false_ids}")
    if s.max_error_m is not None:
        print(f"position error: mean {s.mean_error_m:.2f} m, max {s.max_error_m:.2f} m (pass if <= {args.max_error} m)")
    verdict = s.passed(args.max_error)
    print("PASS" if verdict else "FAIL")
    return 0 if verdict else 1


if __name__ == "__main__":
    raise SystemExit(main())
