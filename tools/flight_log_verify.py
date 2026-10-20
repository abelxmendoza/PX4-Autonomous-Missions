#!/usr/bin/env python3
"""Evidence-first PX4 ULog inspection. Requires: pip install pyulog.

This tool does NOT certify flight safety. Missing fields yield SKIP, not PASS.
"""
import argparse
import json
import math
from pathlib import Path


def inspect_ulog(path, max_gap_s=2.0):
    from pyulog import ULog

    log = ULog(str(path), message_name_filter_list=["vehicle_local_position"])
    datasets = [d for d in log.data_list if d.name == "vehicle_local_position" and d.multi_id == 0]
    checks = []
    if not datasets:
        checks.append({"id": "LOCAL_POSITION_PRESENT", "result": "SKIP",
                       "reason": "vehicle_local_position instance 0 not logged"})
        return {"source": str(path), "checks": checks, "summary": {"pass": 0, "fail": 0, "skip": 1}}
    data = datasets[0].data
    fields = ("timestamp", "x", "y", "z")
    missing = [f for f in fields if f not in data]
    if missing:
        checks.append({"id": "LOCAL_POSITION_VALID", "result": "SKIP",
                       "reason": "Missing fields: " + ", ".join(missing)})
    else:
        samples = len(data["timestamp"])
        if not samples:
            checks.append({"id": "LOCAL_POSITION_VALID", "result": "SKIP",
                           "reason": "No samples"})
        else:
            bad = [i for i in range(samples) if not all(
                math.isfinite(float(data[f][i])) for f in ("x", "y", "z"))]
            checks.append({"id": "LOCAL_POSITION_FINITE", "result": "FAIL" if bad else "PASS",
                           "sample_count": samples, "invalid_samples": len(bad),
                           "first_invalid_timestamp_us": int(data["timestamp"][bad[0]]) if bad else None})
            timestamps = [int(t) for t in data["timestamp"]]
            gaps = [(i, (timestamps[i] - timestamps[i - 1]) / 1e6)
                    for i in range(1, samples)]
            violations = [(i, gap) for i, gap in gaps if gap <= 0 or gap > max_gap_s]
            checks.append({"id": "LOCAL_POSITION_SAMPLE_GAP",
                           "result": "FAIL" if violations else ("SKIP" if samples < 2 else "PASS"),
                           "max_allowed_gap_s": max_gap_s,
                           "max_observed_gap_s": max((gap for _, gap in gaps), default=None),
                           "violation_count": len(violations),
                           "first_violation_timestamp_us": timestamps[violations[0][0]] if violations else None,
                           "reason": "Requires at least two samples" if samples < 2 else None})
    counts = {k.lower(): sum(c["result"] == k for c in checks) for k in ("PASS", "FAIL", "SKIP")}
    return {"source": str(path), "checks": checks, "summary": counts,
            "limitations": ["Single ULog and vehicle_local_position instance 0 only",
                            "Sampling checks are not position-accuracy or safety certification",
                            "Log gaps can reflect logging behavior, not necessarily control-link loss"]}


def main():
    parser = argparse.ArgumentParser(description="Inspect PX4 ULog with explicit PASS/FAIL/SKIP evidence")
    parser.add_argument("ulog", type=Path)
    parser.add_argument("--max-gap-s", type=float, default=2.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.max_gap_s <= 0 or not math.isfinite(args.max_gap_s):
        parser.error("--max-gap-s must be finite and positive")
    if not args.ulog.is_file() or args.ulog.suffix.lower() != ".ulg":
        parser.error("Provide an existing .ulg file")
    try:
        report = inspect_ulog(args.ulog, args.max_gap_s)
    except (ImportError, ValueError, TypeError, OSError) as exc:
        parser.exit(2, f"Could not inspect ULog: {exc}\nInstall pyulog and check the input file.\n")
    rendered = json.dumps(report, indent=2)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    else:
        print(rendered)
    return 1 if report["summary"]["fail"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
