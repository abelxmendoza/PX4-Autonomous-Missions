#!/usr/bin/env python3
"""Recheck curated recordings without ROS/PX4; fail on changed inputs or results."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/px4_offboard"))
from px4_offboard.flight_replay import load_flight_log
from px4_offboard.swarm_verify import verify
from px4_offboard.vv_harness import run_vv
from export_swarm_replay import export


def evaluate(case, write_replays=False):
    path = ROOT / case["path"]
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != case["sha256"]:
        raise ValueError("recording SHA-256 differs from manifest")
    with tempfile.TemporaryDirectory() as directory:
        source = Path(directory) / "recording"
        source.write_bytes(gzip.decompress(raw) if path.suffix == ".gz" else raw)
        if case["kind"] == "flight":
            report = run_vv(load_flight_log(source)).to_dict()
            report["source"] = report["summary"]["source"] = case["path"]
            checks = {r["requirement_id"]: r for r in report["results"]}
            for req in case.get("required_checks", []):
                if req not in checks or checks[req]["skipped"] or not checks[req]["passed"]:
                    raise ValueError(f"required evidence missing or failing: {req}")
        elif case["kind"] == "swarm":
            report = verify(source)
            if case.get("replay"):
                replay = export(source)
                replay["source"] = case["id"]
                output = ROOT / case["replay"]
                if write_replays:
                    output.write_text(json.dumps(replay, separators=(",", ":"), allow_nan=False) + "\n")
                elif json.loads(output.read_text()) != replay:
                    raise ValueError("browser replay differs from raw curated evidence")
        else:
            raise ValueError("unknown recording kind")
    if report["passed"] is not case["expected_pass"]:
        raise ValueError(f"unexpected verdict: {report}")
    if not case["expected_pass"] and report.get("errors") != case["expected_errors"]:
        raise ValueError(f"unexpected failure reason: {report}")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write-reports", action="store_true",
                        help="explicitly refresh report snapshots after reviewing a change")
    parser.add_argument("--write-replays", action="store_true",
                        help="explicitly rebuild browser exports from curated raw evidence")
    args = parser.parse_args()
    manifest = json.loads((ROOT / "evidence/manifest.json").read_text())
    errors = []
    lines = ["| Scenario | Result |", "| --- | --- |"]
    for case in manifest["cases"]:
        try:
            report = evaluate(case, args.write_replays)
            snapshot = ROOT / case["report"]
            if args.write_reports:
                snapshot.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
            elif json.loads(snapshot.read_text()) != report:
                raise ValueError("report changed; inspect before refreshing the snapshot")
            status = "PASS" if case["expected_pass"] else "EXPECTED FAIL (regression caught)"
        except (OSError, ValueError, KeyError, TypeError) as exc:
            status = f"ERROR: {exc}"
            errors.append(case["id"])
        lines.append(f"| {case['id']} | {status} |")
    print("\n".join(lines))
    print(f"\n{len(manifest['cases']) - len(errors)}/{len(manifest['cases'])} evidence expectations matched.")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
