#!/usr/bin/env python3
"""Verify a Jetson/RealSense bench replay report without running PX4."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SRC = ROOT / "src" / "px4_offboard"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))
from px4_offboard.bench.perception_report import PerceptionReportError, verify_perception_report_file  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report")
    parser.add_argument("--output")
    parser.add_argument("--allow-skip", action="store_true")
    args = parser.parse_args()
    try:
        result = verify_perception_report_file(args.report, allow_skip=args.allow_skip)
    except PerceptionReportError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    rendered = json.dumps(result.to_dict(), indent=2, sort_keys=True)
    if args.output:
        Path(args.output).write_text(rendered + "\n", encoding="utf-8")
    else:
        print(rendered)
    return 0 if result.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
