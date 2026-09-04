"""CLI: replay a mission CSV and print a V&V PASS/FAIL report.

Usage:
  ros2 run px4_offboard vv_replay -- flight_log_mission_....csv
  python3 -m px4_offboard.vv_replay path/to/log.csv
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from px4_offboard.flight_replay import load_flight_log
from px4_offboard.mission_logic import Fence
from px4_offboard.vv_harness import DEFAULT_FENCE, run_vv


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Offline V&V: replay offboard_mission CSV and check requirements."
    )
    parser.add_argument("log", type=Path, help="Path to flight_log_mission_*.csv")
    parser.add_argument(
        "--fence-north",
        nargs=2,
        type=float,
        metavar=("MIN", "MAX"),
        default=None,
        help="Override fence north bounds",
    )
    parser.add_argument(
        "--fence-east",
        nargs=2,
        type=float,
        metavar=("MIN", "MAX"),
        default=None,
        help="Override fence east bounds",
    )
    parser.add_argument(
        "--fence-alt-max",
        type=float,
        default=None,
        help="Override max altitude (m)",
    )
    parser.add_argument(
        "--geocage-margin",
        type=float,
        default=1.0,
        help="Geo-cage margin used for REQ-GEOCAGE-01 (default 1.0)",
    )
    parser.add_argument(
        "--geofence-response",
        type=float,
        default=1.5,
        help="Seconds allowed after breach before FAILSAFE (default 1.5)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not args.log.is_file():
        print(f"error: log not found: {args.log}", file=sys.stderr)
        return 2

    fence = DEFAULT_FENCE
    if args.fence_north or args.fence_east or args.fence_alt_max is not None:
        nmin, nmax = args.fence_north or (fence.north_min, fence.north_max)
        emin, emax = args.fence_east or (fence.east_min, fence.east_max)
        alt = (
            fence.altitude_max
            if args.fence_alt_max is None
            else args.fence_alt_max
        )
        fence = Fence(nmin, nmax, emin, emax, alt)

    trace = load_flight_log(args.log)
    report = run_vv(
        trace,
        fence=fence,
        geocage_margin=args.geocage_margin,
        geofence_response_s=args.geofence_response,
    )
    print(report.format_text())
    return 0 if report.passed else 1


if __name__ == "__main__":
    sys.exit(main())
