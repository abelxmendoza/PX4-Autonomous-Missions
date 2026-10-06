#!/usr/bin/env python3
"""Run fault-injection scenarios: python tools/run_fault_scenarios.py config/fault_scenarios/*.yaml --out artifacts/faults"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "px4_offboard"))
from px4_offboard.faults.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
