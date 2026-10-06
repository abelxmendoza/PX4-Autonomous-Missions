#!/usr/bin/env python3
"""python tools/compare_runs.py baseline.csv candidate.csv [--baseline-extra ...] [--candidate-extra ...]"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "px4_offboard"))
from px4_offboard.run_compare import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
