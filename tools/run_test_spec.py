#!/usr/bin/env python3
"""Run a declarative test spec (see docs/test_specs.md).

    python tools/run_test_spec.py --spec config/test_specs/basic_mission.yaml
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "px4_offboard"))
from px4_offboard.testspec.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
