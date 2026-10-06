#!/usr/bin/env python3
"""Evaluate requirements/registry.yaml and write JSON + Markdown validation reports.

    python tools/validation_report.py                # runs the registry's tests itself
    python tools/validation_report.py --junit a.xml  # reuse a CI run's JUnit output
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "px4_offboard"))
from px4_offboard.validation.cli import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
