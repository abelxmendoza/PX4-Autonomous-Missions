"""Generate the validation report: tools/validation_report.py [--junit FILE ...] [--out DIR]"""

from __future__ import annotations

import argparse
import json
import platform
import subprocess
import sys
from pathlib import Path

from .evaluate import ROOT, EvalContext, evaluate_registry, parse_junit, run_pytest_for
from .registry import load_registry
from .report import build_report, to_markdown


def _git_commit() -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, help="registry YAML (default requirements/registry.yaml)")
    parser.add_argument("--out", type=Path, default=ROOT / "artifacts" / "validation")
    parser.add_argument("--junit", type=Path, nargs="*",
                        help="existing JUnit XML file(s) to take test outcomes from; "
                             "if omitted, the registry's tests are run now")
    args = parser.parse_args(argv)

    registry = load_registry(args.registry)
    args.out.mkdir(parents=True, exist_ok=True)
    if args.junit:
        junit_paths, ran_pytest = list(args.junit), False
    else:
        junit = args.out / "junit_registry_tests.xml"
        run_pytest_for([t for r in registry.requirements for t in r.tests], junit)
        junit_paths, ran_pytest = [junit], True
    outcomes = parse_junit(junit_paths)

    ctx = EvalContext()
    results = evaluate_registry(registry, outcomes, ROOT, ctx)
    report = build_report(
        results,
        meta={
            "commit": _git_commit(),
            "python": platform.python_version(),
            "test_outcomes_from": [str(p) for p in junit_paths],
            "pytest_run_by_this_command": ran_pytest,
        },
    )
    (args.out / "validation_report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    (args.out / "validation_report.md").write_text(to_markdown(report))
    for rel, data in ctx.artifacts.items():
        target = args.out / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")

    s = report["summary"]
    print(f"{s['pass']} PASS, {s['fail']} FAIL ({s['known_open']} known open), "
          f"{s['partial']} PARTIAL, {s['not_run']} NOT_RUN -> {args.out}")
    for problem in s["gate_problems"]:
        print(f"GATE: {problem}", file=sys.stderr)
    return 0 if s["gate_ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
