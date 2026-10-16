"""Run a declarative test spec:

    python -m px4_offboard.testspec --spec config/test_specs/basic_mission.yaml [--out DIR]
    python -m px4_offboard.testspec --spec SPEC --validate-only

Exit status: 0 PASSED, 1 FAILED, 2 ERROR (including an invalid spec), 3 ABORTED.
Ctrl-C cancels the run cleanly (ABORTED) and still writes the report.
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
from pathlib import Path

from px4_offboard.validation.registry import load_registry

from .runner import DEFAULT_OUT, SpecRun
from .schema import SpecError, load_spec

EXIT = {"PASSED": 0, "FAILED": 1, "ERROR": 2, "ABORTED": 3}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--spec", type=Path, required=True, help="test spec YAML")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT, help=f"run directory root (default {DEFAULT_OUT})")
    parser.add_argument("--validate-only", action="store_true", help="validate the spec and exit; run nothing")
    parser.add_argument("--log-stderr", action="store_true", help="echo the structured log to stderr")
    args = parser.parse_args(argv)

    if args.validate_only:
        try:
            spec = load_spec(args.spec, {r.id for r in load_registry().requirements})
        except (OSError, SpecError) as exc:
            print(f"INVALID {args.spec}: {exc}", file=sys.stderr)
            return EXIT["ERROR"]
        print(f"VALID {args.spec}: {spec.name} ({spec.vehicle.adapter} adapter, {len(spec.mission)} steps, "
              f"{len(spec.assertions)} assertions, requirements {list(spec.requirement_ids)})")
        return 0

    cancel = threading.Event()
    previous = signal.signal(signal.SIGINT, lambda *_: cancel.set())
    try:
        run = SpecRun(args.spec, args.out, cancel=cancel,
                      echo=(lambda line: print(line, file=sys.stderr)) if args.log_stderr else None)
        report = run.execute()
    finally:
        signal.signal(signal.SIGINT, previous)

    env = report["environment"] or {}
    s = report["summary"]
    label = env.get("kind", "no-vehicle").upper()
    print(f"[{label}] {report['run_id']}: {report['verdict']} -- {report['verdict_reason']}")
    print(f"  steps {s['steps_completed']}/{s['steps_total']}, assertions {s['assertions_passed']}/"
          f"{s['assertions_total']} passed; report {run.run_dir / run.report_file}")
    for r in report["requirements"]:
        print(f"  {r['id']}: {r['result']} ({r['evidence_scope']} evidence; registry verdict unchanged)")
    if env.get("label"):
        print(f"  {env['label']}")
    return EXIT[report["verdict"]]


if __name__ == "__main__":
    raise SystemExit(main())
