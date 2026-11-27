"""CLI.

    # draft one spec (prints the YAML or the refusal; exit 0 only when VALID)
    python -m px4_offboard.nlspec draft "take off to 5 m, fly a 10 m square, land" [--save spec.yaml]

    # score a model on the prompt set; --run-mock also executes VALID drafts on the mock adapter
    python -m px4_offboard.nlspec eval [--set config/nl_prompts/eval_set.yaml] [--run-mock]

Endpoint: --base-url/--model or NLSPEC_BASE_URL / NLSPEC_MODEL / NLSPEC_API_KEY.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from px4_offboard.validation.registry import load_registry

from .client import OpenAICompatClient
from .evaluate import DEFAULT_OUT, DEFAULT_SET, load_cases, run_eval
from .generate import VALID, generate


def _mock_runner(spec_path: str, out_root: Path) -> str:
    from px4_offboard.testspec.runner import SpecRun

    return SpecRun(spec_path, out_root).execute()["verdict"]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base-url", default="")
    ap.add_argument("--model", default="")
    ap.add_argument("--label", default="", help="model label for the report (default: --model)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("draft")
    d.add_argument("request")
    d.add_argument("--save", type=Path)
    e = sub.add_parser("eval")
    e.add_argument("--set", type=Path, default=DEFAULT_SET)
    e.add_argument("--out", type=Path, default=DEFAULT_OUT)
    e.add_argument("--run-mock", action="store_true")
    args = ap.parse_args(argv)

    client = OpenAICompatClient(base_url=args.base_url, model=args.model)
    known = {r.id for r in load_registry().requirements}

    if args.cmd == "draft":
        gen = generate(args.request, client, known)
        if gen.status == VALID:
            print(gen.yaml_text)
            if args.save:
                args.save.write_text(gen.yaml_text)
        tps = gen.chat.completion_tps if gen.chat else None
        print(f"[{gen.status}] {gen.refusal or gen.error or gen.spec.name}"
              + (f"  ({gen.chat.latency_s:.1f}s, {tps:.1f} tok/s)" if tps else ""), file=sys.stderr)
        return 0 if gen.status == VALID else 1

    report = run_eval(client, load_cases(args.set), known, args.label or client.model, args.out,
                      _mock_runner if args.run_mock else None)
    print(json.dumps(report["summary"], indent=2))
    for r in report["results"]:
        detail = r.get("failure") or r.get("caught_by") or ""
        print(f"  {r['result']:4s} {r['id']:<28s} {r['status']:<9s} {detail}")
    print(f"report: {Path(args.out) / report['run_id'] / 'report.json'}")
    return 0 if report["summary"]["unsafe_accepted"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
