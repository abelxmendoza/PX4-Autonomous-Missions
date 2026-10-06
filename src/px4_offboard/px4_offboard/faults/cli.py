"""Command-line runner for fault scenarios (also reachable as tools/run_fault_scenarios.py)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .evidence import run_scenario, series_csv, to_markdown
from .schema import ScenarioError, load_scenario


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run declarative fault-injection scenarios.")
    parser.add_argument("scenarios", nargs="+", help="scenario YAML file(s)")
    parser.add_argument("--out", type=Path, help="write JSON, Markdown and per-fault CSV evidence here")
    args = parser.parse_args(argv)

    all_ok = True
    for path in args.scenarios:
        try:
            scenario = load_scenario(path)
        except (ScenarioError, OSError) as exc:
            print(f"{path}: invalid scenario: {exc}", file=sys.stderr)
            return 2
        evidence = run_scenario(scenario, collect_series=args.out is not None)
        markdown = to_markdown(evidence)
        print(markdown)
        all_ok &= evidence.ok
        if args.out:
            args.out.mkdir(parents=True, exist_ok=True)
            series_dir = args.out / f"{scenario.name}_series"
            series_dir.mkdir(exist_ok=True)
            data = evidence.to_dict()
            for record in data["records"]:
                (series_dir / f"{record['id'].split('/')[1]}.csv").write_text(series_csv(record))
                record.pop("_series")
                record["series_csv"] = f"{series_dir.name}/{record['id'].split('/')[1]}.csv"
            (args.out / f"{scenario.name}.json").write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
            (args.out / f"{scenario.name}.md").write_text(markdown)
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
