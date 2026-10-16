#!/usr/bin/env python3
"""Run the golden virtual-bench campaign and write its evidence.

No network, no PX4, no Gazebo: this runs VirtualTestRig in-process
(deterministic given its seed) and writes:

  evidence/bench/golden_report.json    -- requirement pass/fail + detail
  evidence/bench/golden_campaign.jsonl -- one JSON object per sample
  web/replay/data/bench_campaign.json  -- report + samples, for the browser
                                           instrument-panel replay

Not a manifest case (see evidence/README.md): unlike the flight recordings
under evidence/manifest.json, this campaign is reproduced by re-running the
script, not pinned by SHA-256, since nothing here depends on a one-off PX4/
Gazebo session -- the same seed always produces the same samples.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/px4_offboard"))

from px4_offboard.bench.campaign import golden_spec, report_dict, run_campaign, samples_to_jsonl_rows  # noqa: E402


def main() -> int:
    spec = golden_spec()
    samples, results = run_campaign(spec)
    report = report_dict(spec, samples, results)
    rows = samples_to_jsonl_rows(samples)

    bench_dir = ROOT / "evidence" / "bench"
    bench_dir.mkdir(parents=True, exist_ok=True)
    (bench_dir / "golden_report.json").write_text(json.dumps(report, indent=2) + "\n")
    with (bench_dir / "golden_campaign.jsonl").open("w") as fh:
        for row in rows:
            fh.write(json.dumps(row) + "\n")

    web_data_dir = ROOT / "web" / "replay" / "data"
    web_data_dir.mkdir(parents=True, exist_ok=True)
    web_payload = {"report": report, "samples": rows}
    (web_data_dir / "bench_campaign.json").write_text(json.dumps(web_payload) + "\n")

    print(f"{report['campaign']}: {'PASS' if report['passed'] else 'FAIL'} "
          f"({report['sample_count']} samples)")
    for r in report["requirements"]:
        print(f"  [{'PASS' if r['passed'] else 'FAIL'}] {r['id']} -- {r['detail']}")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
