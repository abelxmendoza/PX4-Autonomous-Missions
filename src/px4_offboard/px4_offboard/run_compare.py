"""Compare flight runs (baseline vs candidate) without hiding run-to-run variance.

Single flights are noisy: in this repository two live flights with identical
settings ended at 9.3% and 13.2% drift. So every metric is reported with the
number of runs behind it and its min..max spread, and a difference is only
called a REGRESSION/IMPROVEMENT when it is larger than the metric's tolerance
and -- if either side has repeated runs -- outside the other side's spread.
With one run per side the verdict is still given but tagged ``single_run``.
"""

from __future__ import annotations

import argparse
import csv
import gzip
import json
import math
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

from .flight_replay import AIRBORNE_STATES, FlightTrace, load_flight_log
from .vv_harness import VO_MIN_PATH_M

# name -> (direction, absolute tolerance, relative tolerance, unit)
METRICS: dict[str, tuple[str, float, float, str]] = {
    "mission_completed": ("higher", 0.0, 0.0, "fraction"),
    "move_duration_s": ("lower", 1.0, 0.10, "s"),
    "final_drift_pct": ("lower", 2.0, 0.0, "%"),
    "peak_drift_pct": ("lower", 3.0, 0.0, "%"),
    "est_err_mean_m": ("lower", 0.5, 0.15, "m"),
    "est_err_max_m": ("lower", 1.0, 0.15, "m"),
    "vo_availability_pct": ("higher", 5.0, 0.0, "%"),
    "vo_outage_count": ("lower", 2.0, 0.0, ""),
    "vo_longest_outage_s": ("lower", 0.5, 0.0, "s"),
    "vo_mean_recovery_s": ("lower", 0.3, 0.0, "s"),
    "vel_tracking_rms_mps": ("lower", 0.1, 0.15, "m/s"),
    "max_tilt_deg": ("lower", 5.0, 0.0, "deg"),
    "min_clearance_m": ("higher", 0.25, 0.0, "m"),
    "log_rate_hz": ("higher", 0.0, 0.10, "Hz"),
    "cpu_pct_mean": ("lower", 5.0, 0.20, "%"),
}


@dataclass
class Run:
    trace: FlightTrace
    path: str
    cpu_pct: list[float]


def load_run(path: str | Path) -> Run:
    path = Path(path)
    raw = path.read_bytes()
    with tempfile.TemporaryDirectory() as d:
        src = Path(d) / "run.csv"
        src.write_bytes(gzip.decompress(raw) if path.suffix == ".gz" else raw)
        trace = load_flight_log(src)
        cpu: list[float] = []
        with src.open(newline="") as fh:
            reader = csv.DictReader(fh)
            if "cpu_pct" in (reader.fieldnames or []):
                for row in reader:
                    try:
                        cpu.append(float(row["cpu_pct"]))
                    except (TypeError, ValueError):
                        pass
    return Run(trace, str(path), cpu)


def compute_metrics(run: Run) -> dict[str, float | bool | None]:
    samples = run.trace.samples
    states = {s.state for s in samples}
    move = [s for s in samples if s.state == "MOVE"]
    airborne = [s for s in samples if s.state in AIRBORNE_STATES]
    m: dict[str, Any] = {k: None for k in METRICS}
    m["mission_completed"] = "LANDING" in states and "FAILSAFE" not in states
    if move:
        m["move_duration_s"] = move[-1].t_s - move[0].t_s
    if len(samples) > 1 and samples[-1].t_s > samples[0].t_s:
        m["log_rate_hz"] = (len(samples) - 1) / (samples[-1].t_s - samples[0].t_s)
    if airborne:
        m["max_tilt_deg"] = max(max(abs(s.roll_deg), abs(s.pitch_deg)) for s in airborne)
    clearances = [s.mapped_clearance_m for s in samples if s.mapped_clearance_m is not None]
    if clearances:
        m["min_clearance_m"] = min(clearances)

    # Definitions below deliberately mirror vv_harness.check_vo_drift /
    # check_vo_availability so the two tools can never disagree on one flight.
    if move and any(s.vo_n is not None for s in samples):
        healthy_moves = [s for s in move if s.vo_healthy]
        m["vo_availability_pct"] = 100.0 * len(healthy_moves) / len(move)
        healthy = [
            s for s in samples
            if s.vo_healthy and s.vo_drift_frac is not None and s.vo_path_m is not None
        ]
        errs = [s.vo_err_m for s in healthy if s.vo_err_m is not None]
        if errs:
            m["est_err_mean_m"] = sum(errs) / len(errs)
            m["est_err_max_m"] = max(errs)
        far = [s for s in healthy if s.vo_path_m >= VO_MIN_PATH_M]
        if far:
            m["final_drift_pct"] = 100.0 * far[-1].vo_drift_frac
            m["peak_drift_pct"] = 100.0 * max(s.vo_drift_frac for s in far)
        outages, recoveries, start = 0, [], None
        for s in move:
            if not s.vo_healthy and start is None:
                start = s.t_s
                outages += 1
            elif s.vo_healthy and start is not None:
                recoveries.append(s.t_s - start)
                start = None
        longest = max(recoveries, default=0.0)
        if start is not None:  # still unavailable at the end of MOVE
            longest = max(longest, move[-1].t_s - start)
        m["vo_outage_count"] = outages
        m["vo_longest_outage_s"] = longest
        m["vo_mean_recovery_s"] = sum(recoveries) / len(recoveries) if recoveries else None

    tracked = [s for s in move if s.vel_cmd_n is not None]
    if tracked:
        sq = [
            (s.vn - s.vel_cmd_n) ** 2 + (s.ve - (s.vel_cmd_e or 0.0)) ** 2 + (s.vd - (s.vel_cmd_d or 0.0)) ** 2
            for s in tracked
        ]
        m["vel_tracking_rms_mps"] = math.sqrt(sum(sq) / len(sq))
    if run.cpu_pct:
        m["cpu_pct_mean"] = sum(run.cpu_pct) / len(run.cpu_pct)
    return m


def _stats(values: list[float]) -> dict[str, float | int] | None:
    if not values:
        return None
    return {"n": len(values), "mean": sum(values) / len(values), "min": min(values), "max": max(values)}


def _verdict(name: str, base: dict, cand: dict) -> tuple[str, str, str]:
    direction, abs_tol, rel_tol, _ = METRICS[name]
    delta = cand["mean"] - base["mean"]
    material = abs(delta) > max(abs_tol, rel_tol * abs(base["mean"]))
    repeated = base["n"] >= 2 or cand["n"] >= 2
    confidence = "repeated_runs" if repeated else "single_run"
    if not material:
        return "UNCHANGED", confidence, "within tolerance"
    overlap = cand["min"] <= base["max"] and base["min"] <= cand["max"]
    if repeated and overlap:
        return (
            "INCONCLUSIVE",
            confidence,
            f"difference is inside the run-to-run spread (baseline {base['min']:.3g}..{base['max']:.3g}, "
            f"candidate {cand['min']:.3g}..{cand['max']:.3g})",
        )
    worse = delta > 0 if direction == "lower" else delta < 0
    note = "variance unknown with one run per side" if not repeated else "outside the run-to-run spread"
    return ("REGRESSION" if worse else "IMPROVEMENT"), confidence, note


def compare(baseline: Sequence[Run], candidate: Sequence[Run]) -> dict[str, Any]:
    base_metrics = [compute_metrics(r) for r in baseline]
    cand_metrics = [compute_metrics(r) for r in candidate]
    rows = []
    for name, (_, _, _, unit) in METRICS.items():
        b = _stats([float(m[name]) for m in base_metrics if m[name] is not None])
        c = _stats([float(m[name]) for m in cand_metrics if m[name] is not None])
        row: dict[str, Any] = {"metric": name, "unit": unit, "baseline": b, "candidate": c}
        if b is None or c is None:
            row.update(verdict="N/A", confidence="none", delta=None, note="not recorded in one or both runs")
        else:
            verdict, confidence, note = _verdict(name, b, c)
            row.update(verdict=verdict, confidence=confidence, delta=c["mean"] - b["mean"], note=note)
        rows.append(row)
    notes = []
    if len(baseline) == 1 and len(candidate) == 1:
        notes.append(
            "Single run per side: run-to-run variance is unknown, so every verdict is tagged single_run. "
            "In this repository two live flights with identical settings (K, L) differ by 3.9 percentage "
            "points in final drift and 7 points in VO availability; differences of that size are not "
            "evidence of a change. Add --baseline-extra/--candidate-extra runs to separate signal from noise."
        )
    count = lambda v: sum(1 for r in rows if r["verdict"] == v)  # noqa: E731
    return {
        "baseline": [r.path for r in baseline],
        "candidate": [r.path for r in candidate],
        "metrics": rows,
        "notes": notes,
        "summary": {
            "regressions": count("REGRESSION"),
            "improvements": count("IMPROVEMENT"),
            "unchanged": count("UNCHANGED"),
            "inconclusive": count("INCONCLUSIVE"),
            "not_available": count("N/A"),
        },
    }


def _fmt(stats: dict | None) -> str:
    if stats is None:
        return "-"
    if stats["n"] == 1:
        return f"{stats['mean']:.3g} (n=1)"
    return f"{stats['mean']:.3g} [{stats['min']:.3g}..{stats['max']:.3g}] (n={stats['n']})"


def format_text(result: dict[str, Any]) -> str:
    header = ("metric", "baseline", "candidate", "delta", "verdict", "note")
    table = [header]
    for r in result["metrics"]:
        delta = "-" if r["delta"] is None else f"{r['delta']:+.3g}"
        tag = r["verdict"] + ("" if r["confidence"] in ("none", "repeated_runs") else " (single run)")
        table.append((r["metric"], _fmt(r["baseline"]), _fmt(r["candidate"]), delta, tag, r["note"]))
    widths = [max(len(row[i]) for row in table) for i in range(len(header) - 1)]
    lines = ["  ".join(cell.ljust(w) for cell, w in zip(row, widths)) + "  " + row[-1] for row in table]
    s = result["summary"]
    lines.append("")
    lines.append(
        f"{s['regressions']} regression(s), {s['improvements']} improvement(s), {s['unchanged']} unchanged, "
        f"{s['inconclusive']} inconclusive, {s['not_available']} n/a"
    )
    lines += [""] + [f"NOTE: {n}" for n in result["notes"]]
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Compare a candidate flight log with a baseline.")
    parser.add_argument("baseline", type=Path, help="baseline flight CSV (.csv or .csv.gz)")
    parser.add_argument("candidate", type=Path, help="candidate flight CSV")
    parser.add_argument("--baseline-extra", type=Path, nargs="*", default=[], help="more baseline runs (variance)")
    parser.add_argument("--candidate-extra", type=Path, nargs="*", default=[], help="more candidate runs (variance)")
    parser.add_argument("--json", type=Path, help="also write the result as JSON")
    parser.add_argument("--fail-on-regression", action="store_true", help="exit 1 if any REGRESSION is reported")
    args = parser.parse_args(argv)

    result = compare(
        [load_run(p) for p in [args.baseline, *args.baseline_extra]],
        [load_run(p) for p in [args.candidate, *args.candidate_extra]],
    )
    sys.stdout.write(format_text(result))
    if args.json:
        args.json.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    return 1 if args.fail_on_regression and result["summary"]["regressions"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
