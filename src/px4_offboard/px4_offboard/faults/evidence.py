"""Run a scenario and turn each fault into an evidence record:
fault injected -> expected behaviour -> observed behaviour -> verdict ->
recovery time. Each fault runs alone (same seed) beside a fault-free baseline,
so every number is attributable to exactly one cause."""

from __future__ import annotations

import csv
import io
from dataclasses import dataclass, field
from typing import Any

from .fusion_rig import FusionRig, RigConfig, RigTrace
from .link_rig import LinkRig, LinkRigConfig, LinkTrace
from .schema import FaultSpec, Scenario

RECOVERY_HOLD_S = 1.0
RECOVERY_VEL_TOL_MPS = 0.35  # baseline peaks at ~0.17 m/s (see test_fusion_rig)
LINK_FRESH_AGE_S = 0.3
INSTANT_FAULT_WINDOW_S = 3.0


@dataclass
class ScenarioEvidence:
    scenario: str
    seed: int
    duration_s: float
    records: list[dict[str, Any]] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        """Gate: no failing fault, other than ones declared as known gaps."""
        return all(r["status"] in ("PASS", "KNOWN_GAP", "PASS_GAP_OBSOLETE") for r in self.records)

    def to_dict(self) -> dict[str, Any]:
        return {
            "scenario": self.scenario,
            "seed": self.seed,
            "duration_s": self.duration_s,
            "ok": self.ok,
            "records": self.records,
        }


def _first_sustained(samples, start_t: float, predicate, hold_s: float) -> float | None:
    run_start = None
    for s in samples:
        if s.t < start_t:
            continue
        if predicate(s):
            run_start = s.t if run_start is None else run_start
            if s.t - run_start >= hold_s - 1e-9:
                return run_start
        else:
            run_start = None
    return None


def _checks(expect: dict[str, Any], observed: dict[str, Any]) -> list[dict[str, Any]]:
    checks = []

    def add(name, threshold, value, passed):
        checks.append({"name": name, "expected": threshold, "observed": value, "passed": bool(passed)})

    if expect.get("must_detect"):
        add("must_detect", True, observed["detected"], observed["detected"])
    if "detect_within_s" in expect:
        t = observed["detection_time_s"]
        add("detect_within_s", expect["detect_within_s"], t, t is not None and t <= expect["detect_within_s"])
    if "recover_within_s" in expect:
        t = observed["recovery_time_s"]
        add("recover_within_s", expect["recover_within_s"], t, t is not None and t <= expect["recover_within_s"])
    if "max_err_growth_m" in expect:
        g = observed["err_growth_m"]
        add("max_err_growth_m", expect["max_err_growth_m"], g, g <= expect["max_err_growth_m"])
    if expect.get("must_stay_alive"):
        add("must_stay_alive", True, not observed["went_stale"], not observed["went_stale"])
    if "min_rx_rate_ratio" in expect:
        r = observed["rx_rate_ratio"]
        add("min_rx_rate_ratio", expect["min_rx_rate_ratio"], r, r >= expect["min_rx_rate_ratio"])
    return checks


def _fusion_record(fault: FaultSpec, base: RigTrace, trace: RigTrace, injected: dict) -> tuple[dict, list]:
    window_end = fault.end_s if fault.duration_s > 0 else fault.start_s + INSTANT_FAULT_WINDOW_S
    detected_t = next(
        (s.t for s in trace.samples if fault.start_s <= s.t <= window_end + 0.7 and not s.healthy), None
    )
    recovery_from = fault.end_s if fault.duration_s > 0 else fault.start_s
    recovered_at = _first_sustained(
        trace.samples,
        recovery_from,
        lambda s: s.healthy and s.vel_err_mps < RECOVERY_VEL_TOL_MPS,
        RECOVERY_HOLD_S,
    )
    err_fault = max(s.err_horiz_m for s in trace.samples if s.t >= fault.start_s)
    err_base = max(s.err_horiz_m for s in base.samples if s.t >= fault.start_s)
    observed = {
        "detected": detected_t is not None,
        "detection_time_s": None if detected_t is None else round(detected_t - fault.start_s, 3),
        "recovery_time_s": None if recovered_at is None else round(max(0.0, recovered_at - recovery_from), 3),
        "max_err_horiz_m": round(err_fault, 3),
        "baseline_max_err_horiz_m": round(err_base, 3),
        "err_growth_m": round(err_fault - err_base, 3),
        "final_err_horiz_m": round(trace.samples[-1].err_horiz_m, 3),
        "vo_accepted": trace.vo_accepted,
        "vo_rejected": trace.vo_rejected,
        "baseline_vo_accepted": base.vo_accepted,
    }
    series = [
        {"t": s.t, "err_horiz_m": round(s.err_horiz_m, 4), "vel_err_mps": round(s.vel_err_mps, 4),
         "healthy": int(s.healthy), "baseline_err_horiz_m": round(b.err_horiz_m, 4)}
        for s, b in zip(trace.samples, base.samples)
    ]
    return observed, series


def _link_record(fault: FaultSpec, base: LinkTrace, trace: LinkTrace) -> tuple[dict, list]:
    warm = 4.0  # ignore the connection warm-up
    during = [s for s in trace.samples if fault.start_s <= s.t < fault.end_s]
    went_stale = any(not s.alive for s in during if s.t > warm)
    detected_t = next(
        (s.t for s in trace.samples if fault.start_s <= s.t <= fault.end_s + 0.5 and (not s.connected or not s.alive)),
        None,
    )
    recovered_at = _first_sustained(
        trace.samples,
        fault.end_s,
        lambda s: s.connected and s.alive and s.last_rx_age_s is not None and s.last_rx_age_s < LINK_FRESH_AGE_S,
        0.5,
    )
    nominal = base.frames_between(fault.start_s, fault.end_s)
    observed = {
        "detected": detected_t is not None,
        "detection_time_s": None if detected_t is None else round(detected_t - fault.start_s, 3),
        "recovery_time_s": None if recovered_at is None else round(max(0.0, recovered_at - fault.end_s), 3),
        "went_stale": went_stale,
        "frames_in_window": trace.frames_between(fault.start_s, fault.end_s),
        "baseline_frames_in_window": nominal,
        "rx_rate_ratio": round(trace.frames_between(fault.start_s, fault.end_s) / nominal, 3) if nominal else 1.0,
        "err_growth_m": 0.0,
    }
    series = [
        {"t": s.t, "connected": int(s.connected), "alive": int(s.alive),
         "last_rx_age_s": "" if s.last_rx_age_s is None else round(s.last_rx_age_s, 3)}
        for s in trace.samples
    ]
    return observed, series


def run_scenario(scenario: Scenario, collect_series: bool = False) -> ScenarioEvidence:
    evidence = ScenarioEvidence(scenario.name, scenario.seed, scenario.duration_s)
    rig_cfg = RigConfig(duration_s=scenario.duration_s, seed=scenario.seed)
    link_cfg = LinkRigConfig(duration_s=scenario.duration_s, seed=scenario.seed)
    background = list(scenario.background)
    base_fusion = FusionRig(rig_cfg).run(background)
    base_link = LinkRig(link_cfg).run(background)

    for index, fault in enumerate(scenario.faults):
        domain = fault.info.domain
        if domain == "sensor":
            rig = FusionRig(rig_cfg)
            trace = rig.run(background + [fault])
            injected = dict(rig.last_counters)
            observed, series = _fusion_record(fault, base_fusion, trace, injected)
        else:
            trace = LinkRig(link_cfg).run(background + [fault])
            injected = {"dropped_frames": trace.dropped_frames, "reconnects": trace.reconnects,
                        "last_reconnect_s": trace.last_reconnect_s}
            observed, series = _link_record(fault, base_link, trace)
        checks = _checks(fault.expect, observed)
        passed = all(c["passed"] for c in checks)
        if fault.known_gap and passed:
            status = "PASS_GAP_OBSOLETE"
        elif fault.known_gap:
            status = "KNOWN_GAP"
        else:
            status = "PASS" if passed else "FAIL"
        record = {
            "id": f"{scenario.name}/{index + 1}-{fault.type}",
            "domain": domain,
            "fault": {"type": fault.type, "start_s": fault.start_s, "duration_s": fault.duration_s,
                      "params": fault.params},
            "injected": injected,
            "expected": {
                "expected_text": fault.info.expected_text,
                "criteria": fault.expect,
                "overrides": fault.expect_overrides,
            },
            "observed": observed,
            "checks": checks,
            "recovery_time_s": observed["recovery_time_s"],
            "passed": passed,
            "status": status,
            "known_gap": fault.known_gap,
            "background": [
                {"type": b.type, "start_s": b.start_s, "duration_s": b.duration_s} for b in background
            ],
        }
        if collect_series:
            record["_series"] = series
        evidence.records.append(record)
    return evidence


def series_csv(record: dict) -> str:
    rows = record["_series"]
    buf = io.StringIO()
    writer = csv.DictWriter(buf, fieldnames=list(rows[0]))
    writer.writeheader()
    writer.writerows(rows)
    return buf.getvalue()


def to_markdown(ev: ScenarioEvidence) -> str:
    lines = [
        f"## Fault scenario `{ev.scenario}` (seed {ev.seed}, {ev.duration_s:g} s)",
        "",
        "| Fault | Window | Injected | Expected | Observed | Detect | Recovery | Result |",
        "| --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for r in ev.records:
        f = r["fault"]
        params = ", ".join(f"{k}={v}" for k, v in f["params"].items())
        window = f"{f['start_s']:g}-{f['start_s'] + f['duration_s']:g} s"
        inj = ", ".join(f"{k}={v}" for k, v in r["injected"].items() if v)
        expected = "; ".join(f"{c['name']} {c['expected']}" for c in r["checks"])
        observed = "; ".join(f"{c['name']}={c['observed']}" for c in r["checks"])
        det = r["observed"]["detection_time_s"]
        rec = r["recovery_time_s"]
        lines.append(
            f"| `{f['type']}` {params} | {window} | {inj or 'none'} | {expected} | {observed} | "
            f"{'-' if det is None else f'{det:g} s'} | {'never' if rec is None else f'{rec:g} s'} | "
            f"**{r['status']}**{' (' + r['known_gap'] + ')' if r['known_gap'] else ''} |"
        )
    lines.append("")
    lines.append(f"Gate: {'OK' if ev.ok else 'FAIL'} (known gaps do not fail the gate but are never reported as passing).")
    return "\n".join(lines) + "\n"
