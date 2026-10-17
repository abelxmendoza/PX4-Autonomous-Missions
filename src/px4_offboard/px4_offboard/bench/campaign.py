"""Scripted test campaign against the virtual rig: setpoints, fault windows,
pass/fail requirements and evidence -- the bench equivalent of the flight
verifiers in swarm_verify.py / vv_replay.py.

Runs entirely in-process against VirtualTestRig.step(); it does not drive the
campaign over the Modbus wire. The live protocol path (modbus_server.py) is
exercised separately (tools/bench_modbus_smoke.py) because real socket/thread
timing is not something to pin as deterministic committed evidence.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

from .virtual_rig import FaultWindow, RigConfig, RigSample, VirtualTestRig, sample_to_dict


@dataclass(frozen=True)
class Setpoint:
    start_s: float
    command_pct: float


@dataclass(frozen=True)
class CampaignSpec:
    name: str
    duration_s: float
    setpoints: Sequence[Setpoint]
    faults: Sequence[FaultWindow] = field(default_factory=tuple)
    config: RigConfig = RigConfig()
    voltage_floor_v: float = 9.0


@dataclass(frozen=True)
class RequirementResult:
    req_id: str
    description: str
    passed: bool
    detail: str


def _command_at(setpoints: Sequence[Setpoint], t: float) -> float:
    command = 0.0
    for sp in sorted(setpoints, key=lambda s: s.start_s):
        if sp.start_s <= t:
            command = sp.command_pct
    return command


def run_campaign(spec: CampaignSpec) -> tuple[list[RigSample], list[RequirementResult]]:
    rig = VirtualTestRig(spec.config)
    for fault in spec.faults:
        rig.add_fault(fault)

    n_steps = int(round(spec.duration_s / spec.config.dt_s))
    samples = [rig.step(_command_at(spec.setpoints, i * spec.config.dt_s)) for i in range(n_steps)]
    return samples, evaluate_requirements(samples, spec)


def detect_stuck_sensor(samples: Sequence[RigSample], key: str = "temp_c", min_run_s: float = 0.5) -> list[float]:
    """Times at which a channel is first flagged as stuck.

    A live sensor reading always carries some noise, so it never repeats exactly. A reading
    that has not changed at all for ``min_run_s`` is flagged once, at the moment the run
    reaches that length; the flag re-arms when the reading moves again.
    """
    flagged: list[float] = []
    run_start: float | None = None
    armed = True
    for prev, cur in zip(samples, samples[1:]):
        if getattr(cur, key) == getattr(prev, key):
            run_start = prev.t if run_start is None else run_start
            if armed and cur.t - run_start >= min_run_s - 1e-9:
                flagged.append(cur.t)
                armed = False
        else:
            run_start, armed = None, True
    return flagged


def evaluate_requirements(samples: Sequence[RigSample], spec: CampaignSpec) -> list[RequirementResult]:
    results: list[RequirementResult] = []

    neg_thrust = [s for s in samples if s.thrust_n < -1e-6]
    results.append(RequirementResult(
        "REQ-BENCH-001",
        "Thrust is never negative",
        passed=not neg_thrust,
        detail=f"{len(neg_thrust)} sample(s) with negative thrust" if neg_thrust else "no negative-thrust samples",
    ))

    dropout_windows = [f for f in spec.faults if f.name == "load_cell_dropout"]
    if dropout_windows:
        missed = [
            w for w in dropout_windows
            if not any(
                w.start_s <= s.t < w.start_s + w.duration_s
                and s.fault_active == "load_cell_dropout" and s.load_cell_n == 0.0
                for s in samples
            )
        ]
        results.append(RequirementResult(
            "REQ-BENCH-002",
            "A load-cell dropout fault is observable in the reading (reads zero, not silently absorbed)",
            passed=not missed,
            detail="dropout read zero in every injected window" if not missed
            else f"{len(missed)} dropout window(s) had no visible effect on the reading",
        ))

    brownouts = [
        s for s in samples
        if s.voltage_v < spec.voltage_floor_v and s.fault_active != "voltage_brownout"
    ]
    results.append(RequirementResult(
        "REQ-BENCH-003",
        f"Battery voltage stays above {spec.voltage_floor_v} V outside an injected brownout",
        passed=not brownouts,
        detail=f"{len(brownouts)} sample(s) below the floor" if brownouts else "voltage stayed above the floor",
    ))

    frozen_windows = [f for f in spec.faults if f.name == "frozen_temp_sensor"]
    stuck = detect_stuck_sensor(samples, "temp_c")
    in_window = lambda t: any(w.start_s <= t < w.start_s + w.duration_s for w in frozen_windows)  # noqa: E731
    missed_freezes = [w for w in frozen_windows if not any(w.start_s <= t <= w.start_s + 1.0 for t in stuck)]
    false_alarms = [t for t in stuck if not in_window(t)]
    results.append(RequirementResult(
        "REQ-BENCH-005",
        "A frozen temperature sensor is detected within 1 s, with no false alarms",
        passed=not missed_freezes and not false_alarms,
        detail=(f"{len(missed_freezes)} freeze(s) not detected within 1 s" if missed_freezes else
                f"{len(false_alarms)} false alarm(s) on a healthy reading" if false_alarms else
                ("stuck reading flagged at " + ", ".join(f"{t:.2f} s" for t in stuck)) if stuck else
                "no freeze injected and none flagged"),
    ))

    gap_tolerance_s = spec.config.dt_s * 1.5
    gaps = [(a.t, b.t) for a, b in zip(samples, samples[1:]) if b.t - a.t > gap_tolerance_s]
    results.append(RequirementResult(
        "REQ-BENCH-004",
        "The sample timeline has no gaps beyond 1.5x the tick interval",
        passed=not gaps,
        detail=f"{len(gaps)} gap(s)" if gaps else "no gaps",
    ))

    return sorted(results, key=lambda r: r.req_id)


def report_dict(spec: CampaignSpec, samples: Sequence[RigSample], results: Sequence[RequirementResult]) -> dict:
    return {
        "campaign": spec.name,
        "duration_s": spec.duration_s,
        "dt_s": spec.config.dt_s,
        "seed": spec.config.seed,
        "sample_count": len(samples),
        "faults": [{"name": f.name, "start_s": f.start_s, "duration_s": f.duration_s} for f in spec.faults],
        "requirements": [
            {"id": r.req_id, "description": r.description, "passed": r.passed, "detail": r.detail}
            for r in results
        ],
        "passed": all(r.passed for r in results),
        # What the automatic checks noticed, and when (the replay's narration reads these).
        "events": [{"t": round(t, 3), "event": "stuck_sensor_flagged", "channel": "temp_c"}
                   for t in detect_stuck_sensor(samples, "temp_c")],
    }


def golden_spec() -> CampaignSpec:
    """The committed golden campaign: a throttle sweep with two sensor faults."""
    return CampaignSpec(
        name="bench_thrust_sweep",
        duration_s=20.0,
        setpoints=[
            Setpoint(0.0, 0.0),
            Setpoint(2.0, 30.0),
            Setpoint(6.0, 60.0),
            Setpoint(10.0, 90.0),
            Setpoint(14.0, 50.0),
            Setpoint(17.0, 0.0),
        ],
        faults=[
            FaultWindow("load_cell_dropout", start_s=7.0, duration_s=2.0),
            FaultWindow("frozen_temp_sensor", start_s=11.0, duration_s=2.0),
        ],
        config=RigConfig(dt_s=0.05, seed=7),
    )


def samples_to_jsonl_rows(samples: Sequence[RigSample]) -> list[dict]:
    return [sample_to_dict(s) for s in samples]
