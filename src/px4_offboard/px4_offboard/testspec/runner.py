"""Run one test spec through the explicit lifecycle in states.py.

Outcome classes:
  FAILED   the target was exercised and did not meet the spec: a step timed out
           or was rejected, or an assertion failed. An incomplete mission is FAILED.
  ERROR    the run could not produce a valid verdict: invalid or unreadable spec,
           registry unavailable, adapter failure, no telemetry, a step timeout
           while the vehicle link is not alive, an exception in the runner, or
           the wall-clock budget exceeded.
  ABORTED  cancelled (``cancel`` event set, e.g. Ctrl-C) before a verdict.
  PASSED   every step completed and every assertion was evaluated and passed.

Artifacts land in ``<out>/<run_id>/``: report.json, telemetry.csv, run.log.jsonl.
"""

from __future__ import annotations

import hashlib
import json
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

from px4_offboard.validation.registry import ROOT, Registry, RegistryError, load_registry

from . import actions
from .actions import COMPLETED, StepResult
from .adapters import AdapterError, VehicleAdapter, build_adapter
from .assertions import NOT_EVALUATED, PASS, AssertionResult, evaluate
from .report import build_report, git_source
from .schema import Spec, SpecError, parse_spec_text
from .states import RunState, RunStateMachine, utc_now
from .telemetry import Sample, TelemetryLog

DEFAULT_OUT = ROOT / "artifacts" / "test_runs"


class RunCancelled(Exception):
    pass


class WallBudgetExceeded(Exception):
    pass


def new_run_id() -> str:
    return f"{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}-{uuid.uuid4().hex[:8]}"


class RunLog:
    """Structured log: one JSON object per line, flushed as written."""

    def __init__(self, path: Path, run_id: str, echo: Callable[[str], None] | None = None) -> None:
        self._fh = path.open("x")
        self._run_id = run_id
        self._echo = echo

    def event(self, event: str, **fields: Any) -> None:
        record = {"ts_utc": utc_now(), "run_id": self._run_id, "event": event, **fields}
        line = json.dumps(record, default=str)
        self._fh.write(line + "\n")
        self._fh.flush()
        if self._echo is not None:
            self._echo(line)

    def close(self) -> None:
        self._fh.close()


class SpecRun:
    telemetry_file = "telemetry.csv"
    log_file = "run.log.jsonl"
    report_file = "report.json"

    def __init__(
        self,
        spec_path: str | Path,
        out_root: str | Path = DEFAULT_OUT,
        *,
        registry: Registry | None = None,
        adapter_factory: Callable[..., VehicleAdapter] = build_adapter,
        cancel: threading.Event | None = None,
        wall_clock: Callable[[], float] = time.monotonic,
        run_id: str | None = None,
        echo: Callable[[str], None] | None = None,
    ) -> None:
        self.run_id = run_id or new_run_id()
        self.spec_path = str(spec_path)
        self.run_dir = Path(out_root) / self.run_id
        self.run_dir.mkdir(parents=True, exist_ok=False)  # a reused id must not overwrite evidence
        self.log = RunLog(self.run_dir / self.log_file, self.run_id, echo)
        self.state_machine = RunStateMachine(self._on_transition)
        self.registry = registry
        self._adapter_factory = adapter_factory
        self.cancel = cancel or threading.Event()
        self._wall_clock = wall_clock
        self._wall_start = wall_clock()
        self.wall_duration_s: float | None = None
        self.spec: Spec | None = None
        self.spec_text_sha256: str | None = None
        self.validation_error: str | None = None
        self.adapter: VehicleAdapter | None = None
        self._stopped = False
        self.environment: dict[str, Any] | None = None
        self.telemetry = TelemetryLog()
        self.telemetry_sha256: str | None = None
        self.steps: list[StepResult] = []
        self.assertion_results: list[AssertionResult] = []
        self.verdict_reason = ""
        self.source = git_source()
        self._current_step = -1
        self.rate_hz = 0.0
        self.vehicle = None
        self.log.event("run_created", spec=self.spec_path, out=str(self.run_dir), **self.source)

    # --- StepContext for actions ------------------------------------------------
    def now(self) -> float:
        return self.adapter.clock.now()

    def tick(self, command: tuple[float, float, float] | None) -> Sample:
        if self.cancel.is_set():
            raise RunCancelled("cancel requested")
        if self._wall_clock() - self._wall_start > self.spec.timeouts.run_wall_s:
            raise WallBudgetExceeded(f"run exceeded its {self.spec.timeouts.run_wall_s} s wall-clock budget")
        if command is not None:
            self.vehicle.set_velocity(*command)
        self.adapter.tick()
        state = self.vehicle.state
        if state is None:
            raise AdapterError("vehicle produced no state after a tick")
        return self.telemetry.record(state, self._current_step, command)

    # --- lifecycle ----------------------------------------------------------------
    def _on_transition(self, old: RunState, new: RunState, reason: str) -> None:
        self.log.event("state", from_state=old.value, to_state=new.value, reason=reason)

    @property
    def evidence_notice(self) -> str:
        kind = (self.environment or {}).get("kind")
        if kind == "mock":
            return ("Mock execution against SimVehicle on simulated time. This is not a PX4 SITL, "
                    "HIL or flight result and verifies only the spec, runner and assertion logic "
                    "against a point-mass test double.")
        return "No vehicle was brought up; nothing was executed."

    def execute(self) -> dict[str, Any]:
        sm = self.state_machine
        try:
            self._validate()
            sm.to(RunState.VALIDATED, f"spec {self.spec.name} sha256 {self.spec.sha256[:12]}")
            self._initialize()
            sm.to(RunState.READY, f"first telemetry at t={self.telemetry.samples[0].t_s:.3f} s")
            sm.to(RunState.RUNNING, f"{len(self.spec.mission)} step(s)")
            self._run_mission()
            self._stop_adapter()
            sm.to(RunState.EVALUATING, f"{len(self.telemetry.samples)} telemetry sample(s)")
            self._evaluate()
        except SpecError as exc:
            self.validation_error = str(exc)
            self._terminal(RunState.ERROR, f"spec invalid: {exc}")
        except RunCancelled as exc:
            self._terminal(RunState.ABORTED, str(exc))
        except WallBudgetExceeded as exc:
            self._terminal(RunState.ERROR, str(exc))
        except Exception as exc:  # anything unexpected is infrastructure, never a pass
            self._terminal(RunState.ERROR, f"{type(exc).__name__}: {exc}")
        finally:
            self._stop_adapter(quiet=True)
            self.wall_duration_s = self._wall_clock() - self._wall_start
        return self._write()

    def _terminal(self, state: RunState, reason: str) -> None:
        self.verdict_reason = reason
        if not self.state_machine.terminal:
            self.state_machine.to(state, reason)

    def _validate(self) -> None:
        try:
            text = Path(self.spec_path).read_text()
        except OSError as exc:
            raise SpecError(f"cannot read spec: {exc}") from exc
        self.spec_text_sha256 = hashlib.sha256(text.encode()).hexdigest()
        if self.registry is None:
            try:
                self.registry = load_registry()
            except (OSError, RegistryError) as exc:
                raise RuntimeError(f"requirement registry unavailable: {exc}") from exc
        known = {r.id for r in self.registry.requirements}
        self.spec = parse_spec_text(text, known, self.spec_path)
        self.log.event("spec_validated", name=self.spec.name, sha256=self.spec.sha256,
                       adapter=self.spec.vehicle.adapter, steps=len(self.spec.mission),
                       assertions=len(self.spec.assertions))

    def _initialize(self) -> None:
        self.state_machine.to(RunState.INITIALIZING, f"adapter {self.spec.vehicle.adapter}")
        self.steps = [StepResult(s.index, s.id, s.action, s.params) for s in self.spec.mission]
        self.adapter = self._adapter_factory(self.spec.vehicle)
        self.environment = self.adapter.environment
        self.vehicle = self.adapter.vehicle
        self.rate_hz = self.adapter.rate_hz
        self.log.event("adapter_selected", environment=self.environment)
        self.adapter.start()
        t0 = self.now()
        while True:
            if self.cancel.is_set():
                raise RunCancelled("cancel requested during initialization")
            if self._wall_clock() - self._wall_start > self.spec.timeouts.run_wall_s:
                raise WallBudgetExceeded("wall-clock budget exceeded during initialization")
            self.adapter.tick()
            if self.vehicle.state is not None:
                self.telemetry.record(self.vehicle.state, -1, None)
                return
            if self.now() - t0 >= self.spec.timeouts.telemetry_s:
                raise AdapterError(f"no telemetry within {self.spec.timeouts.telemetry_s} s of start")

    def _run_mission(self) -> None:
        last = self.telemetry.samples[-1]
        for step, result in zip(self.spec.mission, self.steps):
            self._current_step = step.index
            self.log.event("step_start", step=step.index, id=step.id, action=step.action, params=step.params)
            last = actions.execute(self, step, last, result)
            self.log.event("step_end", step=step.index, id=step.id, status=result.status, detail=result.detail,
                           t_start_s=result.t_start_s, t_end_s=result.t_end_s)
            if result.status == actions.TIMEOUT and not self.vehicle.link_health().alive:
                # A target that stopped talking did not fail the test; the run has no verdict.
                raise AdapterError(f"vehicle link not alive when step {step.id} timed out")
            if result.status != COMPLETED:
                self.log.event("mission_stopped", at_step=step.index,
                               not_run=[s.id for s in self.steps if s.status == actions.NOT_RUN])
                return

    def _stop_adapter(self, quiet: bool = False) -> None:
        if self.adapter is None or self._stopped:
            return
        self._stopped = True
        try:
            self.adapter.stop()
            self.log.event("adapter_stopped")
        except Exception as exc:
            self.log.event("adapter_stop_failed", error=f"{type(exc).__name__}: {exc}")
            if not quiet:
                raise

    def _evaluate(self) -> None:
        samples = self.telemetry.samples
        for a in self.spec.assertions:
            result = evaluate(a, samples)
            self.assertion_results.append(result)
            self.log.event("assertion", id=a.id, type=a.type, status=result.status,
                           observed=result.observed, deciding_sample=result.worst_sample)
        incomplete = [s for s in self.steps if s.status != COMPLETED]
        failed = [a for a in self.assertion_results if a.status != PASS and a.status != NOT_EVALUATED]
        unevaluated = [a for a in self.assertion_results if a.status == NOT_EVALUATED]
        if unevaluated:
            self._terminal(RunState.ERROR, f"assertion(s) not evaluated: {[a.id for a in unevaluated]}")
        elif incomplete or failed:
            reasons = []
            if incomplete:
                first = incomplete[0]
                reasons.append(f"mission incomplete: step {first.id} {first.status}")
            if failed:
                reasons.append(f"assertion(s) failed: {[a.id for a in failed]}")
            self._terminal(RunState.FAILED, "; ".join(reasons))
        else:
            self._terminal(RunState.PASSED, f"{len(self.steps)} step(s) completed, "
                                            f"{len(self.assertion_results)} assertion(s) passed")

    def _write(self) -> dict[str, Any]:
        if self.telemetry.samples:
            text = self.telemetry.to_csv()
            self.telemetry_sha256 = TelemetryLog.sha256(text)
            (self.run_dir / self.telemetry_file).write_text(text)
        report = build_report(self)
        (self.run_dir / self.report_file).write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        self.log.event("run_finished", verdict=report["verdict"], reason=self.verdict_reason,
                       report=str(self.run_dir / self.report_file))
        self.log.close()
        return report
