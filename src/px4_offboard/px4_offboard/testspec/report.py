"""The machine-readable verification report for one run.

Built only from what the run recorded: the state history, step results,
assertion results over the telemetry file, and the adapter's own environment
label. Requirement results are traced to requirements/registry.yaml but are
scoped to this run's environment: a mock run never changes a registry verdict
(that is tools/validation_report.py's job) and says so in every row.
"""

from __future__ import annotations

import subprocess
from typing import Any

from px4_offboard.validation.registry import ROOT, Registry

from .actions import COMPLETED
from .assertions import FAIL, NOT_EVALUATED, PASS, AssertionResult

SCHEMA = 1
KIND = "test_spec_verification_report"

PASSED_REQ = "PASS"
FAILED_REQ = "FAIL"
INCOMPLETE_REQ = "INCOMPLETE"


def git_source() -> dict[str, Any]:
    def git(*args: str) -> str | None:
        try:
            return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip()
        except (OSError, subprocess.CalledProcessError):
            return None

    commit = git("rev-parse", "HEAD")
    status = git("status", "--porcelain")
    return {
        "git_commit": commit or "unknown",
        "git_dirty": None if status is None else bool(status),
    }


def requirement_results(
    registry: Registry | None,
    requirement_ids: tuple[str, ...],
    assertions: list[AssertionResult],
    mission_complete: bool,
    environment_kind: str,
) -> list[dict[str, Any]]:
    rows = []
    for rid in requirement_ids:
        linked = [a for a in assertions if rid in a.requirements]
        statuses = [a.status for a in linked]
        if FAIL in statuses:
            result = FAILED_REQ
        elif statuses and all(s == PASS for s in statuses):
            result = PASSED_REQ if mission_complete else INCOMPLETE_REQ
        else:
            result = NOT_EVALUATED
        req = registry.get(rid) if registry is not None else None
        rows.append({
            "id": rid,
            "title": req.title if req else None,
            "level": req.level if req else None,
            "registry_threshold": req.threshold if req else None,
            "result": result,
            "assertions": [a.id for a in linked],
            "evidence_scope": environment_kind,
            "changes_registry_verdict": False,
            "note": (
                f"Result of this {environment_kind} run's linked assertions only. The registry verdict for "
                f"{rid} comes from tools/validation_report.py and its own verification entries."
            ),
        })
    return rows


def build_report(run) -> dict[str, Any]:
    """``run`` is a finished runner.SpecRun."""
    spec = run.spec
    env = run.environment
    mission_complete = bool(run.steps) and all(s.status == COMPLETED for s in run.steps)
    samples = run.telemetry.samples
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "kind": KIND,
        "run_id": run.run_id,
        "verdict": run.state_machine.state.value,
        "verdict_reason": run.verdict_reason,
        "environment": env,
        "evidence_notice": run.evidence_notice,
        "spec": {
            "path": run.spec_path,
            "sha256": spec.sha256 if spec else run.spec_text_sha256,
            "name": spec.name if spec else None,
            "description": spec.description if spec else None,
            "config": spec.raw if spec else None,
            "validation_error": run.validation_error,
        },
        "source": run.source,
        "timestamps": {
            "created_utc": run.state_machine.history[0].at_utc,
            "running_utc": next((t.at_utc for t in run.state_machine.history if t.state.value == "RUNNING"), None),
            "finished_utc": run.state_machine.history[-1].at_utc,
            "wall_duration_s": run.wall_duration_s,
        },
        "adapter_clock": {
            "first_sample_t_s": samples[0].t_s if samples else None,
            "last_sample_t_s": samples[-1].t_s if samples else None,
        },
        "state_history": [{"state": t.state.value, "at_utc": t.at_utc, "reason": t.reason}
                          for t in run.state_machine.history],
        "mission_complete": mission_complete,
        "steps": [s.as_dict() for s in run.steps],
        "assertions": [a.as_dict(run.telemetry_file) for a in run.assertion_results],
        "requirements": requirement_results(
            run.registry, spec.requirement_ids if spec else (), run.assertion_results,
            mission_complete, env.get("kind", "unknown") if env else "unknown",
        ),
        "telemetry": {
            "file": run.telemetry_file if samples else None,
            "sha256": run.telemetry_sha256,
            "samples": len(samples),
            "source": "VehicleInterface.state after each adapter tick; not synthesized or edited",
        },
        "log": {"file": run.log_file},
        "summary": {
            "steps_completed": sum(1 for s in run.steps if s.status == COMPLETED),
            "steps_total": len(run.steps),
            "assertions_passed": sum(1 for a in run.assertion_results if a.status == PASS),
            "assertions_failed": sum(1 for a in run.assertion_results if a.status == FAIL),
            "assertions_not_evaluated": sum(1 for a in run.assertion_results if a.status == NOT_EVALUATED),
            "assertions_total": len(run.assertion_results),
        },
    }
    return report
