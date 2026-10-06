"""Evaluate registry requirements from evidence. A requirement is PASS only if
something was actually measured or run: absent evidence is NOT_RUN or FAIL,
never a quiet pass."""

from __future__ import annotations

import gzip
import hashlib
import json
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

from px4_offboard.faults.evidence import run_scenario
from px4_offboard.faults.schema import load_scenario
from px4_offboard.flight_replay import load_flight_log
from px4_offboard.vv_harness import run_vv

from .registry import ROOT, Registry, RegistryRequirement


@dataclass
class VerificationResult:
    kind: str
    ref: str
    status: str  # PASS | FAIL | NOT_RUN
    detail: str
    evidence: list[str] = field(default_factory=list)
    children: list["VerificationResult"] = field(default_factory=list)


@dataclass
class RequirementResult:
    id: str
    title: str
    level: str
    status: str  # PASS | FAIL | PARTIAL | NOT_RUN
    verifications: list[VerificationResult]
    tests: list[tuple[str, str]]
    known_open: str | None
    requirement: RegistryRequirement | None = None


class EvalContext:
    """Caches expensive evaluations and collects evidence artifacts to write."""

    def __init__(self, base_dir: Path = ROOT) -> None:
        self.base_dir = Path(base_dir)
        self.flights: dict[str, tuple[dict, dict]] = {}
        self.scenarios: dict[str, Any] = {}
        self.artifacts: dict[str, Any] = {}
        self._manifest: dict[str, dict] | None = None

    def manifest_case(self, case_id: str) -> dict:
        if self._manifest is None:
            data = json.loads((ROOT / "evidence" / "manifest.json").read_text())
            self._manifest = {c["id"]: c for c in data["cases"]}
        return self._manifest[case_id]

    def flight_report(self, case_id: str) -> tuple[dict, dict]:
        if case_id not in self.flights:
            case = self.manifest_case(case_id)
            raw = (ROOT / case["path"]).read_bytes()
            digest = hashlib.sha256(raw).hexdigest()
            if digest != case["sha256"]:
                raise ValueError(f"{case['path']}: SHA-256 differs from the evidence manifest")
            with tempfile.TemporaryDirectory() as d:
                src = Path(d) / "flight.csv"
                src.write_bytes(gzip.decompress(raw) if case["path"].endswith(".gz") else raw)
                report = run_vv(load_flight_log(src)).to_dict()
            self.flights[case_id] = (case, {r["requirement_id"]: r for r in report["results"]})
        return self.flights[case_id]

    def scenario(self, name: str):
        if name not in self.scenarios:
            scenario = load_scenario(ROOT / "config" / "fault_scenarios" / f"{name}.yaml")
            self.scenarios[name] = run_scenario(scenario)
            self.artifacts[f"faults/{name}.json"] = self.scenarios[name].to_dict()
        return self.scenarios[name]


# --- pytest outcomes ------------------------------------------------------------

def parse_junit(paths: Iterable[Path]) -> dict[tuple[str, str], str]:
    outcomes: dict[tuple[str, str], str] = {}
    for path in paths:
        for case in ET.parse(path).getroot().iter("testcase"):
            if case.find("failure") is not None or case.find("error") is not None:
                outcome = "failed"
            elif case.find("skipped") is not None:
                outcome = "skipped"
            else:
                outcome = "passed"
            outcomes[(case.get("classname", ""), case.get("name", ""))] = outcome
    return outcomes


def run_pytest_for(refs: Iterable[str], junit_path: Path) -> None:
    files = sorted({r.split("::")[0] for r in refs})
    subprocess.run(
        [sys.executable, "-m", "pytest", "-q", f"--junitxml={junit_path}", *files],
        cwd=ROOT, capture_output=True, text=True,
    )


def match_tests(refs: Iterable[str], outcomes: dict[tuple[str, str], str]) -> list[tuple[str, str]]:
    result = []
    for ref in refs:
        path, _, leaf = ref.partition("::")
        classname = path[:-3].replace("/", ".") if path.endswith(".py") else path.replace("/", ".")
        found = [
            o for (c, n), o in outcomes.items()
            if c == classname and (not leaf or n == leaf or n.startswith(leaf + "["))
        ]
        if not found:
            outcome = "missing"
        elif "failed" in found:
            outcome = "failed"
        elif "skipped" in found:
            outcome = "skipped"
        else:
            outcome = "passed"
        result.append((ref, outcome))
    return result


# --- verification kinds ---------------------------------------------------------

def _fold(children: list[VerificationResult]) -> str:
    if any(c.status == "FAIL" for c in children):
        return "FAIL"
    if children and all(c.status == "PASS" for c in children):
        return "PASS"
    return "NOT_RUN"


def _eval_flight(v: dict, ctx: EvalContext) -> VerificationResult:
    children, evidence = [], []
    for case_id in v["cases"]:
        case, checks = ctx.flight_report(case_id)
        evidence.append(case["path"])
        for check in v["checks"]:
            r = checks.get(check)
            if r is None or r["skipped"]:
                status, detail = "NOT_RUN", "check skipped or absent in this recording"
            else:
                status, detail = ("PASS" if r["passed"] else "FAIL"), r["detail"]
            children.append(VerificationResult("flight", f"{case_id}:{check}", status, detail))
    return VerificationResult(
        "flight", "+".join(v["cases"]), _fold(children),
        "; ".join(f"{c.ref} {c.status}: {c.detail}" for c in children), evidence, children,
    )


def _eval_fault(v: dict, ctx: EvalContext) -> VerificationResult:
    evidence = ctx.scenario(v["scenario"])
    children = []
    for n in v["faults"]:
        record = evidence.records[n - 1]
        wanted = v.get("checks")
        checks = [c for c in record["checks"] if not wanted or c["name"] in wanted]
        ok = bool(checks) and all(c["passed"] for c in checks)
        detail = ", ".join(f"{c['name']}={c['observed']} (limit {c['expected']})" for c in checks)
        children.append(VerificationResult(
            "fault", f"{v['scenario']}#{n}:{record['fault']['type']}",
            "PASS" if ok else "FAIL", detail,
        ))
    return VerificationResult(
        "fault", v["scenario"], _fold(children),
        "; ".join(f"{c.ref} {c.status}: {c.detail}" for c in children),
        [f"faults/{v['scenario']}.json"], children,
    )


def _eval_external(v: dict, base_dir: Path) -> VerificationResult:
    path = base_dir / v["results_file"]
    ref = f"{v['requires']}: {v['results_file']}"
    if not path.is_file():
        return VerificationResult("external", ref, "NOT_RUN", f"not run -- {v['reason']}")
    try:
        data = json.loads(path.read_text())
        passed = bool(data["passed"])
        detail = str(data.get("detail", ""))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return VerificationResult("external", ref, "FAIL", f"unreadable results file: {exc}", [v["results_file"]])
    return VerificationResult("external", ref, "PASS" if passed else "FAIL", detail, [v["results_file"]])


def aggregate(verifications: list[VerificationResult]) -> str:
    statuses = [v.status for v in verifications]
    if "FAIL" in statuses:
        return "FAIL"
    if all(s == "PASS" for s in statuses):
        return "PASS"
    if any(s == "PASS" for s in statuses):
        return "PARTIAL"
    return "NOT_RUN"


def evaluate_requirement(
    req: RegistryRequirement,
    outcomes: dict[tuple[str, str], str],
    base_dir: Path = ROOT,
    ctx: EvalContext | None = None,
) -> RequirementResult:
    ctx = ctx or EvalContext(base_dir)
    tests = match_tests(req.tests, outcomes)
    verifications: list[VerificationResult] = []
    has_pytest = False
    for v in req.verification:
        if v["kind"] == "flight":
            verifications.append(_eval_flight(v, ctx))
        elif v["kind"] == "fault":
            verifications.append(_eval_fault(v, ctx))
        elif v["kind"] == "external":
            verifications.append(_eval_external(v, Path(base_dir)))
        else:
            has_pytest = True
            bad = [f"{ref} ({o})" for ref, o in tests if o != "passed"]
            verifications.append(VerificationResult(
                "pytest", f"{len(tests)} test(s)", "FAIL" if bad else "PASS",
                "no test did anything other than pass" if not bad else "not passing: " + "; ".join(bad),
            ))
    if not has_pytest:
        failed = [ref for ref, o in tests if o == "failed"]
        if failed:  # a failing supporting test must not hide behind good flight data
            verifications.append(VerificationResult(
                "pytest", "supporting tests", "FAIL", "failing: " + "; ".join(failed)))
    return RequirementResult(
        req.id, req.title, req.level, aggregate(verifications), verifications, tests, req.known_open, req
    )


def evaluate_registry(
    registry: Registry,
    outcomes: dict[tuple[str, str], str],
    base_dir: Path = ROOT,
    ctx: EvalContext | None = None,
) -> list[RequirementResult]:
    ctx = ctx or EvalContext(base_dir)
    return [evaluate_requirement(r, outcomes, base_dir, ctx) for r in registry.requirements]


def gate(results: Iterable[Any]) -> tuple[bool, list[str]]:
    """CI verdict. Tolerated: a FAIL that is declared known_open, and an
    external (hardware/SITL) verification that was not run. Not tolerated:
    anything else failing, missing evidence, or a known_open marker on a
    requirement that no longer fails."""
    problems = []
    for r in results:
        if r.known_open and r.status != "FAIL":
            problems.append(f"{r.id}: marked known_open but is {r.status} -- remove the stale marker")
        elif r.status == "FAIL" and not r.known_open:
            problems.append(f"{r.id}: FAIL")
        if r.status in ("PARTIAL", "NOT_RUN"):
            missing = [v for v in r.verifications if v.status == "NOT_RUN" and v.kind != "external"]
            if missing or not r.verifications:
                problems.append(f"{r.id}: {r.status} without evidence ({', '.join(v.kind for v in missing)})")
    return (not problems, problems)
