"""Registry integrity: every claim must point at something that exists."""
from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

import pytest

from px4_offboard.faults.schema import FAULT_TYPES, load_scenario
from px4_offboard.validation.registry import Registry, RegistryError, load_registry, parse_registry_text

ROOT = Path(__file__).resolve().parents[3]
REGISTRY = load_registry()


def test_ids_are_unique_and_well_formed():
    ids = [r.id for r in REGISTRY.requirements]
    assert len(ids) == len(set(ids))
    assert all(re.fullmatch(r"REQ-[A-Z]+-\d{3}", i) for i in ids)


def test_the_families_the_project_promises_exist():
    families = {r.id.split("-")[1] for r in REGISTRY.requirements}
    assert {"EST", "CTRL", "COMMS", "RECOVERY", "HIL", "SAFE"} <= families


def test_every_requirement_is_fully_described():
    for r in REGISTRY.requirements:
        for name in ("title", "description", "rationale", "threshold", "measurement"):
            assert getattr(r, name).strip(), f"{r.id} has empty {name}"
        assert r.level in ("MUST", "SHOULD")
        assert r.verification, f"{r.id} has no verification"


def test_flight_cases_exist_in_the_evidence_manifest():
    import json

    cases = {c["id"] for c in json.loads((ROOT / "evidence/manifest.json").read_text())["cases"]}
    for r in REGISTRY.requirements:
        for v in r.verification:
            if v["kind"] == "flight":
                assert set(v["cases"]) <= cases, r.id


def test_fault_references_point_at_real_scenarios_faults_and_checks():
    for r in REGISTRY.requirements:
        for v in r.verification:
            if v["kind"] != "fault":
                continue
            scenario = load_scenario(ROOT / "config" / "fault_scenarios" / f"{v['scenario']}.yaml")
            for n in v["faults"]:
                assert 1 <= n <= len(scenario.faults), f"{r.id}: fault {n} not in {v['scenario']}"
                known = set(scenario.faults[n - 1].expect)
                for check in v.get("checks", []):
                    assert check in known, f"{r.id}: {check} is not an expectation of fault {n}"


def test_pytest_verification_needs_tests_and_every_test_reference_exists():
    refs = []
    for r in REGISTRY.requirements:
        if any(v["kind"] == "pytest" for v in r.verification):
            assert r.tests, f"{r.id} verifies by pytest but lists no tests"
        refs += [(r.id, t) for t in r.tests]
    files = {t.split("::")[0] for _, t in refs}
    out = subprocess.run(
        [sys.executable, "-m", "pytest", "--collect-only", "-q", *sorted(files)],
        cwd=ROOT, capture_output=True, text=True,
    ).stdout
    collected = {line.strip() for line in out.splitlines() if "::" in line}
    for rid, ref in refs:
        if "::" in ref:
            assert any(c == ref or c.startswith(ref + "[") for c in collected), f"{rid}: no such test {ref}"
        else:
            assert (ROOT / ref).is_file(), f"{rid}: no such file {ref}"


def test_known_open_requirements_are_only_the_ones_we_document():
    assert {r.id for r in REGISTRY.requirements if r.known_open} == {"REQ-EST-001", "REQ-EST-002", "REQ-EST-006"}


def test_external_verifications_say_why_they_cannot_run_in_ci():
    for r in REGISTRY.requirements:
        for v in r.verification:
            if v["kind"] == "external":
                assert v["requires"] in ("hardware", "px4_sitl") and v["reason"] and v["results_file"]


def test_the_requirements_document_lists_every_registered_id():
    doc = (ROOT / "docs" / "REQUIREMENTS.md").read_text()
    missing = [r.id for r in REGISTRY.requirements if r.id not in doc]
    assert not missing, missing


def test_loader_rejects_a_bad_level():
    base = """
schema: 1
requirements:
  - id: REQ-X-001
    title: t
    level: MAYBE
    description: d
    rationale: r
    threshold: th
    measurement: m
    verification: [{kind: pytest}]
    tests: [a.py]
"""
    with pytest.raises(RegistryError) as exc:
        parse_registry_text(base)
    assert "level" in str(exc.value)


def test_loader_rejects_unknown_verification_kind_and_duplicate_ids():
    entry = """  - id: REQ-X-001
    title: t
    level: MUST
    description: d
    rationale: r
    threshold: th
    measurement: m
    verification: [{kind: %s}]
    tests: [a.py]
"""
    with pytest.raises(RegistryError):
        parse_registry_text("schema: 1\nrequirements:\n" + entry % "telepathy")
    with pytest.raises(RegistryError):
        parse_registry_text("schema: 1\nrequirements:\n" + entry % "pytest" + entry % "pytest")
