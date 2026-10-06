"""Evaluation, gating and report generation for the requirement registry."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from px4_offboard.validation.evaluate import (
    VerificationResult,
    aggregate,
    evaluate_registry,
    evaluate_requirement,
    gate,
    match_tests,
    parse_junit,
)
from px4_offboard.validation.registry import load_registry, parse_registry_text
from px4_offboard.validation.report import build_report, to_markdown

ROOT = Path(__file__).resolve().parents[3]


def vr(status, kind="flight"):
    return VerificationResult(kind=kind, ref="x", status=status, detail="", evidence=[])


@pytest.mark.parametrize(
    "statuses,expected",
    [
        (["PASS"], "PASS"),
        (["PASS", "PASS"], "PASS"),
        (["PASS", "FAIL"], "FAIL"),
        (["FAIL", "NOT_RUN"], "FAIL"),
        (["PASS", "NOT_RUN"], "PARTIAL"),
        (["NOT_RUN"], "NOT_RUN"),
    ],
)
def test_aggregation(statuses, expected):
    assert aggregate([vr(s) for s in statuses]) == expected


class Res:  # minimal stand-in for RequirementResult
    def __init__(self, status, known_open=None, verifs=()):
        self.id, self.status, self.known_open, self.verifications = "REQ-X-001", status, known_open, list(verifs)


def test_gate_semantics():
    assert gate([Res("PASS")])[0]
    assert not gate([Res("FAIL")])[0]                       # an unexplained failure
    assert gate([Res("FAIL", known_open="BUG-1")])[0]       # a documented one is tolerated...
    assert not gate([Res("PASS", known_open="BUG-1")])[0]   # ...but a stale marker is not
    ext = [vr("PASS", "pytest"), vr("NOT_RUN", "external")]
    assert gate([Res("PARTIAL", verifs=ext)])[0]            # hardware not run: allowed, visibly
    assert not gate([Res("PARTIAL", verifs=[vr("PASS"), vr("NOT_RUN", "flight")])])[0]  # evidence missing: not allowed
    assert not gate([Res("NOT_RUN", verifs=[vr("NOT_RUN", "fault")])])[0]


JUNIT = """<testsuites><testsuite>
<testcase classname="src.px4_offboard.test.test_a" name="test_ok"/>
<testcase classname="src.px4_offboard.test.test_a" name="test_param[1]"/>
<testcase classname="src.px4_offboard.test.test_a" name="test_param[2]"><failure message="boom"/></testcase>
<testcase classname="src.px4_offboard.test.test_a" name="test_skipped"><skipped message="no pymavlink"/></testcase>
<testcase classname="src.px4_offboard.test.test_b" name="test_other"/>
</testsuite></testsuites>"""


def test_junit_matching(tmp_path):
    xml = tmp_path / "j.xml"
    xml.write_text(JUNIT)
    outcomes = parse_junit([xml])
    ok = match_tests(["src/px4_offboard/test/test_a.py::test_ok"], outcomes)
    assert ok == [("src/px4_offboard/test/test_a.py::test_ok", "passed")]
    assert match_tests(["src/px4_offboard/test/test_a.py::test_param"], outcomes)[0][1] == "failed"  # any param failing fails it
    assert match_tests(["src/px4_offboard/test/test_a.py::test_skipped"], outcomes)[0][1] == "skipped"
    assert match_tests(["src/px4_offboard/test/test_a.py::test_missing"], outcomes)[0][1] == "missing"
    assert match_tests(["src/px4_offboard/test/test_b.py"], outcomes)[0][1] == "passed"  # whole file
    assert match_tests(["src/px4_offboard/test/test_a.py"], outcomes)[0][1] == "failed"


def _registry(body: str):
    return parse_registry_text("schema: 1\nrequirements:\n" + body)


PYTEST_REQ = """  - id: REQ-X-001
    title: t
    level: MUST
    description: d
    rationale: r
    threshold: th
    measurement: m
    verification: [{kind: pytest}]
    tests: [src/px4_offboard/test/test_a.py::test_ok]
"""


def test_pytest_requirement_passes_only_with_a_passing_recorded_test(tmp_path):
    xml = tmp_path / "j.xml"
    xml.write_text(JUNIT)
    reg = _registry(PYTEST_REQ)
    res = evaluate_requirement(reg.requirements[0], outcomes=parse_junit([xml]))
    assert res.status == "PASS" and res.tests == [("src/px4_offboard/test/test_a.py::test_ok", "passed")]
    # no test evidence at all -> never a pass
    assert evaluate_requirement(reg.requirements[0], outcomes={}).status == "FAIL"


def test_external_verification_is_not_run_without_a_results_file_and_honours_one(tmp_path):
    body = PYTEST_REQ.replace("verification: [{kind: pytest}]", (
        "verification: [{kind: external, requires: hardware, results_file: r.json, reason: no bench}]"
    )).replace("tests: [src/px4_offboard/test/test_a.py::test_ok]", "tests: []")
    req = _registry(body).requirements[0]
    assert evaluate_requirement(req, outcomes={}, base_dir=tmp_path).status == "NOT_RUN"
    (tmp_path / "r.json").write_text(json.dumps({"passed": True, "detail": "bench run 1"}))
    assert evaluate_requirement(req, outcomes={}, base_dir=tmp_path).status == "PASS"
    (tmp_path / "r.json").write_text(json.dumps({"passed": False, "detail": "no heartbeat"}))
    assert evaluate_requirement(req, outcomes={}, base_dir=tmp_path).status == "FAIL"
    (tmp_path / "r.json").write_text("not json")
    assert evaluate_requirement(req, outcomes={}, base_dir=tmp_path).status == "FAIL"


def test_flight_requirement_reads_the_real_verifier_on_curated_recordings():
    reg = load_registry()
    drift = evaluate_requirement(reg.get("REQ-EST-001"), outcomes={})
    details = {v.ref: (v.status, v.detail) for v in drift.verifications[0].children}
    assert details["stereo_vo_flight_k:REQ-VO-DRIFT-01"][0] == "PASS"
    assert details["stereo_vo_flight_l:REQ-VO-DRIFT-01"][0] == "FAIL"
    assert "9.3%" in details["stereo_vo_flight_k:REQ-VO-DRIFT-01"][1]
    assert drift.status == "FAIL"           # one failing run fails the requirement
    assert drift.verifications[0].evidence  # recording path(s) cited


@pytest.fixture(scope="module")
def all_results(tmp_path_factory):
    # Evaluate everything except test outcomes, which come from this very session in CI.
    reg = load_registry()
    outcomes = {}
    return evaluate_registry(reg, outcomes=outcomes, base_dir=ROOT)


pytestmark_integration = pytest.mark.integration


@pytest.mark.integration
def test_registry_evaluates_and_every_open_item_is_visible(all_results):
    by_id = {r.id: r for r in all_results}
    assert by_id["REQ-EST-001"].status == "FAIL" and by_id["REQ-EST-001"].known_open
    assert by_id["REQ-EST-006"].status == "FAIL" and by_id["REQ-EST-006"].known_open
    assert by_id["REQ-HIL-004"].status == "NOT_RUN"
    assert by_id["REQ-HIL-003"].status in ("NOT_RUN", "PASS", "FAIL")
    for rid in ("REQ-EST-003", "REQ-EST-004", "REQ-COMMS-004", "REQ-RECOVERY-001", "REQ-RECOVERY-002", "REQ-RECOVERY-003"):
        fault = [v for v in by_id[rid].verifications if v.kind == "fault"][0]
        assert fault.status == "PASS", (rid, fault.detail)


@pytest.mark.integration
def test_report_traces_requirement_to_test_to_evidence_to_result(all_results):
    report = build_report(all_results)
    json.dumps(report, allow_nan=False)
    entry = next(r for r in report["requirements"] if r["id"] == "REQ-EST-004")
    assert entry["result"] in ("PASS", "FAIL")
    assert entry["verification"][0]["evidence"]  # fault scenario evidence path
    assert entry["tests"]                         # supporting tests listed
    md = to_markdown(report)
    for r in load_registry().requirements:
        assert r.id in md
    assert "NOT_RUN" in md and "requires a physical" in md.lower() or "hardware" in md.lower()
    assert report["summary"]["pass"] + report["summary"]["fail"] + report["summary"]["partial"] + report["summary"]["not_run"] == len(report["requirements"])


def test_markdown_table_cells_escape_pipes():
    report = {
        "summary": {"pass": 1, "fail": 0, "partial": 0, "not_run": 0, "known_open": 0, "gate_ok": True, "gate_problems": []},
        "requirements": [{
            "id": "REQ-X-001", "title": "t", "level": "MUST", "threshold": "|v| <= 3", "result": "PASS",
            "known_open": None, "rationale": "r", "measurement": "m", "legacy_ids": [], "verification": [], "tests": [],
        }],
    }
    row = [ln for ln in to_markdown(report).splitlines() if ln.startswith("| REQ-X-001")][0]
    assert row.count("|") - row.count("\\|") == 6  # 5 columns -> 6 real separators
