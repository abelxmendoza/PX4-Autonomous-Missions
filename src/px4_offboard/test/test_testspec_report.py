"""Runner + verification report against the mock adapter (SimVehicle on a FakeClock).

These runs are mock executions. Nothing here starts PX4, Gazebo or MAVLink.
"""
from __future__ import annotations

import csv
import hashlib
import itertools
import json
import threading
from pathlib import Path

import pytest
import yaml

from px4_offboard.testspec import cli
from px4_offboard.testspec.adapters import MockAdapter, build_adapter
from px4_offboard.testspec.runner import SpecRun, new_run_id
from px4_offboard.validation.registry import load_registry

ROOT = Path(__file__).resolve().parents[3]
EXAMPLE = ROOT / "config" / "test_specs" / "basic_mission.yaml"
REGISTRY = load_registry()


def _write(tmp_path: Path, mutate=None) -> Path:
    doc = yaml.safe_load(EXAMPLE.read_text())
    if mutate:
        mutate(doc)
    path = tmp_path / "spec.yaml"
    path.write_text(yaml.safe_dump(doc))
    return path


def _run(tmp_path: Path, spec: Path | None = None, **kw) -> tuple[dict, SpecRun]:
    run = SpecRun(spec or EXAMPLE, tmp_path / "runs", registry=REGISTRY, **kw)
    return run.execute(), run


@pytest.fixture(scope="module")
def passed(tmp_path_factory):
    return _run(tmp_path_factory.mktemp("pass"))


def test_basic_mission_passes_on_the_mock_and_says_it_is_a_mock(passed):
    report, run = passed
    assert report["verdict"] == "PASSED"
    env = report["environment"]
    assert env["kind"] == "mock" and env["vehicle_class"] == "SimVehicle"
    assert env["px4_in_loop"] is False and env["px4_sitl"] is False
    assert env["hil"] is False and env["physical_flight"] is False
    assert "Not PX4, not SITL, not HIL" in env["label"]
    assert "not a PX4 SITL, HIL or flight result" in report["evidence_notice"]
    assert report["kind"] == "test_spec_verification_report" and report["schema"] == 1


def test_report_traces_state_history_steps_and_assertions(passed):
    report, _ = passed
    assert [h["state"] for h in report["state_history"]] == [
        "CREATED", "VALIDATED", "INITIALIZING", "READY", "RUNNING", "EVALUATING", "PASSED"]
    assert report["mission_complete"] is True
    assert all(s["status"] == "COMPLETED" for s in report["steps"])
    assert report["summary"]["assertions_passed"] == report["summary"]["assertions_total"] == 5
    for a in report["assertions"]:
        assert a["telemetry_ref"]["file"] == "telemetry.csv"
        assert a["telemetry_ref"]["sample_range"] == [0, report["telemetry"]["samples"] - 1]


def test_report_carries_config_commit_and_timestamps(passed):
    report, _ = passed
    assert report["spec"]["name"] == "basic_mission"
    assert report["spec"]["sha256"] == hashlib.sha256(EXAMPLE.read_bytes()).hexdigest()
    assert report["spec"]["config"]["vehicle"]["adapter"] == "mock"
    commit = report["source"]["git_commit"]
    assert commit == "unknown" or len(commit) == 40
    ts = report["timestamps"]
    assert ts["created_utc"] <= ts["running_utc"] <= ts["finished_utc"]
    assert ts["wall_duration_s"] > 0


def test_requirement_rows_come_from_the_registry_and_do_not_claim_a_registry_verdict(passed):
    report, _ = passed
    [row] = report["requirements"]
    req = REGISTRY.get("REQ-GEOFENCE-007")
    assert row["id"] == req.id and row["title"] == req.title and row["level"] == req.level
    assert row["result"] == "PASS"
    assert row["assertions"] == ["stays_in_keep_in"]
    assert row["evidence_scope"] == "mock"
    assert row["changes_registry_verdict"] is False


def test_telemetry_file_is_what_the_report_hashes_and_counts(passed):
    report, run = passed
    path = run.run_dir / "telemetry.csv"
    assert hashlib.sha256(path.read_bytes()).hexdigest() == report["telemetry"]["sha256"]
    rows = list(csv.DictReader(path.open()))
    assert len(rows) == report["telemetry"]["samples"]
    assert [int(r["index"]) for r in rows] == list(range(len(rows)))
    assert rows[0]["step"] == "-1" and rows[0]["cmd_vn_mps"] == ""  # no setpoint was sent before the first sample
    landed = report["assertions"][3]
    last = rows[landed["telemetry_ref"]["deciding_sample"]]
    assert float(last["north_m"]) == pytest.approx(landed["observed"]["north_m"])


def test_structured_log_is_json_lines_for_this_run(passed):
    report, run = passed
    lines = [json.loads(l) for l in (run.run_dir / "run.log.jsonl").read_text().splitlines()]
    assert {l["run_id"] for l in lines} == {report["run_id"]}
    events = [l["event"] for l in lines]
    assert events[0] == "run_created" and events[-1] == "run_finished"
    assert events.count("step_start") == events.count("step_end") == 8
    assert events.count("assertion") == 5
    assert json.loads((run.run_dir / "report.json").read_text()) == report


def test_a_step_timeout_is_failed_and_later_steps_are_not_run(tmp_path):
    spec = _write(tmp_path, lambda d: d["mission"][2].update(timeout_s=1.0))
    report, _ = _run(tmp_path, spec)
    assert report["verdict"] == "FAILED"
    assert "mission incomplete: step leg_north TIMEOUT" in report["verdict_reason"]
    statuses = [s["status"] for s in report["steps"]]
    assert statuses == ["COMPLETED", "COMPLETED", "TIMEOUT"] + ["NOT_RUN"] * 5
    assert report["mission_complete"] is False
    # The keep-in held over the partial flight, but an incomplete mission cannot verify the requirement.
    assert report["assertions"][0]["status"] == "PASS"
    assert report["requirements"][0]["result"] == "INCOMPLETE"


def test_a_failed_assertion_is_failed_and_fails_its_requirement(tmp_path):
    def shrink(d):
        d["assertions"][0]["polygon_ned"] = [[-5, -5], [8, -5], [8, 13], [-5, 13]]
    report, _ = _run(tmp_path, _write(tmp_path, shrink))
    assert report["verdict"] == "FAILED"
    assert "stays_in_keep_in" in report["verdict_reason"]
    fence = report["assertions"][0]
    assert fence["status"] == "FAIL" and fence["observed"]["first_violation"]["reason"] == "outside_horizontal"
    assert report["requirements"][0]["result"] == "FAIL"


def test_an_invalid_spec_is_error_with_nothing_executed(tmp_path):
    spec = _write(tmp_path, lambda d: d["mission"].insert(1, {"action": "barrel_roll"}))
    report, run = _run(tmp_path, spec)
    assert report["verdict"] == "ERROR"
    assert [h["state"] for h in report["state_history"]] == ["CREATED", "ERROR"]
    assert "unknown action 'barrel_roll'" in report["spec"]["validation_error"]
    assert report["environment"] is None
    assert "nothing was executed" in report["evidence_notice"]
    assert report["telemetry"]["samples"] == 0 and not (run.run_dir / "telemetry.csv").exists()
    assert report["steps"] == [] and report["assertions"] == []


def test_a_missing_spec_file_is_error(tmp_path):
    report, _ = _run(tmp_path, tmp_path / "nope.yaml")
    assert report["verdict"] == "ERROR" and "cannot read spec" in report["verdict_reason"]


class _SilentAdapter(MockAdapter):
    """Connects but never produces a state: models a target that sends no telemetry."""

    def tick(self) -> None:
        self.clock.advance(1.0 / self.rate_hz)


class _BrokenAdapter(MockAdapter):
    def start(self) -> None:
        raise ConnectionRefusedError("udp 14540 refused")


@pytest.mark.parametrize(
    "factory, fragment",
    [
        (lambda v: _SilentAdapter(**v.params), "no telemetry within 2.0 s"),
        (lambda v: _BrokenAdapter(**v.params), "ConnectionRefusedError"),
    ],
)
def test_infrastructure_failures_are_error_not_failed(tmp_path, factory, fragment):
    report, _ = _run(tmp_path, adapter_factory=factory)
    assert report["verdict"] == "ERROR"
    assert fragment in report["verdict_reason"]
    assert [h["state"] for h in report["state_history"]][-2:] == ["INITIALIZING", "ERROR"]
    assert report["assertions"] == []
    assert report["requirements"][0]["result"] == "NOT_EVALUATED"


def test_a_step_timeout_with_a_dead_link_is_error_not_failed(tmp_path):
    from px4_offboard.vehicle.interface import LinkHealth

    class _DeadLink(MockAdapter):
        def __init__(self, **kw):
            super().__init__(**kw)
            self.vehicle.link_health = lambda: LinkHealth(True, False, 0.0, None)

    spec = _write(tmp_path, lambda d: d["mission"][2].update(timeout_s=1.0))
    report, _ = _run(tmp_path, spec, adapter_factory=lambda v: _DeadLink(**v.params))
    assert report["verdict"] == "ERROR"
    assert "link not alive when step leg_north timed out" in report["verdict_reason"]


def test_cancellation_mid_mission_is_aborted_and_never_passed(tmp_path):
    cancel = threading.Event()

    class _CancelAfter(MockAdapter):
        ticks = 0

        def tick(self) -> None:
            super().tick()
            _CancelAfter.ticks += 1
            if _CancelAfter.ticks == 150:
                cancel.set()

    report, _ = _run(tmp_path, adapter_factory=lambda v: _CancelAfter(**v.params), cancel=cancel)
    assert report["verdict"] == "ABORTED"
    assert [h["state"] for h in report["state_history"]][-2:] == ["RUNNING", "ABORTED"]
    assert report["telemetry"]["samples"] > 0  # what was recorded before the cancel is kept
    assert report["assertions"] == []
    assert report["requirements"][0]["result"] == "NOT_EVALUATED"


def test_exceeding_the_wall_clock_budget_is_error(tmp_path):
    ticks = itertools.count()
    report, _ = _run(tmp_path, wall_clock=lambda: next(ticks) * 1.0)  # 1 s of "wall time" per read
    assert report["verdict"] == "ERROR"
    assert "wall-clock budget" in report["verdict_reason"]


def test_run_ids_are_unique_and_a_run_directory_is_never_reused(tmp_path):
    ids = {new_run_id() for _ in range(200)}
    assert len(ids) == 200
    SpecRun(EXAMPLE, tmp_path, registry=REGISTRY, run_id="fixed")
    with pytest.raises(FileExistsError):
        SpecRun(EXAMPLE, tmp_path, registry=REGISTRY, run_id="fixed")


def test_build_adapter_returns_the_mock_for_mock_specs():
    from px4_offboard.testspec.schema import VehicleSpec

    adapter = build_adapter(VehicleSpec("mock", {"rate_hz": 20.0, "velocity_tau_s": 0.3}))
    assert isinstance(adapter, MockAdapter) and adapter.environment["kind"] == "mock"


def test_cli_exit_codes(tmp_path, capsys):
    assert cli.main(["--spec", str(EXAMPLE), "--out", str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert "[MOCK]" in out and "PASSED" in out and "registry verdict unchanged" in out
    failing = _write(tmp_path, lambda d: d["assertions"][1].update(limit_m=4.0))
    assert cli.main(["--spec", str(failing), "--out", str(tmp_path)]) == 1
    invalid = _write(tmp_path, lambda d: d.update(faults=[{"type": "comm_disconnect"}]))
    assert cli.main(["--spec", str(invalid), "--out", str(tmp_path)]) == 2
    assert cli.main(["--spec", str(invalid), "--validate-only"]) == 2
    assert "unsupported fault" in capsys.readouterr().err
    assert cli.main(["--spec", str(EXAMPLE), "--validate-only"]) == 0
