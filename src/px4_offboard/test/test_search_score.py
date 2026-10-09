"""Scoring a search report against ground truth."""
import pytest

from px4_offboard.search_score import score

TRUTH = {"targets": [{"id": 3, "north": 10.0, "east": 0.0}, {"id": 9, "north": 20.0, "east": 5.0},
                     {"id": 25, "north": 30.0, "east": -5.0}]}


def test_perfect_report_passes():
    report = {"targets": [{"id": t["id"], "north": t["north"], "east": t["east"]} for t in TRUTH["targets"]]}
    s = score(report, TRUTH)
    assert s.missed == [] and s.false_ids == [] and s.max_error_m == 0.0 and s.passed(1.0)


def test_errors_misses_and_false_ids_are_all_reported():
    report = {"targets": [{"id": 3, "north": 10.0, "east": 1.5}, {"id": 9, "north": 23.0, "east": 9.0},
                          {"id": 44, "north": 1.0, "east": 1.0}]}
    s = score(report, TRUTH)
    assert s.found == {3: pytest.approx(1.5), 9: pytest.approx(5.0)}
    assert s.missed == [25] and s.false_ids == [44]
    assert s.max_error_m == pytest.approx(5.0) and s.mean_error_m == pytest.approx(3.25)
    assert not s.passed(10.0)  # a miss or a false id fails regardless of accuracy


def test_accuracy_threshold_decides_when_everything_is_found():
    report = {"targets": [{"id": t["id"], "north": t["north"] + 1.2, "east": t["east"]} for t in TRUTH["targets"]]}
    s = score(report, TRUTH)
    assert s.passed(1.5) and not s.passed(1.0)


def test_an_empty_report_misses_everything():
    s = score({"targets": []}, TRUTH)
    assert s.missed == [3, 9, 25] and s.max_error_m is None and not s.passed(100.0)
