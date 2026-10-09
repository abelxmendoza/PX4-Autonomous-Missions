"""The website's search page must show exactly the committed evidence and ground truth."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def test_search_page_data_is_byte_identical_to_the_evidence_and_the_world_truth():
    web = ROOT / "web" / "replay" / "data"
    assert (web / "search_report.json").read_bytes() == \
        (ROOT / "evidence" / "search" / "single_drone_sweep_report.json").read_bytes()
    assert (web / "search_targets.json").read_bytes() == \
        (ROOT / "worlds" / "search_field_targets.json").read_bytes()
