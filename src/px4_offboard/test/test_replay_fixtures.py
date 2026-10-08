"""The browser demo's stereo flight must stay the pinned evidence flight."""
from __future__ import annotations

import gzip
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def test_demo_stereo_flight_is_byte_identical_to_the_pinned_evidence_flight():
    # .github/workflows/unit-tests.yml runs the verifier over web/replay/data/*.csv and
    # requires every file to pass, except this one: flight K fails VO availability on
    # purpose and is checked instead by scripts/verify_evidence.py, which pins its exact
    # failing checks. That exclusion is only sound while this copy IS that flight.
    demo = (ROOT / "web/replay/data/mission_stereo_vo.csv").read_bytes()
    pinned = gzip.decompress((ROOT / "evidence/stereo_vo/flight_K.csv.gz").read_bytes())
    assert demo == pinned
