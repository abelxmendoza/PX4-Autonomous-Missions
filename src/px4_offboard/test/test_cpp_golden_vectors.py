"""The vectors the C++ codec is tested against must match what the Python codec produces."""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def test_committed_golden_vectors_are_current():
    result = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "gen_golden_frames.py"), "--check"],
        capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr


def test_every_valid_golden_frame_parses_with_pymavlink():
    import pytest

    common = pytest.importorskip("pymavlink.dialects.v20.common")
    for line in (ROOT / "cpp/test/golden_frames.txt").read_text().splitlines():
        fields = line.split("|")
        if fields[0] != "frame":
            continue
        msg = common.MAVLink(None).parse_char(bytes.fromhex(fields[7]))
        assert msg is not None and msg.get_msgId() == int(fields[2]), fields[1]
        assert msg.get_header().seq == int(fields[3]), fields[1]
