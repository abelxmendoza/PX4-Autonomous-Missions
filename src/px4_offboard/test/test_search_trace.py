"""Flight trace recorded during a search flight, for the browser replay."""
import json
import math

import pytest

from px4_offboard.search_geolocate import quat_from_euler
from px4_offboard.search_trace import TraceRecorder


def test_pose_frames_are_throttled_to_the_requested_rate_and_carry_yaw():
    rec = TraceRecorder(rate_hz=10.0)
    q = quat_from_euler(0.0, 0.0, math.radians(90))
    for k in range(100):  # 100 samples over 1 s
        rec.pose(k / 100, (1.0, 2.0, -8.0), q)
    frames = rec.to_dict()["frames"]
    assert len(frames) == 10
    assert frames[0]["yaw_deg"] == pytest.approx(90.0, abs=1e-6)
    assert frames[0]["d"] == -8.0


def test_sightings_and_confirmations_are_kept_in_time_order():
    rec = TraceRecorder()
    rec.sighting(2.0, 9, 20.3, 6.1)
    rec.sighting(2.1, 9, 20.4, 6.0)
    rec.confirmed(2.2, 9)
    d = rec.to_dict()
    assert [s["id"] for s in d["sightings"]] == [9, 9]
    assert d["confirmations"] == [{"t": 2.2, "id": 9}]


def test_trace_round_trips_through_json_without_truth():
    rec = TraceRecorder()
    rec.pose(0.0, (0.0, 0.0, 0.0), quat_from_euler(0, 0, 0))
    text = json.dumps(rec.to_dict(world="search_field"))
    d = json.loads(text)
    assert d["schema"] == 1 and d["world"] == "search_field"
    assert "targets" not in d and "truth" not in json.dumps(d).lower()


def test_rejects_time_going_backwards():
    rec = TraceRecorder()
    rec.pose(1.0, (0, 0, 0), quat_from_euler(0, 0, 0))
    with pytest.raises(ValueError):
        rec.pose(0.5, (0, 0, 0), quat_from_euler(0, 0, 0))
