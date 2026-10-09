"""Path trail drawn in the Gazebo window during a flight."""
import os

import pytest

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
marker_pb2 = pytest.importorskip("gz.msgs10.marker_pb2")

from px4_offboard.gz_trail import TrailPoints, build_trail_marker  # noqa: E402


def test_marker_is_a_gui_only_pink_line_strip_in_gazebo_axes():
    m = build_trail_marker([(1.0, 2.0, -8.0), (3.0, 4.0, -8.0)])
    assert m.type == marker_pb2.Marker.LINE_STRIP
    assert m.visibility == marker_pb2.Marker.GUI   # never in a sensor's image
    assert m.action == marker_pb2.Marker.ADD_MODIFY
    # NED (north, east, down) -> Gazebo ENU (x=east, y=north, z=up)
    assert [(p.x, p.y, p.z) for p in m.point] == [(2.0, 1.0, 8.0), (4.0, 3.0, 8.0)]
    assert m.material.diffuse.r == pytest.approx(1.0) and m.material.diffuse.b > 0.5


def test_points_are_kept_only_after_the_drone_has_moved_far_enough():
    t = TrailPoints(min_step_m=0.5)
    for k in range(20):                      # 0.00 .. 0.95 m in 5 cm steps
        t.add((k * 0.05, 0.0, -8.0))
    assert [p[0] for p in t.points] == [0.0, 0.5]   # 0.95 is only 0.45 m past the last point
    t.add((0.95, 0.0, -8.0))
    assert len(t.points) == 2                # standing still adds nothing
    t.add((1.0, 0.0, -8.0))
    assert [p[0] for p in t.points] == [0.0, 0.5, 1.0]


def test_marker_needs_at_least_two_points():
    assert build_trail_marker([(0, 0, -1)]) is None
