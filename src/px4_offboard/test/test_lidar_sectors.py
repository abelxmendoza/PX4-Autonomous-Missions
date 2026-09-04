import math

from px4_offboard.sensor_logic import sector_from_scan


def _scan(front: float, left: float, right: float):
    return sector_from_scan(
        [right, front, left],
        angle_min=-math.pi / 4,
        angle_step=math.pi / 4,
        range_min=0.1,
        range_max=30.0,
        trigger_m=6.0,
        front_deg=20.0,
        side_deg=90.0,
        side_trigger_m=2.0,
    )


def test_long_range_side_return_does_not_trigger_detour():
    label, mins = _scan(front=6.6, left=20.0, right=5.8)
    assert label is None
    assert mins["right"] == 5.8


def test_close_side_return_remains_an_emergency_trigger():
    label, _ = _scan(front=6.6, left=20.0, right=1.8)
    assert label == "right"


def test_forward_sector_keeps_long_detection_range():
    label, _ = _scan(front=5.8, left=20.0, right=20.0)
    assert label == "front"
