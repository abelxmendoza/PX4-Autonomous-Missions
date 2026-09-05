import math

from px4_offboard.sensor_logic import SectorEma, get_sector_bands, sector_from_scan


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


def test_sector_bands_are_cached_and_partition_indices():
    a = get_sector_bands(720, -math.pi, 2 * math.pi / 720, 35.0, 90.0)
    b = get_sector_bands(720, -math.pi, 2 * math.pi / 720, 35.0, 90.0)
    assert a is b
    assert len(a.front) + len(a.left) + len(a.right) <= 720
    assert len(a.front) > 0 and len(a.left) > 0 and len(a.right) > 0


def test_sector_ema_low_passes_spikes():
    ema = SectorEma(alpha=0.5)
    ema.update({"front": 4.0, "left": 10.0, "right": 10.0})
    smoothed = ema.update({"front": 1.0, "left": 10.0, "right": 10.0})
    assert 1.0 < smoothed["front"] < 4.0
