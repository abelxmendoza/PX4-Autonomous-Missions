import pytest

from px4_offboard.frame_transforms import enu_to_ned


def test_enu_position_converts_to_px4_ned():
    assert enu_to_ned(3.0, 7.0, 2.5) == pytest.approx([7.0, 3.0, -2.5])
