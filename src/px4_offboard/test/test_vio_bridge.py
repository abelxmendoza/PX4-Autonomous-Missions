import pytest

from px4_offboard.frame_transforms import enu_to_ned, named_enu_pose_to_ned


def test_enu_position_converts_to_px4_ned():
    assert enu_to_ned(3.0, 7.0, 2.5) == pytest.approx([7.0, 3.0, -2.5])


def test_named_model_pose_converts_gazebo_enu_to_ned():
    poses = [
        ("obstacle_1", -6.0, 10.0, 5.75),
        ("x500_lidar_2d_0", -5.7, 15.6, 7.5),
    ]
    assert named_enu_pose_to_ned(poses, "x500_lidar_2d_0") == pytest.approx(
        [15.6, -5.7, -7.5]
    )
    assert named_enu_pose_to_ned(poses, "missing") is None
