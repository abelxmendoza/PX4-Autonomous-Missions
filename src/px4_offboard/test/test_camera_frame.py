"""Unit tests for the Gazebo->ROS camera pixel-format mapping."""

import pytest

from px4_offboard.camera_frame import UnsupportedPixelFormat, gz_pixel_format_to_ros_fields


def test_rgb_int8_maps_to_rgb8_with_3_bytes_per_pixel():
    fields = gz_pixel_format_to_ros_fields("RGB_INT8", width=640)
    assert fields.encoding == "rgb8"
    assert fields.step == 640 * 3
    assert fields.is_bigendian is False


def test_mono8_maps_to_1_byte_per_pixel():
    fields = gz_pixel_format_to_ros_fields("L_INT8", width=320)
    assert fields.encoding == "mono8"
    assert fields.step == 320


def test_rgba_int8_maps_to_4_bytes_per_pixel():
    fields = gz_pixel_format_to_ros_fields("RGBA_INT8", width=100)
    assert fields.encoding == "rgba8"
    assert fields.step == 400


def test_unsupported_pixel_format_raises_instead_of_guessing():
    with pytest.raises(UnsupportedPixelFormat):
        gz_pixel_format_to_ros_fields("BAYER_RGGB8", width=640)


def test_sensor_stamp_prefers_the_gz_header_and_rejects_unset_values():
    from px4_offboard.camera_frame import sensor_stamp

    assert sensor_stamp(12, 345_000_000) == (12, 345_000_000)
    assert sensor_stamp(0, 100) == (0, 100)  # sim just started: still a real stamp
    assert sensor_stamp(0, 0) is None  # unset header
    assert sensor_stamp(-1, 5) is None
