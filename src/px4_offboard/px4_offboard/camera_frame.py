"""Pure conversion from a Gazebo camera frame to ROS sensor_msgs/Image fields.

Kept dependency-free (no gz/rclpy imports) so the mapping itself is unit
testable without a running simulator, matching frame_transforms.py.
"""

from __future__ import annotations

from dataclasses import dataclass

# Gazebo gz.msgs.PixelFormatType -> (ROS sensor_msgs/Image encoding, bytes/pixel).
# Only the formats PX4's shipped camera models actually emit are mapped;
# anything else is a genuine configuration error, not a silent guess.
_GZ_TO_ROS_ENCODING: dict[str, tuple[str, int]] = {
    "RGB_INT8": ("rgb8", 3),
    "BGR_INT8": ("bgr8", 3),
    "RGBA_INT8": ("rgba8", 4),
    "BGRA_INT8": ("bgra8", 4),
    "L_INT8": ("mono8", 1),
    "L_INT16": ("mono16", 2),
}


class UnsupportedPixelFormat(ValueError):
    pass


@dataclass(frozen=True)
class RosImageFields:
    encoding: str
    step: int
    is_bigendian: bool = False


def gz_pixel_format_to_ros_fields(pixel_format_name: str, width: int) -> RosImageFields:
    """Map a gz PixelFormatType enum name to ROS Image encoding + row step."""
    try:
        encoding, bytes_per_pixel = _GZ_TO_ROS_ENCODING[pixel_format_name]
    except KeyError as exc:
        raise UnsupportedPixelFormat(
            f"no ROS encoding mapping for gz pixel format {pixel_format_name!r}"
        ) from exc
    return RosImageFields(encoding=encoding, step=width * bytes_per_pixel)
