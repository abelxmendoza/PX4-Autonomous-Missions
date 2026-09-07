"""Pure coordinate-frame transforms used by simulator bridges."""


def enu_to_ned(east: float, north: float, up: float) -> list[float]:
    return [north, east, -up]


def named_enu_pose_to_ned(
    poses: list[tuple[str, float, float, float]],
    model_name: str,
) -> list[float] | None:
    """Pick the named Gazebo model pose and convert world ENU metres to NED."""
    for name, east, north, up in poses:
        if name == model_name:
            return enu_to_ned(float(east), float(north), float(up))
    return None
