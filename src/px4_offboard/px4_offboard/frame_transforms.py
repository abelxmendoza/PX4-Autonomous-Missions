"""Pure coordinate-frame transforms used by simulator bridges."""


def enu_to_ned(east: float, north: float, up: float) -> list[float]:
    return [north, east, -up]
