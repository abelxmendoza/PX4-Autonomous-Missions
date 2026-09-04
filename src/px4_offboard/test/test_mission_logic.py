import math

import pytest

from px4_offboard.mission_logic import (
    Fence,
    Obstacle,
    blocking_height,
    circle_target,
    detect_obstacle,
    distance_3d,
    yaw_toward,
)


OBSTACLES = (
    Obstacle(east=0.0, north=5.0, size_east=2.0, size_north=2.0, height=4.0),
    Obstacle(east=3.0, north=5.0, size_east=2.0, size_north=2.0, height=7.0),
)


def test_fence_contains_inclusive_boundaries():
    fence = Fence(-5.0, 55.0, -23.0, 17.0, 12.0)

    assert fence.contains([-5.0, 17.0, -12.0])
    assert not fence.contains([-5.01, 0.0, -5.0])
    assert not fence.contains([0.0, 17.01, -5.0])
    assert not fence.contains([0.0, 0.0, -12.06])


def test_fence_clamps_horizontal_position_and_altitude():
    fence = Fence(-5.0, 55.0, -23.0, 17.0, 12.0)

    target, changed = fence.clamp([100.0, -100.0, -20.0], margin=1.0)

    assert target == [54.0, -22.0, -12.0]
    assert changed


def test_fence_does_not_mutate_input():
    fence = Fence(-5.0, 55.0, -23.0, 17.0, 12.0)
    original = [1.0, 2.0, -3.0]

    result, changed = fence.clamp(original, margin=1.0)

    assert result == original
    assert result is not original
    assert not changed


def test_invalid_fence_is_rejected():
    with pytest.raises(ValueError):
        Fence(1.0, -1.0, -1.0, 1.0, 5.0)


def test_distance_and_yaw_use_ned_axes():
    assert distance_3d([0.0, 0.0, 0.0], [3.0, 4.0, 12.0]) == 13.0
    assert yaw_toward([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]) == 0.0
    assert yaw_toward([0.0, 0.0, 0.0], [0.0, 1.0, 0.0]) == pytest.approx(math.pi / 2)
    assert math.isnan(yaw_toward([0.0, 0.0, 0.0], [0.05, 0.05, 0.0]))


def test_circle_target_at_quarter_period():
    target = circle_target(elapsed_s=5.0, radius_m=8.0, period_s=20.0, altitude_m=5.0)

    assert target == pytest.approx([0.0, 8.0, -5.0], abs=1e-12)


def test_circle_rejects_nonpositive_period():
    with pytest.raises(ValueError):
        circle_target(0.0, 1.0, 0.0, 1.0)


def test_detects_obstacle_ahead_in_ned_frame():
    result = detect_obstacle(
        position=[0.0, 0.0, -3.0],
        target=[10.0, 0.0, -3.0],
        obstacles=OBSTACLES,
        detection_margin_m=3.0,
        front_angle_deg=35.0,
        side_angle_deg=70.0,
    )

    assert result == "front"


def test_ignores_obstacle_when_vehicle_is_above_it():
    result = detect_obstacle(
        position=[0.0, 0.0, -5.0],
        target=[10.0, 0.0, -5.0],
        obstacles=[OBSTACLES[0]],
        detection_margin_m=3.0,
        front_angle_deg=35.0,
        side_angle_deg=70.0,
    )

    assert result is None


def test_sidestep_mode_detects_obstacle_even_when_above_it():
    result = detect_obstacle(
        position=[0.0, 0.0, -5.0],
        target=[10.0, 0.0, -5.0],
        obstacles=[OBSTACLES[0]],
        detection_margin_m=3.0,
        front_angle_deg=35.0,
        side_angle_deg=70.0,
        detect_when_above=True,
    )

    assert result == "front"


def test_blocking_height_uses_tallest_obstacle_in_corridor():
    height = blocking_height(
        position=[0.0, 0.0, -3.0],
        target=[10.0, 0.0, -3.0],
        obstacles=OBSTACLES,
        detection_margin_m=3.0,
        side_angle_deg=70.0,
    )

    assert height == 7.0
