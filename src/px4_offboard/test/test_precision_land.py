"""Precision-land guidance: the tracktor-beam search / approach / descend law."""

from __future__ import annotations

import math

import numpy as np
import pytest

from px4_offboard.ekf_fusion import quat_to_rotation_matrix
from px4_offboard.precision_land import (
    OPTICAL_TO_BODY,
    CameraTag,
    CommandMode,
    LandState,
    PrecisionLand,
    PrecisionLandParams,
    VehicleSnapshot,
    generate_search_waypoints,
    offboard_setpoint,
    tag_to_ned,
)


def _optical_from_mount(vehicle: VehicleSnapshot, north: float, east: float, down: float = 0.0) -> CameraTag:
    """Marker in the camera optical frame, from the mount in search_geolocate.py.

    Image right is body +y, image down is body -x, and the optical axis is body +z.
    This does not use OPTICAL_TO_BODY.
    """
    rotation = quat_to_rotation_matrix(np.array(vehicle.attitude_wxyz, dtype=float))
    delta = np.array([north, east, down]) - np.array(vehicle.position_ned, dtype=float)
    body = rotation.T @ delta - np.array((0.0, 0.0, 0.05))
    return CameraTag((float(body[1]), float(-body[0]), float(body[2])), time_s=vehicle.time_s)


def _vehicle(position, velocity=(0.0, 0.0, 0.0), landed=False, time_s=1.0, yaw=0.0) -> VehicleSnapshot:
    half = yaw / 2.0
    return VehicleSnapshot(
        position_ned=tuple(position),
        velocity_ned=tuple(velocity),
        attitude_wxyz=(math.cos(half), 0.0, 0.0, math.sin(half)),
        landed=landed,
        time_s=time_s,
    )


def _tag_under(vehicle: VehicleSnapshot, north: float, east: float, down: float = 0.0) -> CameraTag:
    """Optical-frame pose of a level marker, inverse of tag_to_ned for identity marker rotation."""
    world = np.array([north, east, down], dtype=float)
    attitude = quat_to_rotation_matrix(np.array(vehicle.attitude_wxyz, dtype=float))
    offset = np.array(PrecisionLandParams().camera_offset_body_ned, dtype=float)
    in_body = attitude.T @ (world - np.array(vehicle.position_ned, dtype=float))
    optical = OPTICAL_TO_BODY.T @ (in_body - offset)
    return CameraTag(tuple(float(v) for v in optical), time_s=vehicle.time_s)


def test_downward_camera_maps_optical_axes_into_body_ned():
    vehicle = _vehicle((0.0, 0.0, -5.0))
    # 2 m straight out of a downward lens, camera 5 cm below the body origin.
    straight_down = CameraTag((0.0, 0.0, 2.0), time_s=1.0)
    position, yaw = tag_to_ned(straight_down, vehicle)
    assert position == pytest.approx((0.0, 0.0, -5.0 + 0.05 + 2.0))
    # Marker x (image right) is body +y. With identity attitude that yaw is +pi/2.
    assert yaw == pytest.approx(math.pi / 2.0)

    right = CameraTag((1.0, 0.0, 2.0), time_s=1.0)
    assert tag_to_ned(right, vehicle)[0][1] == pytest.approx(1.0)
    aft = CameraTag((0.0, 1.0, 2.0), time_s=1.0)
    assert tag_to_ned(aft, vehicle)[0][0] == pytest.approx(-1.0)


def test_search_spiral_starts_on_the_vehicle_and_descends():
    params = PrecisionLandParams()
    waypoints = generate_search_waypoints(5.0, -3.0, -5.0, params)
    assert waypoints[0] == pytest.approx((5.0, -3.0, -5.0))
    angle = 2.0 * math.pi / params.search_points_per_layer
    radius = params.search_max_radius_m / params.search_points_per_layer
    assert waypoints[1] == pytest.approx((5.0 + radius * math.cos(angle), -3.0 + radius * math.sin(angle), -5.0))
    # 4 laps from z=-5 down toward the -1 m floor, 17 points out and 17 back.
    assert len(waypoints) == 4 * 2 * (params.search_points_per_layer + 1)
    assert waypoints[-1][2] > waypoints[0][2]
    # Already near the floor: C++ truncates a negative layer count to zero, then forces one lap.
    assert len(generate_search_waypoints(0.0, 0.0, -0.4, params)) == 2 * (params.search_points_per_layer + 1)


def test_stale_detection_does_not_abort_the_search():
    lander = PrecisionLand(PrecisionLandParams(target_timeout_s=5.0))
    start = _vehicle((1.0, 2.0, -4.0), time_s=1.0)
    lander.activate(start.position_ned)
    lander.observe_tag(_tag_under(start, 1.0, 2.0), start)
    later = _vehicle((1.0, 2.0, -4.0), time_s=10.0)
    command = lander.step(later)
    assert lander.state == LandState.SEARCH
    assert command.mode == CommandMode.POSITION
    assert not lander.failed


def test_fresh_marker_approaches_then_velocity_pulls_back_over_it():
    lander = PrecisionLand()
    over_pad = _vehicle((0.0, 0.0, -4.0), time_s=2.0)
    lander.activate(over_pad.position_ned)
    lander.observe_tag(_tag_under(over_pad, 0.0, 0.0), over_pad)
    # Already on the approach point, so this tick commits to the descent.
    first = lander.step(over_pad)
    assert lander.state == LandState.DESCEND
    assert first.mode == CommandMode.VELOCITY
    assert first.velocity_ned[2] == pytest.approx(0.6)
    assert first.velocity_ned[0] == pytest.approx(0.0, abs=1e-6)

    offset = _vehicle((1.0, -0.5, -3.5), time_s=2.1)
    lander.observe_tag(_tag_under(offset, 0.0, 0.0), offset)
    command = lander.step(offset)
    # P gain 1.7: north error +1 m commands -1.7 m/s, east error -0.5 m commands +0.85 m/s.
    assert command.velocity_ned[0] == pytest.approx(-1.7)
    assert command.velocity_ned[1] == pytest.approx(0.85)
    assert command.velocity_ned[2] == pytest.approx(0.6)


def test_losing_the_marker_during_descent_holds_position():
    lander = PrecisionLand(PrecisionLandParams(target_timeout_s=1.0, vel_i_gain=0.5))
    hover = _vehicle((0.0, 0.0, -4.0), time_s=1.0)
    lander.activate(hover.position_ned)
    lander.observe_tag(_tag_under(hover, 0.0, 0.0), hover)
    assert lander.step(hover).mode == CommandMode.VELOCITY
    # Build a real horizontal command, then lose the marker.
    offset = _vehicle((1.0, 0.0, -3.0), time_s=1.2)
    lander.observe_tag(_tag_under(offset, 0.0, 0.0), offset)
    moving = lander.step(offset)
    assert moving.mode == CommandMode.VELOCITY
    assert moving.velocity_ned[0] != 0.0

    lost = _vehicle((0.2, 0.0, -3.0), velocity=moving.velocity_ned, time_s=3.0)
    command = lander.step(lost)
    assert lander.failed
    assert command.mode == CommandMode.HOLD
    assert command.position_ned == pytest.approx((0.2, 0.0, -3.0))
    fields = offboard_setpoint(command)
    assert fields["use_position"] is True
    assert all(math.isnan(v) for v in fields["velocity_ned"])
    # The hold keeps streaming. A fresh marker does not resume the old velocity.
    again = _vehicle((0.2, 0.0, -3.0), time_s=3.2)
    assert lander.observe_tag(_tag_under(again, 5.0, 5.0), again) is None
    held = lander.step(again)
    assert held.mode == CommandMode.HOLD
    assert all(math.isnan(v) for v in offboard_setpoint(held)["velocity_ned"])
    # A new enable starts from a cleared integrator, not the descent that was in progress.
    lander.activate(again.position_ned)
    lander.observe_tag(_tag_under(again, 0.2, 0.0), again)
    restarted = lander.step(again)
    assert restarted.mode == CommandMode.VELOCITY
    assert restarted.velocity_ned[0] == pytest.approx(0.0, abs=1e-6)


def test_a_fix_that_jumps_is_ignored_until_the_target_times_out():
    lander = PrecisionLand(PrecisionLandParams(target_timeout_s=1.0, max_fix_jump_m=2.0))
    over = _vehicle((0.0, 0.0, -4.0), time_s=1.0)
    lander.activate(over.position_ned)
    lander.observe_tag(_tag_under(over, 0.0, 0.0), over)
    assert lander.step(over).state == LandState.DESCEND

    glitch = _vehicle((0.4, 0.0, -3.8), time_s=1.2)
    assert lander.observe_tag(_tag_under(glitch, 8.0, 0.0), glitch) is None
    command = lander.step(glitch)
    assert command.mode == CommandMode.VELOCITY
    assert command.tag_ned == pytest.approx((0.0, 0.0, 0.0), abs=0.05)
    # Still steering toward the last good fix (north of the vehicle is negative vx? vehicle at 0.4, tag at 0).
    assert command.velocity_ned[0] < 0.0
    assert command.tag_age_s == pytest.approx(0.2)

    # The good fix returns inside the jump gate and refreshes the age.
    seen = _vehicle((0.4, 0.0, -3.8), time_s=1.4)
    lander.observe_tag(_tag_under(seen, 0.1, 0.0), seen)
    refreshed = lander.step(seen)
    assert refreshed.tag_age_s == pytest.approx(0.0, abs=1e-9)
    assert refreshed.mode == CommandMode.VELOCITY

    # A stream of impossible fixes does not refresh the timestamp, so the hold still trips.
    stale = _vehicle((0.4, 0.0, -3.5), time_s=2.6)
    assert lander.observe_tag(_tag_under(stale, 9.0, 1.0), stale) is None
    failed = lander.step(stale)
    assert failed.mode == CommandMode.HOLD
    assert all(math.isnan(v) for v in offboard_setpoint(failed)["velocity_ned"])


def test_marker_ned_and_velocity_follow_the_pad_at_every_yaw():
    """Optical measurement is built from the downward-camera mount description, not from
    OPTICAL_TO_BODY, so a sign error in that matrix cannot hide inside a round trip.
    """
    pad = (2.0, -1.5, 0.0)
    # Hand-checked nadir case: yaw 0, marker 2 m north, camera 5 cm below the origin.
    nadir = _vehicle((0.0, 0.0, -5.0), yaw=0.0)
    optical = (0.0, -2.0, 4.95)  # image-down is aft, so a forward marker has negative image y
    recovered, _yaw = tag_to_ned(CameraTag(optical, time_s=1.0), nadir)
    assert recovered == pytest.approx((2.0, 0.0, 0.0), abs=1e-6)

    for yaw in (0.0, math.pi / 2, math.pi, -math.pi / 2, 0.7, 2.4):
        vehicle = _vehicle((0.0, 0.0, -4.0), yaw=yaw, time_s=2.0)
        measurement = _optical_from_mount(vehicle, *pad)
        position, _ = tag_to_ned(measurement, vehicle)
        assert position == pytest.approx(pad, abs=1e-6), yaw

        lander = PrecisionLand()
        over_pad = _vehicle(pad[:2] + (-4.0,), yaw=yaw, time_s=2.0)
        lander.activate(over_pad.position_ned)
        lander.observe_tag(_optical_from_mount(over_pad, *pad), over_pad)
        assert lander.step(over_pad).state == LandState.DESCEND

        offset = _vehicle((0.0, 0.0, -3.5), yaw=yaw, time_s=2.1)
        lander.observe_tag(_optical_from_mount(offset, *pad), offset)
        command = lander.step(offset)
        toward = (pad[0] - offset.position_ned[0], pad[1] - offset.position_ned[1])
        vel = command.velocity_ned
        assert vel[0] * toward[0] + vel[1] * toward[1] > 0.0, yaw
        # Per-axis clamp, same as tracktor-beam: a diagonal error can exceed max_velocity.
        dx = offset.position_ned[0] - pad[0]
        dy = offset.position_ned[1] - pad[1]
        assert vel[0] == pytest.approx(max(-3.0, min(3.0, -1.7 * dx)))
        assert vel[1] == pytest.approx(max(-3.0, min(3.0, -1.7 * dy)))


def test_closed_loop_descent_lands_on_the_marker():
    params = PrecisionLandParams(delta_velocity=0.3)
    lander = PrecisionLand(params)
    position = np.array([1.6, -0.8, -4.0])
    velocity = np.zeros(3)
    lander.activate(tuple(position))
    dt = 0.05
    command = None
    for step in range(1, 700):
        time_s = step * dt
        landed = bool(position[2] >= -0.02)
        vehicle = _vehicle(position, velocity, landed=landed, time_s=time_s)
        lander.observe_tag(_tag_under(vehicle, 0.0, 0.0), vehicle)
        command = lander.step(vehicle)
        if command.mode == CommandMode.POSITION:
            error = np.array(command.position_ned) - position
            velocity = np.clip(2.0 * error, -2.0, 2.0)
            position = position + velocity * dt
        elif command.mode == CommandMode.VELOCITY:
            velocity = np.array(command.velocity_ned, dtype=float)
            position = position + velocity * dt
        elif command.mode == CommandMode.DONE:
            break
    assert command is not None and command.mode == CommandMode.DONE
    assert lander.state == LandState.FINISHED
    assert math.hypot(position[0], position[1]) < 0.25
    assert position[2] == pytest.approx(0.0, abs=0.05)
