"""Precision landing onto an ArUco marker.

Same guidance law as ARK Electronics / Rainmaker tracktor-beam
(https://github.com/ARK-Electronics/tracktor-beam): spiral search until the
marker is in view, fly over it at the current altitude, then descend on a
horizontal PI velocity loop while PX4's land detector finishes the touchdown.

This module is the controller only. It does not register a custom PX4 flight
mode (that needs px4_ros2_interface_lib and a matching autopilot). The ROS
node streams the resulting setpoints as ordinary OFFBOARD trajectory commands.

NED throughout. The marker measurement is in the camera optical frame
(x right, y down, z out of the lens), which a downward camera maps into body
FRD by (x, y, z) -> (-y, x, z). That is the same mount as
``search_geolocate.py`` and tracktor-beam's ``getTagWorld``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

import numpy as np

from .ekf_fusion import quat_to_rotation_matrix

# Optical (x right, y down, z forward) -> body FRD (x forward, y right, z down).
OPTICAL_TO_BODY = np.array(
    [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
    dtype=float,
)


class LandState(Enum):
    IDLE = "idle"
    SEARCH = "search"
    APPROACH = "approach"
    DESCEND = "descend"
    FINISHED = "finished"


class CommandMode(Enum):
    NONE = "none"
    POSITION = "position"
    VELOCITY = "velocity"
    HOLD = "hold"
    DONE = "done"


@dataclass(frozen=True)
class PrecisionLandParams:
    """Defaults match tracktor-beam ``precision_land/cfg/params.yaml``."""

    descent_vel: float = 0.6
    vel_p_gain: float = 1.7
    vel_i_gain: float = 0.0
    max_velocity: float = 3.0
    target_timeout_s: float = 5.0
    delta_position: float = 0.2
    delta_velocity: float = 0.2
    search_max_radius_m: float = 2.0
    search_layer_spacing_m: float = 0.5
    search_points_per_layer: int = 16
    # NED down of the lowest search lap. -1 m is 1 m above a z=0 pad.
    search_floor_z: float = -1.0
    # x500_search_cam sits 5 cm below the body origin. Tracktor-beam's
    # x500_mono_cam_down uses (0, 0, -0.1) on that airframe.
    camera_offset_body_ned: tuple[float, float, float] = (0.0, 0.0, 0.05)
    # A new fix farther than this from the last accepted one is dropped.
    # The pad does not move; a multi-metre jump is a bad solve, not a new target.
    max_fix_jump_m: float = 2.0
    # Not in tracktor-beam (None keeps its behaviour). When set, the approach setpoint is a
    # carrot this many m/s x ``approach_lookahead_s`` ahead instead of the marker itself, so
    # PX4 does not fly a long approach at cruise speed and tilt the marker out of a
    # downward camera's view.
    approach_max_speed_mps: float | None = None
    approach_lookahead_s: float = 1.0


@dataclass(frozen=True)
class CameraTag:
    """Marker pose in the camera optical frame."""

    position_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    time_s: float = 0.0


@dataclass(frozen=True)
class VehicleSnapshot:
    position_ned: tuple[float, float, float]
    velocity_ned: tuple[float, float, float]
    attitude_wxyz: tuple[float, float, float, float]
    landed: bool
    time_s: float


@dataclass(frozen=True)
class LandCommand:
    state: LandState
    mode: CommandMode
    position_ned: tuple[float, float, float] | None = None
    velocity_ned: tuple[float, float, float] | None = None
    yaw: float | None = None
    tag_ned: tuple[float, float, float] | None = None
    tag_age_s: float | None = None


@dataclass
class _WorldTag:
    position: tuple[float, float, float] | None = None
    yaw: float = 0.0
    time_s: float = 0.0
    seen: bool = False


def tag_to_ned(
    tag: CameraTag,
    vehicle: VehicleSnapshot,
    camera_offset_body_ned: tuple[float, float, float] = (0.0, 0.0, 0.05),
) -> tuple[tuple[float, float, float], float]:
    """Marker origin in the local NED frame, and the marker yaw."""
    attitude = quat_to_rotation_matrix(np.asarray(vehicle.attitude_wxyz, dtype=float))
    ox, oy, oz, ow = tag.orientation_xyzw
    tag_rotation = quat_to_rotation_matrix(np.array([ow, ox, oy, oz], dtype=float))
    in_body = np.asarray(camera_offset_body_ned, dtype=float) + OPTICAL_TO_BODY @ np.asarray(
        tag.position_xyz, dtype=float
    )
    world = np.asarray(vehicle.position_ned, dtype=float) + attitude @ in_body
    world_rotation = attitude @ OPTICAL_TO_BODY @ tag_rotation
    yaw = math.atan2(float(world_rotation[1, 0]), float(world_rotation[0, 0]))
    return (float(world[0]), float(world[1]), float(world[2])), yaw


def generate_search_waypoints(
    center_north: float,
    center_east: float,
    current_z: float,
    params: PrecisionLandParams,
) -> list[tuple[float, float, float]]:
    """Out-and-back spiral, centered on the vehicle, descending between laps.

    Tracktor-beam hardcodes the spiral at the local origin because its demo
    starts the mode while already hovering over the pad. Centering it on the
    vehicle is the same pattern once the mission has flown somewhere else.
    Layer count uses truncating division, matching the C++ ``int`` cast.
    """
    raw_layers = int((params.search_floor_z - current_z) / params.search_layer_spacing_m)
    layers = int(raw_layers / 2)
    if layers < 1:
        layers = 1
    points = params.search_points_per_layer
    if points < 1:
        raise ValueError("search_points_per_layer must be positive")
    radius_step = params.search_max_radius_m / points
    z = current_z
    waypoints: list[tuple[float, float, float]] = []
    for _layer in range(layers):
        lap: list[tuple[float, float, float]] = []
        radius = 0.0
        for point in range(points + 1):
            angle = 2.0 * math.pi * point / points
            lap.append(
                (
                    center_north + radius * math.cos(angle),
                    center_east + radius * math.sin(angle),
                    z,
                )
            )
            radius += radius_step
        waypoints.extend(lap)
        z += params.search_layer_spacing_m
        waypoints.extend((p[0], p[1], z) for p in reversed(lap))
        z += params.search_layer_spacing_m
    return waypoints


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def offboard_setpoint(command: LandCommand) -> dict | None:
    """Trajectory fields for one guidance command.

    Position and hold commands set every velocity component to NaN. PX4 keeps
    the last finite velocity setpoint if those fields stay filled, so a lost
    marker must not leave the descent rate or the horizontal PI output running.
    """
    nan = float("nan")
    if command.mode in (CommandMode.POSITION, CommandMode.HOLD):
        if command.position_ned is None:
            return None
        return {
            "use_position": True,
            "position_ned": tuple(float(v) for v in command.position_ned),
            "velocity_ned": (nan, nan, nan),
            "yaw": nan,
        }
    if command.mode in (CommandMode.VELOCITY, CommandMode.DONE):
        if command.velocity_ned is None:
            return None
        yaw = float(command.yaw) if command.yaw is not None else nan
        return {
            "use_position": False,
            "position_ned": (nan, nan, nan),
            "velocity_ned": tuple(float(v) for v in command.velocity_ned),
            "yaw": yaw,
        }
    return None


class PrecisionLand:
    def __init__(self, params: PrecisionLandParams | None = None):
        self.params = params if params is not None else PrecisionLandParams()
        self._state = LandState.IDLE
        self._active = False
        self._failed = False
        self.failure_reason = ""
        self.last_observation_reject = ""
        self._hold: tuple[float, float, float] | None = None
        self._tag = _WorldTag()
        self._approach_z = 0.0
        self._waypoints: list[tuple[float, float, float]] = []
        self._index = 0
        self._ix = 0.0
        self._iy = 0.0

    @property
    def state(self) -> LandState:
        return self._state

    @property
    def active(self) -> bool:
        return self._active

    @property
    def failed(self) -> bool:
        return self._failed

    def activate(self, position_ned: tuple[float, float, float]) -> None:
        self._waypoints = generate_search_waypoints(
            position_ned[0], position_ned[1], position_ned[2], self.params
        )
        self._index = 0
        self._tag = _WorldTag()
        self._ix = 0.0
        self._iy = 0.0
        self._approach_z = position_ned[2]
        self._hold = None
        self._failed = False
        self.failure_reason = ""
        self.last_observation_reject = ""
        self._active = True
        self._state = LandState.SEARCH

    def deactivate(self) -> None:
        self._active = False
        self._failed = False
        self._hold = None
        self._tag = _WorldTag()
        self._state = LandState.IDLE

    def observe_tag(self, tag: CameraTag, vehicle: VehicleSnapshot) -> tuple[float, float, float] | None:
        """Store a camera-frame detection as a NED target. Ignored until activate()."""
        if not self._active or self._failed:
            return None
        position, yaw = tag_to_ned(tag, vehicle, self.params.camera_offset_body_ned)
        if self._tag.position is not None:
            jump = math.hypot(position[0] - self._tag.position[0], position[1] - self._tag.position[1])
            if jump > self.params.max_fix_jump_m:
                self.last_observation_reject = "marker_outlier_rejected"
                return None
        self.last_observation_reject = ""
        self._tag = _WorldTag(position=position, yaw=yaw, time_s=tag.time_s, seen=True)
        return position

    def step(self, vehicle: VehicleSnapshot) -> LandCommand:
        if not self._active:
            return self._emit(LandCommand(LandState.IDLE, CommandMode.NONE), vehicle)
        if self._state == LandState.IDLE:
            if self._failed and self._hold is not None:
                return self._emit(
                    LandCommand(LandState.IDLE, CommandMode.HOLD, position_ned=self._hold), vehicle
                )
            return self._emit(LandCommand(LandState.IDLE, CommandMode.NONE), vehicle)

        lost = self._target_lost(vehicle.time_s)
        if self._state == LandState.SEARCH:
            if self._tag.seen and not lost:
                self._approach_z = vehicle.position_ned[2]
                self._state = LandState.APPROACH
            else:
                return self._emit(self._search(vehicle), vehicle)

        if self._state == LandState.APPROACH:
            if lost or self._tag.position is None:
                return self._emit(self._fail(vehicle), vehicle)
            target = (self._tag.position[0], self._tag.position[1], self._approach_z)
            if not self._reached(vehicle, target):
                return self._emit(self._position(self._approach_carrot(vehicle, target)), vehicle)
            self._ix = 0.0
            self._iy = 0.0
            self._state = LandState.DESCEND

        if self._state == LandState.DESCEND:
            if lost or self._tag.position is None:
                return self._emit(self._fail(vehicle), vehicle)
            if vehicle.landed:
                self._state = LandState.FINISHED
            else:
                vx, vy = self._velocity_xy(vehicle)
                return self._emit(
                    LandCommand(
                        LandState.DESCEND,
                        CommandMode.VELOCITY,
                        velocity_ned=(vx, vy, self.params.descent_vel),
                        yaw=self._tag.yaw,
                        tag_ned=self._tag.position,
                    ),
                    vehicle,
                )

        if self._state == LandState.FINISHED:
            return self._emit(
                LandCommand(
                    LandState.FINISHED,
                    CommandMode.DONE,
                    velocity_ned=(0.0, 0.0, 0.0),
                    yaw=self._tag.yaw,
                    tag_ned=self._tag.position,
                ),
                vehicle,
            )
        return self._emit(LandCommand(self._state, CommandMode.NONE), vehicle)

    def _emit(self, command: LandCommand, vehicle: VehicleSnapshot) -> LandCommand:
        age = None if not self._tag.seen else vehicle.time_s - self._tag.time_s
        return LandCommand(
            state=command.state,
            mode=command.mode,
            position_ned=command.position_ned,
            velocity_ned=command.velocity_ned,
            yaw=command.yaw,
            tag_ned=command.tag_ned,
            tag_age_s=age,
        )

    def _search(self, vehicle: VehicleSnapshot) -> LandCommand:
        waypoint = self._waypoints[self._index]
        command = self._position(waypoint)
        if self._reached(vehicle, waypoint):
            self._index = (self._index + 1) % len(self._waypoints)
        return command

    def _position(self, target: tuple[float, float, float]) -> LandCommand:
        return LandCommand(
            self._state,
            CommandMode.POSITION,
            position_ned=target,
            yaw=self._tag.yaw if self._tag.seen else None,
            tag_ned=self._tag.position,
        )

    def _approach_carrot(self, vehicle: VehicleSnapshot, target: tuple[float, float, float]) -> tuple[float, float, float]:
        if self.params.approach_max_speed_mps is None:
            return target
        dn = target[0] - vehicle.position_ned[0]
        de = target[1] - vehicle.position_ned[1]
        dist = math.hypot(dn, de)
        reach = self.params.approach_max_speed_mps * self.params.approach_lookahead_s
        if dist <= reach:
            return target
        k = reach / dist
        return (vehicle.position_ned[0] + dn * k, vehicle.position_ned[1] + de * k, target[2])

    def _fail(self, vehicle: VehicleSnapshot) -> LandCommand:
        previous = self._state
        if previous is LandState.DESCEND:
            self.failure_reason = "marker_lost_during_descent"
        elif previous is LandState.APPROACH and self._tag.seen:
            self.failure_reason = "marker_stale"
        else:
            self.failure_reason = "marker_never_acquired"
        self._state = LandState.IDLE
        self._failed = True
        self._hold = vehicle.position_ned
        self._ix = 0.0
        self._iy = 0.0
        return LandCommand(LandState.IDLE, CommandMode.HOLD, position_ned=self._hold)

    def _target_lost(self, now_s: float) -> bool:
        if not self._tag.seen or self._tag.position is None:
            return True
        return (now_s - self._tag.time_s) > self.params.target_timeout_s

    def _reached(self, vehicle: VehicleSnapshot, target: tuple[float, float, float]) -> bool:
        delta_pos = math.dist(vehicle.position_ned, target)
        speed = math.hypot(*vehicle.velocity_ned)
        return delta_pos < self.params.delta_position and speed < self.params.delta_velocity

    def _velocity_xy(self, vehicle: VehicleSnapshot) -> tuple[float, float]:
        """Horizontal PI on position error. The integrator matches tracktor-beam:
        it accumulates raw metres each tick (not metres*seconds) and is clamped
        to ``max_velocity`` before the I gain. Default ``vel_i_gain`` is 0.
        """
        assert self._tag.position is not None
        dx = vehicle.position_ned[0] - self._tag.position[0]
        dy = vehicle.position_ned[1] - self._tag.position[1]
        limit = self.params.max_velocity
        self._ix = _clamp(self._ix + dx, limit)
        self._iy = _clamp(self._iy + dy, limit)
        vx = _clamp(-(self.params.vel_p_gain * dx + self.params.vel_i_gain * self._ix), limit)
        vy = _clamp(-(self.params.vel_p_gain * dy + self.params.vel_i_gain * self._iy), limit)
        return vx, vy
