"""Payload codecs for the handful of MAVLink messages the validation harness
uses. Field order on the wire is MAVLink's (largest fields first), checked
byte-for-byte against pymavlink in ``test_comms_messages.py``."""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass

MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_RESULT_ACCEPTED = 0
MAV_MODE_FLAG_SAFETY_ARMED = 0x80
MAV_FRAME_LOCAL_NED = 1
# type_mask bits set = field ignored. Use velocity only (vx,vy,vz): ignore
# position (bits 0-2), acceleration (6-8), yaw (10) and yaw rate (11).
VELOCITY_ONLY_MASK = 0b0000_1101_1100_0111

_HEARTBEAT = struct.Struct("<IBBBBB")
_ATTITUDE = struct.Struct("<Iffffff")
_LOCAL_POSITION = struct.Struct("<Iffffff")
_COMMAND_LONG = struct.Struct("<7fHBBB")
_COMMAND_ACK = struct.Struct("<HB")
_SET_TARGET = struct.Struct("<I11fHBBB")


def _pad(payload: bytes, st: struct.Struct) -> bytes:
    """Restore the zero tail MAVLink v2 senders truncate; refuse overlong data."""
    if len(payload) > st.size:
        raise ValueError(f"payload of {len(payload)} bytes exceeds message size {st.size}")
    return payload.ljust(st.size, b"\x00")


@dataclass(frozen=True)
class Heartbeat:
    mav_type: int
    autopilot: int
    base_mode: int
    custom_mode: int
    system_status: int

    @property
    def armed(self) -> bool:
        return bool(self.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)


def pack_heartbeat(
    mav_type: int = 2,
    autopilot: int = 12,
    base_mode: int = 0,
    custom_mode: int = 0,
    system_status: int = 4,
) -> bytes:
    return _HEARTBEAT.pack(custom_mode, mav_type, autopilot, base_mode, system_status, 3)


def unpack_heartbeat(payload: bytes) -> Heartbeat:
    custom, mav_type, autopilot, base, status, _ = _HEARTBEAT.unpack(_pad(payload, _HEARTBEAT))
    return Heartbeat(mav_type, autopilot, base, custom, status)


@dataclass(frozen=True)
class Attitude:
    time_boot_ms: int
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


def pack_attitude(
    time_boot_ms: int,
    roll: float,
    pitch: float,
    yaw: float,
    rollspeed: float = 0.0,
    pitchspeed: float = 0.0,
    yawspeed: float = 0.0,
) -> bytes:
    return _ATTITUDE.pack(time_boot_ms, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed)


def unpack_attitude(payload: bytes) -> Attitude:
    t, roll, pitch, yaw, *_ = _ATTITUDE.unpack(_pad(payload, _ATTITUDE))
    return Attitude(t, math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


@dataclass(frozen=True)
class LocalPositionNed:
    time_boot_ms: int
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float


def pack_local_position_ned(
    time_boot_ms: int, x: float, y: float, z: float, vx: float, vy: float, vz: float
) -> bytes:
    return _LOCAL_POSITION.pack(time_boot_ms, x, y, z, vx, vy, vz)


def unpack_local_position_ned(payload: bytes) -> LocalPositionNed:
    return LocalPositionNed(*_LOCAL_POSITION.unpack(_pad(payload, _LOCAL_POSITION)))


@dataclass(frozen=True)
class CommandLong:
    command: int
    target_system: int
    target_component: int
    params: tuple[float, ...]


def pack_command_long(
    command: int, target_system: int, target_component: int, *params: float
) -> bytes:
    p = list(params) + [0.0] * (7 - len(params))
    return _COMMAND_LONG.pack(*p[:7], command, target_system, target_component, 0)


def unpack_command_long(payload: bytes) -> CommandLong:
    *params, command, tsys, tcomp, _ = _COMMAND_LONG.unpack(_pad(payload, _COMMAND_LONG))
    return CommandLong(command, tsys, tcomp, tuple(params))


@dataclass(frozen=True)
class CommandAck:
    command: int
    result: int

    @property
    def accepted(self) -> bool:
        return self.result == MAV_RESULT_ACCEPTED


def pack_command_ack(command: int, result: int) -> bytes:
    return _COMMAND_ACK.pack(command, result)


def unpack_command_ack(payload: bytes) -> CommandAck:
    # COMMAND_ACK has extension fields after command/result; only these two are used.
    return CommandAck(*_COMMAND_ACK.unpack(payload[:3].ljust(3, b"\x00")))


@dataclass(frozen=True)
class SetPositionTarget:
    type_mask: int
    coordinate_frame: int
    vx: float
    vy: float
    vz: float


def pack_velocity_setpoint(
    target_system: int, target_component: int, vx: float, vy: float, vz: float
) -> bytes:
    return _SET_TARGET.pack(
        0, 0.0, 0.0, 0.0, vx, vy, vz, 0.0, 0.0, 0.0, 0.0, 0.0,
        VELOCITY_ONLY_MASK, target_system, target_component, MAV_FRAME_LOCAL_NED,
    )


def unpack_set_position_target(payload: bytes) -> SetPositionTarget:
    f = _SET_TARGET.unpack(_pad(payload, _SET_TARGET))
    return SetPositionTarget(type_mask=f[12], coordinate_frame=f[15], vx=f[4], vy=f[5], vz=f[6])
