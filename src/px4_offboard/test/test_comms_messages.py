"""Message payload codecs, byte-checked against pymavlink where available."""
from __future__ import annotations

import math

import pytest

from px4_offboard.comms import messages as m
from px4_offboard.comms.mavlink_frame import FrameParser, encode_frame


def test_heartbeat_roundtrip_and_armed_flag():
    payload = m.pack_heartbeat(custom_mode=0x04000000, base_mode=0x81)
    hb = m.unpack_heartbeat(payload)
    assert hb.armed is True and hb.custom_mode == 0x04000000
    assert m.unpack_heartbeat(m.pack_heartbeat(base_mode=0x01)).armed is False


def test_attitude_roundtrip_in_degrees():
    att = m.unpack_attitude(m.pack_attitude(1000, math.radians(5), math.radians(-3), math.radians(90)))
    assert (att.roll_deg, att.pitch_deg, att.yaw_deg) == pytest.approx((5, -3, 90), abs=1e-4)


def test_local_position_roundtrip():
    pos = m.unpack_local_position_ned(m.pack_local_position_ned(2000, 1.0, 2.0, -3.0, 0.5, 0.0, -0.25))
    assert pos.time_boot_ms == 2000
    assert (pos.x, pos.y, pos.z, pos.vx, pos.vz) == pytest.approx((1.0, 2.0, -3.0, 0.5, -0.25))


def test_short_payloads_from_zero_truncation_are_padded_not_rejected():
    # An all-zero tail is truncated by the sender; the decoder must restore it.
    full = m.pack_local_position_ned(5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    pos = m.unpack_local_position_ned(full.rstrip(b"\x00") or b"\x00")
    assert pos.time_boot_ms == 5 and pos.vz == 0.0


def test_payload_longer_than_the_message_definition_is_rejected():
    with pytest.raises(ValueError):
        m.unpack_attitude(bytes(29))


def test_command_ack_roundtrip():
    ack = m.unpack_command_ack(m.pack_command_ack(m.MAV_CMD_COMPONENT_ARM_DISARM, m.MAV_RESULT_ACCEPTED))
    assert ack.command == 400 and ack.accepted


def test_arm_command_carries_the_arm_flag_in_param1():
    cmd = m.unpack_command_long(m.pack_command_long(m.MAV_CMD_COMPONENT_ARM_DISARM, 1, 1, 1.0))
    assert cmd.command == 400 and cmd.params[0] == 1.0 and cmd.target_system == 1


def test_velocity_setpoint_masks_everything_but_velocity():
    sp = m.unpack_set_position_target(m.pack_velocity_setpoint(1, 1, 1.0, -2.0, 0.5))
    assert (sp.vx, sp.vy, sp.vz) == pytest.approx((1.0, -2.0, 0.5))
    assert sp.type_mask == m.VELOCITY_ONLY_MASK
    assert sp.coordinate_frame == m.MAV_FRAME_LOCAL_NED


# --- byte-for-byte agreement with pymavlink --------------------------------

common = pytest.importorskip("pymavlink.dialects.v20.common")


def _reference(build):
    mav = common.MAVLink(None, srcSystem=1, srcComponent=1)
    msg = build(mav)
    return bytes(msg.pack(mav))


def test_heartbeat_payload_matches_pymavlink():
    ref = _reference(lambda mav: mav.heartbeat_encode(2, 12, 0x81, 7, 4, 3))
    ours = encode_frame(0, m.pack_heartbeat(mav_type=2, autopilot=12, base_mode=0x81, custom_mode=7), seq=0)
    assert ours == ref


def test_attitude_payload_matches_pymavlink():
    ref = _reference(lambda mav: mav.attitude_encode(10, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6))
    ours = encode_frame(30, m.pack_attitude(10, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6), seq=0)
    assert ours == ref


def test_local_position_payload_matches_pymavlink():
    ref = _reference(lambda mav: mav.local_position_ned_encode(10, 1, 2, 3, 4, 5, 6))
    ours = encode_frame(32, m.pack_local_position_ned(10, 1, 2, 3, 4, 5, 6), seq=0)
    assert ours == ref


def test_command_long_payload_matches_pymavlink():
    ref = _reference(lambda mav: mav.command_long_encode(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0))
    ours = encode_frame(76, m.pack_command_long(400, 1, 1, 1.0), seq=0)
    assert ours == ref


def test_velocity_setpoint_payload_matches_pymavlink():
    ref = _reference(
        lambda mav: mav.set_position_target_local_ned_encode(
            0, 1, 1, m.MAV_FRAME_LOCAL_NED, m.VELOCITY_ONLY_MASK, 0, 0, 0, 1.0, -2.0, 0.5, 0, 0, 0, 0, 0
        )
    )
    ours = encode_frame(84, m.pack_velocity_setpoint(1, 1, 1.0, -2.0, 0.5), seq=0)
    assert ours == ref


def test_pymavlink_parser_accepts_frames_we_encode():
    mav = common.MAVLink(None)
    wire = encode_frame(30, m.pack_attitude(10, 0.1, 0.2, 0.3), seq=5)
    msg = mav.parse_char(wire)
    assert msg is not None and msg.get_msgId() == 30 and msg.get_header().seq == 5
    assert FrameParser().feed(wire)[0].frame.seq == 5
