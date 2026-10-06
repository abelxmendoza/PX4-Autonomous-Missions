"""MAVLink v2 frame codec: encode/parse, malformed-input handling, and
agreement with pymavlink (the reference implementation) when it is installed."""
from __future__ import annotations

import random
import struct

import pytest

from px4_offboard.comms.mavlink_frame import (
    CRC_EXTRA,
    MSG_HEARTBEAT,
    FrameParser,
    ParseEvent,
    crc_x25,
    encode_frame,
)

HEARTBEAT_PAYLOAD = struct.pack("<IBBBBB", 0, 12, 3, 81, 4, 3)  # custom_mode, type, autopilot, base_mode, system_status, mavlink_version


def _frames(parser: FrameParser, data: bytes):
    return [e for e in parser.feed(data) if e.kind == ParseEvent.FRAME]


def test_crc_x25_matches_the_published_check_value():
    # CRC-16/MCRF4XX (MAVLink's X.25) of "123456789" is 0x6F91.
    assert crc_x25(b"123456789") == 0x6F91


def test_roundtrip_single_frame():
    wire = encode_frame(MSG_HEARTBEAT, HEARTBEAT_PAYLOAD, seq=7, sysid=1, compid=1)
    events = _frames(FrameParser(), wire)
    assert len(events) == 1
    frame = events[0].frame
    assert (frame.msgid, frame.seq, frame.sysid, frame.compid) == (MSG_HEARTBEAT, 7, 1, 1)
    assert frame.payload == HEARTBEAT_PAYLOAD


def test_byte_by_byte_delivery_yields_the_same_frame():
    wire = encode_frame(MSG_HEARTBEAT, HEARTBEAT_PAYLOAD, seq=1)
    parser = FrameParser()
    out = []
    for b in wire:
        out += _frames(parser, bytes([b]))
    assert len(out) == 1 and out[0].frame.payload == HEARTBEAT_PAYLOAD


def test_trailing_zero_payload_bytes_are_truncated_on_the_wire_like_mavlink_v2():
    wire = encode_frame(MSG_HEARTBEAT, b"\x01\x00\x00\x00\x00\x00\x00\x00\x00", seq=0)
    assert wire[1] == 1  # length byte: only the first non-zero byte is sent
    frame = _frames(FrameParser(), wire)[0].frame
    assert frame.payload == b"\x01"


def test_bad_crc_is_reported_and_the_next_good_frame_still_parses():
    good = encode_frame(MSG_HEARTBEAT, HEARTBEAT_PAYLOAD, seq=2)
    bad = bytearray(encode_frame(MSG_HEARTBEAT, HEARTBEAT_PAYLOAD, seq=1))
    bad[-1] ^= 0xFF
    parser = FrameParser()
    events = parser.feed(bytes(bad) + good)
    kinds = [e.kind for e in events]
    assert ParseEvent.BAD_CRC in kinds
    frames = [e for e in events if e.kind == ParseEvent.FRAME]
    assert [f.frame.seq for f in frames] == [2]
    assert parser.stats.bad_crc == 1


def test_garbage_before_a_frame_is_skipped_and_counted():
    wire = encode_frame(MSG_HEARTBEAT, HEARTBEAT_PAYLOAD, seq=3)
    parser = FrameParser()
    events = parser.feed(b"\x00\x13\x37" + wire)
    assert [e.frame.seq for e in events if e.kind == ParseEvent.FRAME] == [3]
    assert parser.stats.garbage_bytes == 3


def test_truncated_frame_waits_for_more_bytes_and_does_not_emit():
    wire = encode_frame(MSG_HEARTBEAT, HEARTBEAT_PAYLOAD, seq=4)
    parser = FrameParser()
    assert parser.feed(wire[:-3]) == []
    assert parser.pending_bytes == len(wire) - 3
    events = _frames(parser, wire[-3:])
    assert len(events) == 1


def test_stale_partial_frame_can_be_discarded():
    wire = encode_frame(MSG_HEARTBEAT, HEARTBEAT_PAYLOAD, seq=4)
    parser = FrameParser()
    parser.feed(wire[:5])
    parser.reset()
    assert parser.pending_bytes == 0
    assert len(_frames(parser, wire)) == 1  # a clean frame after reset parses


def test_unknown_message_id_cannot_be_crc_checked_and_is_reported():
    # Header/payload valid for an msgid with no known crc_extra.
    header = bytes([0xFD, 1, 0, 0, 0, 1, 1]) + (0xFFFF).to_bytes(3, "little")
    wire = header + b"\x01" + b"\x00\x00"
    parser = FrameParser()
    events = parser.feed(wire)
    assert [e.kind for e in events] == [ParseEvent.UNKNOWN_MSG]
    assert parser.stats.unknown_msg == 1


def test_signed_frames_are_reported_unsupported_not_misparsed():
    wire = bytearray(encode_frame(MSG_HEARTBEAT, HEARTBEAT_PAYLOAD, seq=0))
    wire[2] = 0x01  # incompat flags: signed
    parser = FrameParser()
    events = parser.feed(bytes(wire))
    assert ParseEvent.UNSUPPORTED in [e.kind for e in events]
    assert parser.stats.unsupported == 1


def test_random_noise_never_raises_and_never_fabricates_a_frame():
    rng = random.Random(1234)
    parser = FrameParser()
    for _ in range(200):
        chunk = bytes(rng.randrange(256) for _ in range(rng.randrange(1, 64)))
        for e in parser.feed(chunk):
            assert e.kind != ParseEvent.FRAME or e.frame.msgid in CRC_EXTRA


def test_encode_rejects_unknown_ids_and_oversize_payloads():
    with pytest.raises(ValueError):
        encode_frame(0xFFFF, b"", seq=0)
    with pytest.raises(ValueError):
        encode_frame(MSG_HEARTBEAT, bytes(256), seq=0)


# --- agreement with the reference implementation --------------------------

mavlink2 = pytest.importorskip("pymavlink.dialects.v20.common")


def test_crc_extra_table_matches_pymavlink():
    for msgid, extra in CRC_EXTRA.items():
        assert mavlink2.mavlink_map[msgid].crc_extra == extra, msgid


def test_encoded_frames_are_byte_identical_to_pymavlink():
    mav = mavlink2.MAVLink(None, srcSystem=1, srcComponent=1)
    mav.robust_parsing = False
    msg = mav.heartbeat_encode(12, 3, 81, 0, 4, 3)
    ref = msg.pack(mav)
    ours = encode_frame(MSG_HEARTBEAT, HEARTBEAT_PAYLOAD, seq=msg.get_header().seq)
    assert ours == bytes(ref)


def test_frames_encoded_by_pymavlink_parse_with_ours():
    mav = mavlink2.MAVLink(None, srcSystem=42, srcComponent=7)
    wire = bytes(mav.attitude_encode(1234, 0.1, -0.2, 3.0, 0.0, 0.0, 0.0).pack(mav))
    frame = _frames(FrameParser(), wire)[0].frame
    assert (frame.sysid, frame.compid, frame.msgid) == (42, 7, 30)
    assert struct.unpack("<Iffffff", frame.payload.ljust(28, b"\0"))[1:4] == pytest.approx((0.1, -0.2, 3.0))
