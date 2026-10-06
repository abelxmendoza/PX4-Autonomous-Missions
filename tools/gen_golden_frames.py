#!/usr/bin/env python3
"""Write cpp/test/golden_frames.txt: MAVLink frame vectors shared by the Python and C++ codecs.

Valid frames are generated with the Python codec, which test_comms_frame.py
verifies byte-for-byte against pymavlink. Stream cases pin how malformed input
is classified. Run with --check to fail if the committed file is stale.

Line formats (fields separated by '|'):
  frame|name|msgid|seq|sysid|compid|payload_hex|wire_hex
  stream|name|input_hex|expected events, comma separated (frame:<seq>, bad_crc, unknown_msg, unsupported)
"""
import argparse
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "px4_offboard"))
from px4_offboard.comms import messages as m  # noqa: E402
from px4_offboard.comms.mavlink_frame import FrameParser, ParseEvent, encode_frame  # noqa: E402

OUT = ROOT / "cpp" / "test" / "golden_frames.txt"


def vectors() -> list[str]:
    frames = [
        ("heartbeat", 0, m.pack_heartbeat(base_mode=0x81, custom_mode=7), 0, 1, 1),
        ("attitude", 30, m.pack_attitude(1234, 0.1, -0.2, 3.0), 5, 42, 7),
        ("local_position", 32, m.pack_local_position_ned(10, 1.0, 2.0, -3.0, 0.5, 0.0, -0.25), 255, 1, 1),
        ("arm_command", 76, m.pack_command_long(400, 1, 1, 1.0), 9, 255, 190),
        ("velocity_setpoint", 84, m.pack_velocity_setpoint(1, 1, 1.0, -2.0, 0.5), 17, 255, 190),
        ("all_zero_payload", 0, bytes(9), 3, 1, 1),
    ]
    lines = []
    wires = {}
    for name, msgid, payload, seq, sysid, compid in frames:
        wire = encode_frame(msgid, payload, seq=seq, sysid=sysid, compid=compid)
        wires[name] = wire
        lines.append(f"frame|{name}|{msgid}|{seq}|{sysid}|{compid}|{payload.hex()}|{wire.hex()}")

    good = lambda seq: encode_frame(0, m.pack_heartbeat(), seq=seq)  # noqa: E731
    bad = bytearray(good(1))
    bad[-1] ^= 0xFF
    unknown = bytes([0xFD, 1, 0, 0, 0, 1, 1, 0xFF, 0xFF, 0x00, 0x01, 0x00, 0x00])
    signed = bytearray(good(0))
    signed[2] = 0x01
    streams = {
        "two_frames_back_to_back": good(1) + good(2),
        "garbage_prefix": b"\x00\x13\x37" + good(3),
        "bad_crc_then_good": bytes(bad) + good(2),
        "truncated_only": good(4)[:-3],
        "unknown_msgid": unknown + good(5),
        "signed_flag_unsupported": bytes(signed),
    }
    for name, data in streams.items():
        events = []
        for e in FrameParser().feed(data):
            events.append(f"frame:{e.frame.seq}" if e.kind == ParseEvent.FRAME else e.kind)
        lines.append(f"stream|{name}|{data.hex()}|{','.join(events)}")
    return lines


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true", help="exit 1 if the committed file differs")
    args = parser.parse_args()
    text = "\n".join(vectors()) + "\n"
    if args.check:
        if not OUT.exists() or OUT.read_text() != text:
            print(f"{OUT} is stale: run python tools/gen_golden_frames.py", file=sys.stderr)
            return 1
        return 0
    OUT.write_text(text)
    print(f"wrote {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
