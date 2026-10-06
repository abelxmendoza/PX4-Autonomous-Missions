"""MAVLink v2 frame codec and incremental stream parser.

Implemented from the public wire-format description (not a pymavlink wrapper)
so the failure handling is explicit and testable: byte-at-a-time delivery,
resynchronisation after garbage, CRC failures, truncated frames, unknown
message ids and signed frames. ``test_comms_frame.py`` cross-checks the
encoder and the ``CRC_EXTRA`` table byte-for-byte against pymavlink.

Wire layout (v2): ``FD len incompat compat seq sysid compid msgid[3]
payload[len] crc[2]``. The CRC is CRC-16/MCRF4XX over everything after ``FD``
plus a per-message ``crc_extra`` byte. Trailing zero payload bytes are
truncated by the sender (at least one byte is kept).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

STX_V2 = 0xFD
HEADER_LEN = 10
CRC_LEN = 2

MSG_HEARTBEAT = 0
MSG_SYS_STATUS = 1
MSG_ATTITUDE = 30
MSG_LOCAL_POSITION_NED = 32
MSG_GLOBAL_POSITION_INT = 33
MSG_COMMAND_LONG = 76
MSG_COMMAND_ACK = 77
MSG_SET_POSITION_TARGET_LOCAL_NED = 84
MSG_STATUSTEXT = 253

# crc_extra per message id (from the MAVLink common dialect). Verified against
# pymavlink in test_comms_frame.py; an id missing here cannot be CRC-checked
# and is reported as UNKNOWN_MSG rather than trusted.
CRC_EXTRA: dict[int, int] = {
    MSG_HEARTBEAT: 50,
    MSG_SYS_STATUS: 124,
    MSG_ATTITUDE: 39,
    MSG_LOCAL_POSITION_NED: 185,
    MSG_GLOBAL_POSITION_INT: 104,
    MSG_COMMAND_LONG: 152,
    MSG_COMMAND_ACK: 143,
    MSG_SET_POSITION_TARGET_LOCAL_NED: 143,
    MSG_STATUSTEXT: 83,
}


def crc_x25(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/MCRF4XX, the checksum MAVLink calls X.25."""
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Frame:
    msgid: int
    seq: int
    sysid: int
    compid: int
    payload: bytes


def encode_frame(
    msgid: int, payload: bytes, seq: int, sysid: int = 1, compid: int = 1
) -> bytes:
    if msgid not in CRC_EXTRA:
        raise ValueError(f"unknown message id {msgid}: no crc_extra to protect it")
    if len(payload) > 255:
        raise ValueError("payload longer than 255 bytes")
    body = bytes(payload).rstrip(b"\x00") or b"\x00"
    header = bytes(
        [len(body), 0, 0, seq & 0xFF, sysid & 0xFF, compid & 0xFF]
    ) + msgid.to_bytes(3, "little")
    crc = crc_x25(header + body + bytes([CRC_EXTRA[msgid]]))
    return bytes([STX_V2]) + header + body + crc.to_bytes(2, "little")


class ParseEvent:
    """One thing the parser observed: a good frame or a classified error."""

    FRAME = "frame"
    BAD_CRC = "bad_crc"
    UNKNOWN_MSG = "unknown_msg"
    UNSUPPORTED = "unsupported"

    def __init__(self, kind: str, frame: Frame | None = None) -> None:
        self.kind = kind
        self.frame = frame

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"ParseEvent({self.kind!r}, {self.frame!r})"


@dataclass
class ParseStats:
    frames: int = 0
    bad_crc: int = 0
    unknown_msg: int = 0
    unsupported: int = 0
    garbage_bytes: int = 0


class FrameParser:
    """Incremental parser: ``feed`` any chunking of the byte stream."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.stats = ParseStats()

    @property
    def pending_bytes(self) -> int:
        return len(self._buf)

    def reset(self) -> None:
        """Drop a partial frame (e.g. after a read timeout or reconnect)."""
        self._buf.clear()

    def feed(self, data: bytes) -> list[ParseEvent]:
        self._buf += data
        events: list[ParseEvent] = []
        buf = self._buf
        while True:
            start = buf.find(STX_V2)
            if start < 0:
                self.stats.garbage_bytes += len(buf)
                buf.clear()
                break
            if start:
                self.stats.garbage_bytes += start
                del buf[:start]
            if len(buf) < HEADER_LEN:
                break
            length, incompat = buf[1], buf[2]
            if incompat:  # signed or unknown incompat flags: do not guess a length
                self.stats.unsupported += 1
                events.append(ParseEvent(ParseEvent.UNSUPPORTED))
                del buf[:1]
                continue
            total = HEADER_LEN + length + CRC_LEN
            if len(buf) < total:
                break
            msgid = int.from_bytes(buf[7:10], "little")
            extra = CRC_EXTRA.get(msgid)
            if extra is None:
                self.stats.unknown_msg += 1
                events.append(ParseEvent(ParseEvent.UNKNOWN_MSG))
                del buf[:total]
                continue
            received = int.from_bytes(buf[total - CRC_LEN : total], "little")
            if crc_x25(bytes(buf[1 : total - CRC_LEN]) + bytes([extra])) != received:
                self.stats.bad_crc += 1
                events.append(ParseEvent(ParseEvent.BAD_CRC))
                del buf[:1]  # resync: a real frame may start inside this one
                continue
            frame = Frame(
                msgid=msgid,
                seq=buf[4],
                sysid=buf[5],
                compid=buf[6],
                payload=bytes(buf[HEADER_LEN : HEADER_LEN + length]),
            )
            del buf[:total]
            self.stats.frames += 1
            events.append(ParseEvent(ParseEvent.FRAME, frame))
        return events
