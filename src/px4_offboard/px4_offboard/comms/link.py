"""MAVLink link supervision over any :class:`Transport`.

Owns the failure policy the layers below deliberately do not have: read
timeouts, partial-frame expiry, link-loss detection (valid frames only --
noise must not keep a dead link "alive"), and bounded exponential-backoff
reconnect with timing, so a fault scenario can report recovery time.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .clock import Clock
from .mavlink_frame import Frame, FrameParser, ParseEvent, ParseStats, encode_frame
from .serial_transport import PeerUnknown, Transport, TransportError


MAX_DRAIN_READS = 256


class LinkError(Exception):
    pass


class LinkDown(LinkError):
    """The operation needs a connected link and there is none."""


class LinkNotReady(LinkError):
    """Connected, but nobody to talk to yet (listening UDP before the first datagram)."""


class LinkState(Enum):
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"


@dataclass(frozen=True)
class ReconnectPolicy:
    initial_delay_s: float = 0.1
    max_delay_s: float = 2.0
    multiplier: float = 2.0
    max_attempts: int = 5


@dataclass
class LinkStats:
    tx_frames: int = 0
    rx_frames: int = 0
    read_timeouts: int = 0
    partial_frame_timeouts: int = 0
    disconnects: int = 0
    reconnects: int = 0
    last_reconnect_s: float | None = None


class MavlinkLink:
    def __init__(
        self,
        transport: Transport,
        clock: Clock,
        policy: ReconnectPolicy | None = None,
        link_timeout_s: float = 3.0,
        frame_timeout_s: float = 0.5,
        sysid: int = 1,
        compid: int = 191,
    ) -> None:
        self._transport = transport
        self._clock = clock
        self.policy = policy or ReconnectPolicy()
        self.link_timeout_s = link_timeout_s
        self.frame_timeout_s = frame_timeout_s
        self.sysid = sysid
        self.compid = compid
        self.state = LinkState.DISCONNECTED
        self.stats = LinkStats()
        self._parser = FrameParser()
        self._seq = 0
        self._last_frame_t: float | None = None
        self._partial_since: float | None = None
        self._down_since: float | None = None

    # --- introspection ---------------------------------------------------
    @property
    def parser_stats(self) -> ParseStats:
        return self._parser.stats

    @property
    def parser_pending_bytes(self) -> int:
        return self._parser.pending_bytes

    def is_alive(self) -> bool:
        if self.state is not LinkState.CONNECTED or self._last_frame_t is None:
            return False
        return (self._clock.now() - self._last_frame_t) <= self.link_timeout_s

    # --- connection management -------------------------------------------
    def _try_open(self) -> bool:
        try:
            self._transport.close()
            self._transport.open()
        except TransportError:
            return False
        self._parser.reset()
        self._partial_since = None
        self.state = LinkState.CONNECTED
        return True

    def _backoff_open(self) -> bool:
        delay = self.policy.initial_delay_s
        for attempt in range(self.policy.max_attempts):
            if self._try_open():
                return True
            if attempt < self.policy.max_attempts - 1:
                self._clock.sleep(delay)
                delay = min(delay * self.policy.multiplier, self.policy.max_delay_s)
        return False

    def connect(self) -> None:
        if not self._backoff_open():
            raise LinkError(
                f"could not open transport after {self.policy.max_attempts} attempts"
            )

    def reconnect(self) -> bool:
        """Re-open after a loss; records how long the link was down."""
        if self.state is LinkState.CONNECTED:
            return True
        if not self._backoff_open():
            return False
        self.stats.reconnects += 1
        if self._down_since is not None:
            self.stats.last_reconnect_s = self._clock.now() - self._down_since
        self._down_since = None
        self._last_frame_t = None  # liveness must be re-earned by a valid frame
        return True

    def disconnect(self) -> None:
        """Deliberate close: not counted as a link loss."""
        self.state = LinkState.DISCONNECTED
        self._down_since = None
        self._last_frame_t = None
        try:
            self._transport.close()
        except TransportError:
            pass

    def _mark_down(self) -> None:
        if self.state is LinkState.CONNECTED:
            self.state = LinkState.DISCONNECTED
            self.stats.disconnects += 1
            self._down_since = self._clock.now()
        try:
            self._transport.close()
        except TransportError:
            pass

    # --- data path --------------------------------------------------------
    def send(self, msgid: int, payload: bytes) -> None:
        if self.state is not LinkState.CONNECTED:
            raise LinkDown("send on a disconnected link")
        wire = encode_frame(msgid, payload, seq=self._seq, sysid=self.sysid, compid=self.compid)
        view = memoryview(wire)
        try:
            while view:
                written = self._transport.write(bytes(view))
                if written <= 0:
                    raise TransportError("write made no progress")
                view = view[written:]
        except PeerUnknown as exc:
            raise LinkNotReady(str(exc)) from exc
        except TransportError as exc:
            self._mark_down()
            raise LinkDown(str(exc)) from exc
        self._seq = (self._seq + 1) & 0xFF
        self.stats.tx_frames += 1

    def poll(self, timeout_s: float = 0.1) -> list[Frame]:
        if self.state is not LinkState.CONNECTED:
            return []
        chunks: list[bytes] = []
        try:
            first = self._transport.read(4096, timeout_s)
            chunks.append(first)
            # Drain what is already buffered: a datagram/chunk per read would
            # otherwise let a busy stream outrun the poll rate.
            while first and len(chunks) < MAX_DRAIN_READS:
                more = self._transport.read(4096, 0.0)
                if not more:
                    break
                chunks.append(more)
        except TransportError:
            self._mark_down()
            return []
        now = self._clock.now()
        if not first:
            self.stats.read_timeouts += 1
        frames: list[Frame] = []
        for event in self._parser.feed(b"".join(chunks)):
            if event.kind == ParseEvent.FRAME:
                frames.append(event.frame)
        if frames:
            self.stats.rx_frames += len(frames)
            self._last_frame_t = now
        self._expire_partial_frame(now)
        return frames

    def _expire_partial_frame(self, now: float) -> None:
        if self._parser.pending_bytes == 0:
            self._partial_since = None
            return
        if self._partial_since is None:
            self._partial_since = now
        elif now - self._partial_since > self.frame_timeout_s:
            self._parser.reset()
            self._partial_since = None
            self.stats.partial_frame_timeouts += 1
