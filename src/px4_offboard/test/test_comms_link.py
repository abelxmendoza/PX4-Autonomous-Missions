"""MavlinkLink: timeouts, malformed input, link-loss detection, reconnect."""
from __future__ import annotations

import struct

import pytest

from px4_offboard.comms.clock import FakeClock
from px4_offboard.comms.link import (
    LinkDown,
    LinkError,
    LinkState,
    MavlinkLink,
    ReconnectPolicy,
)
from px4_offboard.comms.mavlink_frame import MSG_HEARTBEAT, FrameParser, encode_frame
from px4_offboard.comms.serial_transport import MockSerialDevice, SerialConfig

HB = struct.pack("<IBBBBB", 0, 12, 3, 81, 4, 3)


@pytest.fixture
def clock():
    return FakeClock()


@pytest.fixture
def dev(clock):
    return MockSerialDevice(SerialConfig(device="mock0", baud=115200), clock)


def make_link(dev, clock, **kw):
    kw.setdefault("policy", ReconnectPolicy(initial_delay_s=0.1, max_delay_s=0.4, multiplier=2.0, max_attempts=4))
    return MavlinkLink(dev, clock, **kw)


def hb(seq):
    return encode_frame(MSG_HEARTBEAT, HB, seq=seq)


def test_connect_opens_the_device(dev, clock):
    link = make_link(dev, clock)
    link.connect()
    assert link.state is LinkState.CONNECTED and dev.is_open


def test_connect_retries_with_exponential_backoff_then_gives_up(dev, clock):
    dev.unplug()
    link = make_link(dev, clock)
    with pytest.raises(LinkError):
        link.connect()
    assert dev.open_attempts == 4
    # sleeps between the 4 attempts: 0.1 + 0.2 + 0.4 (capped); none after the last
    assert clock.now() == pytest.approx(0.7)
    assert link.state is LinkState.DISCONNECTED


def test_connect_succeeds_when_the_device_appears_mid_backoff(dev, clock):
    dev.unplug()
    link = make_link(dev, clock)
    # make the device appear after the second failed attempt
    original_open = dev.open

    def open_and_replug():
        if dev.open_attempts >= 2:
            dev.plug()
        original_open()

    dev.open = open_and_replug
    link.connect()
    assert link.state is LinkState.CONNECTED
    assert dev.open_attempts == 3


def test_poll_returns_complete_frames_even_when_delivered_in_fragments(dev, clock):
    dev.max_chunk = 3
    link = make_link(dev, clock)
    link.connect()
    dev.inject_rx(hb(1) + hb(2))
    frames = []
    for _ in range(20):
        frames += link.poll(0.05)
    assert [f.seq for f in frames] == [1, 2]
    assert link.stats.rx_frames == 2


def test_poll_timeout_returns_nothing_and_is_counted(dev, clock):
    link = make_link(dev, clock)
    link.connect()
    assert link.poll(0.2) == []
    assert link.stats.read_timeouts == 1


def test_corrupted_frame_between_good_ones_is_dropped_and_counted(dev, clock):
    link = make_link(dev, clock)
    link.connect()
    bad = bytearray(hb(2))
    bad[-1] ^= 0x55
    dev.inject_rx(hb(1) + bytes(bad) + hb(3))
    frames = link.poll(0.1)
    assert [f.seq for f in frames] == [1, 3]
    assert link.parser_stats.bad_crc == 1


def test_a_frame_that_never_completes_is_discarded_after_the_frame_timeout(dev, clock):
    link = make_link(dev, clock, frame_timeout_s=0.5)
    link.connect()
    dev.inject_rx(hb(1)[:6])
    link.poll(0.05)
    assert link.parser_pending_bytes == 6
    clock.advance(0.6)
    link.poll(0.05)
    assert link.parser_pending_bytes == 0
    assert link.stats.partial_frame_timeouts == 1
    dev.inject_rx(hb(2))
    assert [f.seq for f in link.poll(0.1)] == [2]  # the stream recovers


def test_link_is_alive_only_while_valid_frames_keep_arriving(dev, clock):
    link = make_link(dev, clock, link_timeout_s=1.0)
    link.connect()
    assert not link.is_alive()  # nothing heard yet
    dev.inject_rx(hb(1))
    link.poll(0.05)
    assert link.is_alive()
    clock.advance(1.5)
    assert not link.is_alive()


def test_garbage_does_not_keep_the_link_alive(dev, clock):
    link = make_link(dev, clock, link_timeout_s=1.0)
    link.connect()
    dev.inject_rx(hb(1))
    link.poll(0.05)
    clock.advance(0.8)
    dev.inject_rx(b"\x00\x01\x02\x03" * 8)
    link.poll(0.05)
    clock.advance(0.5)
    assert not link.is_alive()


def test_send_encodes_with_wrapping_sequence_numbers(dev, clock):
    link = make_link(dev, clock)
    link.connect()
    for _ in range(258):
        link.send(MSG_HEARTBEAT, HB)
    parser = FrameParser()
    seqs = [e.frame.seq for e in parser.feed(bytes(dev.tx_log)) if e.frame]
    assert seqs[:3] == [0, 1, 2] and seqs[255:] == [255, 0, 1]


def test_short_writes_are_completed(dev, clock):
    dev.max_write = 4
    link = make_link(dev, clock)
    link.connect()
    link.send(MSG_HEARTBEAT, HB)
    assert bytes(dev.tx_log) == encode_frame(MSG_HEARTBEAT, HB, seq=0, compid=link.compid)


def test_a_write_that_makes_no_progress_is_a_link_failure(dev, clock):
    dev.max_write = 0
    link = make_link(dev, clock)
    link.connect()
    with pytest.raises(LinkDown):
        link.send(MSG_HEARTBEAT, HB)
    assert link.state is LinkState.DISCONNECTED


def test_send_while_disconnected_raises_link_down(dev, clock):
    link = make_link(dev, clock)
    with pytest.raises(LinkDown):
        link.send(MSG_HEARTBEAT, HB)


def test_unplug_is_detected_on_poll_and_reconnect_is_timed(dev, clock):
    link = make_link(dev, clock)
    link.connect()
    dev.inject_rx(hb(1))
    link.poll(0.05)
    dev.unplug()
    assert link.poll(0.05) == []
    assert link.state is LinkState.DISCONNECTED
    assert link.stats.disconnects == 1
    t_down = clock.now()
    clock.advance(0.3)  # the cable stays out for a while
    dev.plug()
    assert link.reconnect() is True
    assert link.state is LinkState.CONNECTED
    assert link.stats.reconnects == 1
    assert link.stats.last_reconnect_s == pytest.approx(clock.now() - t_down)
    dev.inject_rx(hb(2))
    assert [f.seq for f in link.poll(0.1)] == [2]


def test_reconnect_discards_a_half_received_frame(dev, clock):
    link = make_link(dev, clock)
    link.connect()
    dev.inject_rx(hb(1)[:7])
    link.poll(0.05)
    dev.unplug()
    link.poll(0.05)
    dev.plug()
    assert link.reconnect()
    dev.inject_rx(hb(9))
    assert [f.seq for f in link.poll(0.1)] == [9]


def test_reconnect_gives_up_when_the_device_stays_gone(dev, clock):
    link = make_link(dev, clock)
    link.connect()
    dev.unplug()
    link.poll(0.05)
    assert link.reconnect() is False
    assert link.state is LinkState.DISCONNECTED


def test_one_poll_drains_everything_already_received_not_just_one_chunk(dev, clock):
    # Regression: a poll that read a single chunk fell behind a 40 frame/s
    # stream and the backlog grew without bound (state went stale by seconds).
    link = make_link(dev, clock)
    link.connect()
    for seq in range(30):
        dev.inject_rx(hb(seq))  # 30 separate deliveries, like UDP datagrams
    assert len(link.poll(0.05)) == 30
    assert link.stats.read_timeouts == 0
