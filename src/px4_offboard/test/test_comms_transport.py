"""Serial transport: configuration, the deterministic mock device, and a real
pyserial round trip over an OS pseudo-terminal (no hardware involved)."""
from __future__ import annotations

import os
import sys

import pytest

from px4_offboard.comms.clock import FakeClock
from px4_offboard.comms.serial_transport import (
    MockSerialDevice,
    PySerialTransport,
    SerialConfig,
    TransportClosed,
    TransportError,
)


def test_config_rejects_nonsense_values():
    with pytest.raises(ValueError):
        SerialConfig(device="", baud=57600)
    with pytest.raises(ValueError):
        SerialConfig(device="/dev/ttyACM0", baud=1234)  # not a standard rate
    with pytest.raises(ValueError):
        SerialConfig(device="/dev/ttyACM0", baud=57600, read_timeout_s=0.0)


def test_config_defaults_match_pixhawk_telem_usage():
    cfg = SerialConfig(device="/dev/ttyACM0")
    assert cfg.baud == 57600 and cfg.data_bits == 8 and cfg.parity == "N" and cfg.stop_bits == 1


@pytest.fixture
def clock():
    return FakeClock()


@pytest.fixture
def dev(clock):
    return MockSerialDevice(SerialConfig(device="mock0", baud=115200), clock)


def test_read_before_open_raises_closed(dev):
    with pytest.raises(TransportClosed):
        dev.read(16, 0.1)
    with pytest.raises(TransportClosed):
        dev.write(b"x")


def test_injected_bytes_are_returned_in_order_and_chunked(dev):
    dev.max_chunk = 4
    dev.open()
    dev.inject_rx(b"0123456789")
    assert dev.read(100, 0.1) == b"0123"
    assert dev.read(100, 0.1) == b"4567"
    assert dev.read(2, 0.1) == b"89"


def test_read_timeout_returns_empty_and_advances_the_clock(dev, clock):
    dev.open()
    t0 = clock.now()
    assert dev.read(16, 0.25) == b""
    assert clock.now() - t0 == pytest.approx(0.25)


def test_scheduled_data_arrives_when_its_time_comes(dev, clock):
    dev.open()
    dev.inject_rx_after(0.5, b"late")
    assert dev.read(16, 0.2) == b""  # not yet
    assert dev.read(16, 0.5) == b"late"
    assert clock.now() == pytest.approx(0.5)


def test_writes_are_captured_and_short_writes_are_reported(dev):
    dev.open()
    assert dev.write(b"hello") == 5
    assert dev.tx_log == b"hello"
    dev.max_write = 2
    assert dev.write(b"abcdef") == 2


def test_unplugging_breaks_reads_writes_and_open_until_replugged(dev):
    dev.open()
    dev.unplug()
    with pytest.raises(TransportError):
        dev.read(1, 0.1)
    with pytest.raises(TransportError):
        dev.write(b"x")
    dev.close()
    with pytest.raises(TransportError):
        dev.open()
    dev.plug()
    dev.open()
    assert dev.is_open
    dev.inject_rx(b"ok")
    assert dev.read(8, 0.1) == b"ok"


def test_unplug_discards_unread_bytes(dev):
    dev.open()
    dev.inject_rx(b"stale")
    dev.unplug()
    dev.plug()
    dev.open()
    assert dev.read(8, 0.01) == b""


@pytest.mark.skipif(sys.platform == "win32", reason="needs POSIX pseudo-terminals")
def test_pyserial_transport_round_trip_over_a_pty():
    master, slave = os.openpty()
    try:
        name = os.ttyname(slave)
        transport = PySerialTransport(SerialConfig(device=name, baud=115200, read_timeout_s=0.2))
        try:
            transport.open()
        except TransportError as exc:  # some CI sandboxes forbid termios on a pty
            pytest.skip(f"pty not usable here: {exc}")
        os.write(master, b"ping")
        got = b""
        for _ in range(10):
            got += transport.read(16, 0.2)
            if got == b"ping":
                break
        assert got == b"ping"
        assert transport.write(b"pong") == 4
        assert os.read(master, 16) == b"pong"
        transport.close()
        with pytest.raises(TransportClosed):
            transport.read(1, 0.01)
    finally:
        os.close(master)
        os.close(slave)


def test_pyserial_transport_reports_a_missing_device_as_a_transport_error():
    transport = PySerialTransport(SerialConfig(device="/dev/does-not-exist-px4", baud=57600))
    with pytest.raises(TransportError):
        transport.open()
