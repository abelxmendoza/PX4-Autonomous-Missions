"""Connection strings and the UDP transport (loopback only)."""
from __future__ import annotations

import socket

import pytest

from px4_offboard.comms.serial_transport import TransportClosed, TransportError
from px4_offboard.comms.udp_transport import UdpTransport
from px4_offboard.vehicle.connection import ConnectionSpec, parse_connection


@pytest.mark.parametrize(
    "text,kind,host,port,device,baud",
    [
        ("udpin:0.0.0.0:14540", "udpin", "0.0.0.0", 14540, None, None),
        ("udp:127.0.0.1:14550", "udp", "127.0.0.1", 14550, None, None),
        ("serial:/dev/ttyACM0:921600", "serial", None, None, "/dev/ttyACM0", 921600),
        ("serial:/dev/ttyUSB0", "serial", None, None, "/dev/ttyUSB0", 57600),
        ("serial:COM3:115200", "serial", None, None, "COM3", 115200),
    ],
)
def test_valid_connection_strings(text, kind, host, port, device, baud):
    spec = parse_connection(text)
    assert (spec.kind, spec.host, spec.port, spec.device, spec.baud) == (kind, host, port, device, baud)


@pytest.mark.parametrize(
    "text",
    ["", "udp", "udp:127.0.0.1", "udp:127.0.0.1:notaport", "udp:127.0.0.1:70000",
     "serial:", "serial:/dev/ttyACM0:1234", "carrier-pigeon:1:2"],
)
def test_invalid_connection_strings_are_rejected(text):
    with pytest.raises(ValueError):
        parse_connection(text)


def test_spec_roundtrips_through_its_string_form():
    for text in ("udpin:0.0.0.0:14540", "serial:/dev/ttyACM0:921600"):
        assert str(parse_connection(text)) == text


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_udpin_learns_the_peer_and_replies_to_it():
    port = _free_port()
    server = UdpTransport(ConnectionSpec("udpin", host="127.0.0.1", port=port))
    server.open()
    peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    peer.bind(("127.0.0.1", 0))
    try:
        assert server.read(16, 0.05) == b""  # nothing yet: timeout, not an error
        peer.sendto(b"hello", ("127.0.0.1", port))
        assert server.read(16, 1.0) == b"hello"
        assert server.write(b"world") == 5
        peer.settimeout(1.0)
        assert peer.recvfrom(16)[0] == b"world"
    finally:
        peer.close()
        server.close()


def test_udpin_cannot_write_before_a_peer_is_known():
    port = _free_port()
    server = UdpTransport(ConnectionSpec("udpin", host="127.0.0.1", port=port))
    server.open()
    try:
        with pytest.raises(TransportError):
            server.write(b"x")
    finally:
        server.close()


def test_udp_out_sends_to_the_configured_remote():
    remote = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    remote.bind(("127.0.0.1", 0))
    remote.settimeout(1.0)
    port = remote.getsockname()[1]
    client = UdpTransport(ConnectionSpec("udp", host="127.0.0.1", port=port))
    client.open()
    try:
        client.write(b"ping")
        assert remote.recvfrom(16)[0] == b"ping"
    finally:
        client.close()
        remote.close()


def test_use_after_close_and_double_bind_are_transport_errors():
    port = _free_port()
    a = UdpTransport(ConnectionSpec("udpin", host="127.0.0.1", port=port))
    a.open()
    b = UdpTransport(ConnectionSpec("udpin", host="127.0.0.1", port=port))
    try:
        with pytest.raises(TransportError):
            b.open()
    finally:
        a.close()
    with pytest.raises(TransportClosed):
        a.read(1, 0.01)
