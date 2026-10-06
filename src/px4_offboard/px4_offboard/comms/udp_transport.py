"""UDP :class:`Transport` for PX4 SITL and companion-link MAVLink endpoints."""

from __future__ import annotations

import select
import socket

from .serial_transport import Transport, TransportClosed, TransportError


class UdpTransport(Transport):
    def __init__(self, spec) -> None:
        if spec.kind not in ("udpin", "udp"):
            raise ValueError("UdpTransport needs a udp/udpin ConnectionSpec")
        self.spec = spec
        self._sock: socket.socket | None = None
        self._peer: tuple[str, int] | None = None

    def open(self) -> None:
        self.close()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            if self.spec.kind == "udpin":
                sock.bind((self.spec.host, self.spec.port))
            else:
                self._peer = (self.spec.host, self.spec.port)
        except OSError as exc:
            sock.close()
            raise TransportError(f"cannot open {self.spec}: {exc}") from exc
        sock.setblocking(False)
        self._sock = sock

    def close(self) -> None:
        sock, self._sock = self._sock, None
        if self.spec.kind == "udpin":
            self._peer = None
        if sock is not None:
            sock.close()

    @property
    def is_open(self) -> bool:
        return self._sock is not None

    def _require(self) -> socket.socket:
        if self._sock is None:
            raise TransportClosed("UDP socket is not open")
        return self._sock

    def read(self, max_bytes: int, timeout_s: float) -> bytes:
        sock = self._require()
        try:
            ready, _, _ = select.select([sock], [], [], max(timeout_s, 0.0))
            if not ready:
                return b""
            data, addr = sock.recvfrom(max_bytes)
        except OSError as exc:
            raise TransportError(f"UDP read failed: {exc}") from exc
        if self.spec.kind == "udpin":
            self._peer = addr  # reply to whoever last spoke to us
        return data

    def write(self, data: bytes) -> int:
        sock = self._require()
        if self._peer is None:
            raise TransportError("no UDP peer yet: nothing has been received")
        try:
            return sock.sendto(data, self._peer)
        except OSError as exc:
            raise TransportError(f"UDP write failed: {exc}") from exc
