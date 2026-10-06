"""Serial transport abstraction: pyserial for a real UART, a deterministic mock
for CI. Both implement :class:`Transport`, so the link layer above cannot tell
them apart."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass

from .clock import Clock

STANDARD_BAUDS = frozenset(
    {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000, 921600, 1500000, 2000000}
)


class TransportError(Exception):
    """The device is gone or misbehaving; the link should reconnect."""


class PeerUnknown(TransportError):
    """A connectionless transport has nobody to send to yet (not a device failure)."""


class TransportClosed(TransportError):
    """Operation attempted on a transport that is not open."""


@dataclass(frozen=True)
class SerialConfig:
    device: str
    baud: int = 57600
    data_bits: int = 8
    parity: str = "N"
    stop_bits: int = 1
    read_timeout_s: float = 0.1

    def __post_init__(self) -> None:
        if not self.device:
            raise ValueError("serial device path must not be empty")
        if self.baud not in STANDARD_BAUDS:
            raise ValueError(f"baud {self.baud} is not a standard UART rate")
        if self.data_bits not in (5, 6, 7, 8):
            raise ValueError("data_bits must be 5-8")
        if self.parity not in ("N", "E", "O"):
            raise ValueError("parity must be N, E or O")
        if self.stop_bits not in (1, 2):
            raise ValueError("stop_bits must be 1 or 2")
        if self.read_timeout_s <= 0:
            raise ValueError("read_timeout_s must be positive")


class Transport(ABC):
    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @property
    @abstractmethod
    def is_open(self) -> bool: ...

    @abstractmethod
    def read(self, max_bytes: int, timeout_s: float) -> bytes:
        """Up to ``max_bytes``; ``b""`` on timeout. Raises on device loss."""

    @abstractmethod
    def write(self, data: bytes) -> int:
        """Bytes accepted (may be fewer than ``len(data)``)."""


class PySerialTransport(Transport):
    """A real UART (``/dev/ttyACM0``, ``/dev/ttyUSB0``, ``COM3``) via pyserial."""

    def __init__(self, config: SerialConfig) -> None:
        self.config = config
        self._port = None

    def open(self) -> None:
        import serial  # imported lazily: CI without pyserial can still import this module

        self.close()
        try:
            self._port = serial.Serial(
                port=self.config.device,
                baudrate=self.config.baud,
                bytesize=self.config.data_bits,
                parity=self.config.parity,
                stopbits=self.config.stop_bits,
                timeout=self.config.read_timeout_s,
                write_timeout=1.0,
            )
        except (OSError, ValueError, serial.SerialException) as exc:
            self._port = None
            raise TransportError(f"cannot open {self.config.device}: {exc}") from exc

    def close(self) -> None:
        port, self._port = self._port, None
        if port is not None:
            try:
                port.close()
            except Exception:  # closing a vanished device must not raise
                pass

    @property
    def is_open(self) -> bool:
        return self._port is not None and self._port.is_open

    def _require_open(self):
        if not self.is_open:
            raise TransportClosed("serial port is not open")
        return self._port

    def read(self, max_bytes: int, timeout_s: float) -> bytes:
        import serial

        port = self._require_open()
        try:
            port.timeout = timeout_s
            return port.read(max_bytes)
        except (OSError, serial.SerialException) as exc:
            raise TransportError(f"read failed: {exc}") from exc

    def write(self, data: bytes) -> int:
        import serial

        port = self._require_open()
        try:
            return int(port.write(data) or 0)
        except (OSError, serial.SerialException) as exc:
            raise TransportError(f"write failed: {exc}") from exc


class MockSerialDevice(Transport):
    """Deterministic UART stand-in driven by an injected clock.

    Not a model of any electrical behaviour: it reproduces the *software-visible*
    conditions a driver must survive (data in chunks, late data, short writes,
    unplug/replug) so those paths run in CI without a device.
    """

    def __init__(self, config: SerialConfig, clock: Clock) -> None:
        self.config = config
        self._clock = clock
        self._open = False
        self._plugged = True
        self._rx: deque[tuple[float, bytes]] = deque()  # (ready_time, bytes)
        self.tx_log = bytearray()
        self.max_chunk = 1 << 16
        self.max_write = 1 << 16
        self.open_attempts = 0
        self.on_write = None  # optional hook(bytes): lets a mock peer react to TX

    # --- test controls -------------------------------------------------
    def inject_rx(self, data: bytes) -> None:
        self.inject_rx_after(0.0, data)

    def inject_rx_after(self, delay_s: float, data: bytes) -> None:
        self._rx.append((self._clock.now() + delay_s, bytes(data)))

    def unplug(self) -> None:
        self._plugged = False
        self._rx.clear()

    def plug(self) -> None:
        self._plugged = True

    @property
    def plugged(self) -> bool:
        return self._plugged

    # --- Transport -----------------------------------------------------
    def open(self) -> None:
        self.open_attempts += 1
        if not self._plugged:
            raise TransportError(f"{self.config.device}: no such device")
        self._open = True

    def close(self) -> None:
        self._open = False

    @property
    def is_open(self) -> bool:
        return self._open

    def _check(self) -> None:
        if not self._open:
            raise TransportClosed("mock serial device is not open")
        if not self._plugged:
            raise TransportError(f"{self.config.device}: device disconnected")

    def read(self, max_bytes: int, timeout_s: float) -> bytes:
        self._check()
        deadline = self._clock.now() + timeout_s
        while True:
            if self._rx and self._rx[0][0] <= self._clock.now():
                ready, data = self._rx.popleft()
                take = min(max_bytes, self.max_chunk)
                if len(data) > take:
                    self._rx.appendleft((ready, data[take:]))
                    data = data[:take]
                return data
            next_ready = self._rx[0][0] if self._rx else float("inf")
            if next_ready <= deadline:
                self._clock.sleep(next_ready - self._clock.now())
                continue
            self._clock.sleep(deadline - self._clock.now())
            return b""

    def write(self, data: bytes) -> int:
        self._check()
        accepted = bytes(data[: self.max_write])
        self.tx_log += accepted
        if self.on_write is not None and accepted:
            self.on_write(accepted)
        return len(accepted)
