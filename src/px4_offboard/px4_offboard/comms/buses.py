"""I2C and SPI interfaces plus deterministic mock buses.

Interface-and-mock only: there is no driver for a real bus here, and nothing in
this module may be cited as evidence about hardware. It exists so register
conventions, NACK/retry handling and bus-hang handling are testable in CI, and
so a real ``smbus2``/``spidev`` backend has a defined seam to plug into.
"""

from __future__ import annotations

from abc import ABC, abstractmethod


class BusError(Exception):
    pass


class BusNack(BusError):
    """Device did not acknowledge its address (absent or busy)."""


class BusTimeout(BusError):
    """The bus is hung (e.g. SDA held low); no transaction completes."""


class RegisterDevice:
    """A byte-register peripheral with auto-incrementing burst access."""

    def __init__(self, registers: dict[int, int] | None = None) -> None:
        self.registers: dict[int, int] = dict(registers or {})

    def read(self, reg: int, n: int) -> bytes:
        return bytes(self.registers.get((reg + i) & 0xFF, 0) for i in range(n))

    def write(self, reg: int, data: bytes) -> None:
        for i, byte in enumerate(data):
            self.registers[(reg + i) & 0xFF] = byte


class I2CBus(ABC):
    @abstractmethod
    def read_register(self, addr: int, reg: int, n: int) -> bytes: ...

    @abstractmethod
    def write_register(self, addr: int, reg: int, data: bytes) -> None: ...


class MockI2CBus(I2CBus):
    def __init__(self, devices: dict[int, RegisterDevice]) -> None:
        self._devices = dict(devices)
        self._nacks = 0
        self._hung = False
        self.transactions = 0

    def nack_next(self, count: int) -> None:
        self._nacks = count

    def hang(self) -> None:
        self._hung = True

    def recover(self) -> None:
        self._hung = False

    def _device(self, addr: int) -> RegisterDevice:
        self.transactions += 1
        if self._hung:
            raise BusTimeout("I2C bus hung")
        if self._nacks > 0:
            self._nacks -= 1
            raise BusNack(f"address 0x{addr:02x} NACK")
        if addr not in self._devices:
            raise BusNack(f"address 0x{addr:02x} NACK")
        return self._devices[addr]

    def read_register(self, addr: int, reg: int, n: int) -> bytes:
        return self._device(addr).read(reg, n)

    def write_register(self, addr: int, reg: int, data: bytes) -> None:
        self._device(addr).write(reg, data)


def probe_i2c_device(
    bus: I2CBus, addr: int, who_am_i_reg: int, expected: int, retries: int = 3
) -> bool:
    """True if the chip answers with ``expected``; False if it answers with
    something else. NACKs are retried ``retries`` times in total, then raised;
    a hung bus is not retried (retrying cannot clear it)."""
    last: BusNack | None = None
    for _ in range(max(1, retries)):
        try:
            return bus.read_register(addr, who_am_i_reg, 1)[0] == expected
        except BusNack as exc:
            last = exc
    assert last is not None
    raise last


class SPIBus(ABC):
    @abstractmethod
    def transfer(self, tx: bytes) -> bytes:
        """Full-duplex: returns as many bytes as were sent."""


READ_BIT = 0x80  # the usual MEMS-IMU convention; part-specific in real life


class MockSPIBus(SPIBus):
    def __init__(self, device: RegisterDevice) -> None:
        self._device = device
        self.last_tx = b""

    def transfer(self, tx: bytes) -> bytes:
        if not tx:
            raise ValueError("SPI transfer needs at least the command byte")
        self.last_tx = bytes(tx)
        reg = tx[0] & ~READ_BIT & 0xFF
        if tx[0] & READ_BIT:
            return b"\x00" + self._device.read(reg, len(tx) - 1)
        self._device.write(reg, bytes(tx[1:]))
        return b"\x00" * len(tx)


def spi_read_register(bus: SPIBus, reg: int, n: int) -> bytes:
    return bus.transfer(bytes([reg | READ_BIT]) + b"\x00" * n)[1:]


def spi_write_register(bus: SPIBus, reg: int, data: bytes) -> None:
    bus.transfer(bytes([reg & ~READ_BIT & 0xFF]) + bytes(data))
