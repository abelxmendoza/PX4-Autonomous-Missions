"""I2C / SPI interface stubs with mock buses. These test driver-side logic
(NACK retry, bus hang, register conventions). They say nothing about real
silicon: no physical bus was exercised."""
from __future__ import annotations

import pytest

from px4_offboard.comms.buses import (
    BusNack,
    BusTimeout,
    MockI2CBus,
    MockSPIBus,
    RegisterDevice,
    probe_i2c_device,
    spi_read_register,
    spi_write_register,
)

WHO_AM_I = 0x75


def device():
    return RegisterDevice({WHO_AM_I: 0x68, 0x3B: 0x12, 0x3C: 0x34, 0x3D: 0x56})


def test_i2c_reads_a_register_and_auto_increments_for_burst_reads():
    bus = MockI2CBus({0x68: device()})
    assert bus.read_register(0x68, WHO_AM_I, 1) == b"\x68"
    assert bus.read_register(0x68, 0x3B, 3) == b"\x12\x34\x56"


def test_i2c_write_then_read_back():
    bus = MockI2CBus({0x68: device()})
    bus.write_register(0x68, 0x6B, b"\x01")
    assert bus.read_register(0x68, 0x6B, 1) == b"\x01"


def test_absent_address_nacks():
    bus = MockI2CBus({0x68: device()})
    with pytest.raises(BusNack):
        bus.read_register(0x69, WHO_AM_I, 1)


def test_hung_bus_times_out():
    bus = MockI2CBus({0x68: device()})
    bus.hang()
    with pytest.raises(BusTimeout):
        bus.read_register(0x68, WHO_AM_I, 1)
    bus.recover()
    assert bus.read_register(0x68, WHO_AM_I, 1) == b"\x68"


def test_probe_retries_through_transient_nacks():
    bus = MockI2CBus({0x68: device()})
    bus.nack_next(2)
    assert probe_i2c_device(bus, 0x68, WHO_AM_I, expected=0x68, retries=3) is True
    assert bus.transactions == 3


def test_probe_gives_up_after_the_retry_budget():
    bus = MockI2CBus({0x68: device()})
    bus.nack_next(5)
    with pytest.raises(BusNack):
        probe_i2c_device(bus, 0x68, WHO_AM_I, expected=0x68, retries=3)


def test_probe_reports_a_wrong_chip_id_instead_of_raising():
    bus = MockI2CBus({0x68: RegisterDevice({WHO_AM_I: 0x71})})
    assert probe_i2c_device(bus, 0x68, WHO_AM_I, expected=0x68, retries=1) is False


def test_spi_register_read_sets_the_read_bit_and_clocks_out_dummy_bytes():
    bus = MockSPIBus(device())
    assert spi_read_register(bus, WHO_AM_I, 1) == b"\x68"
    assert bus.last_tx[0] == WHO_AM_I | 0x80
    assert spi_read_register(bus, 0x3B, 3) == b"\x12\x34\x56"


def test_spi_register_write_clears_the_read_bit():
    dev = device()
    bus = MockSPIBus(dev)
    spi_write_register(bus, 0x6B, b"\x01")
    assert bus.last_tx[0] == 0x6B
    assert dev.registers[0x6B] == 0x01


def test_spi_rejects_empty_transfers():
    bus = MockSPIBus(device())
    with pytest.raises(ValueError):
        bus.transfer(b"")
