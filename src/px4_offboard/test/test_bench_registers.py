"""Modbus register encode/decode round-trip for the virtual rig."""
from __future__ import annotations

import pytest

from px4_offboard.bench.modbus_registers import (
    REGISTER_COUNT,
    decode_command,
    decode_registers,
    encode_command,
    encode_sample,
)
from px4_offboard.bench.virtual_rig import RigSample


def test_round_trip_preserves_values_within_fixed_point_precision():
    sample = RigSample(
        t=1.5, command_pct=73.21, rpm=6543.0, thrust_n=12.345,
        load_cell_n=12.30, voltage_v=15.87, temp_c=61.42, fault_active="",
    )
    regs = encode_sample(sample)
    assert len(regs) == REGISTER_COUNT
    decoded = decode_registers(regs)
    assert decoded.command_pct == pytest.approx(sample.command_pct, abs=0.01)
    assert decoded.rpm == pytest.approx(sample.rpm, abs=1.0)
    assert decoded.thrust_n == pytest.approx(sample.thrust_n, abs=0.01)
    assert decoded.load_cell_n == pytest.approx(sample.load_cell_n, abs=0.01)
    assert decoded.voltage_v == pytest.approx(sample.voltage_v, abs=0.01)
    assert decoded.temp_c == pytest.approx(sample.temp_c, abs=0.01)
    assert decoded.fault_active == ""


def test_fault_code_round_trips_by_name():
    sample = RigSample(0, 0, 0, 0, 0, 16.0, 25.0, fault_active="load_cell_dropout")
    decoded = decode_registers(encode_sample(sample))
    assert decoded.fault_active == "load_cell_dropout"


def test_unknown_fault_name_is_rejected_at_encode_time():
    sample = RigSample(0, 0, 0, 0, 0, 16.0, 25.0, fault_active="not_a_real_fault")
    with pytest.raises(ValueError):
        encode_sample(sample)


def test_command_encode_decode_round_trip():
    assert decode_command(encode_command(55.5)) == pytest.approx(55.5, abs=0.01)


def test_values_clamp_into_the_u16_range_instead_of_wrapping():
    sample = RigSample(0, 100.0, 70000.0, 1000.0, 1000.0, 1000.0, 1000.0, fault_active="")
    regs = encode_sample(sample)
    assert all(0 <= r <= 65535 for r in regs)


def test_decode_registers_rejects_the_wrong_length():
    with pytest.raises(ValueError):
        decode_registers([0, 0, 0])


def test_decode_registers_rejects_an_unknown_fault_code():
    with pytest.raises(ValueError):
        decode_registers([0, 0, 0, 0, 0, 0, 255])
