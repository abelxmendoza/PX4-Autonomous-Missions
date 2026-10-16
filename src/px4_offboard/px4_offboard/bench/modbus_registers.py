"""Fixed-point Modbus holding-register map for the virtual test rig.

Pure encode/decode, no network -- kept separate from modbus_server.py so the
register layout is testable without a TCP server or pymodbus. All registers
are unsigned 16-bit; physical values are fixed-point scaled by 100
(hundredths of a unit) except rpm (raw, fits in u16 for any sane max_rpm) and
fault_code (a small enum). No bench value this rig produces is negative, so
signed/two's-complement encoding is intentionally not supported here.
"""

from __future__ import annotations

from dataclasses import dataclass

from .virtual_rig import RigSample

COMMAND_PCT = 0
RPM = 1
THRUST_N = 2
LOAD_CELL_N = 3
VOLTAGE_V = 4
TEMP_C = 5
FAULT_CODE = 6

REGISTER_COUNT = 7

FAULT_CODES: dict[str, int] = {
    "": 0,
    "load_cell_dropout": 1,
    "frozen_temp_sensor": 2,
    "voltage_brownout": 3,
}
FAULT_NAMES_BY_CODE: dict[int, str] = {v: k for k, v in FAULT_CODES.items()}


def _clamp_u16(value: float) -> int:
    return max(0, min(65535, round(value)))


def encode_command(command_pct: float) -> int:
    return _clamp_u16(command_pct * 100)


def decode_command(value: int) -> float:
    return value / 100.0


def encode_sample(sample: RigSample) -> list[int]:
    if sample.fault_active not in FAULT_CODES:
        raise ValueError(f"unknown fault for register encoding: {sample.fault_active!r}")
    return [
        _clamp_u16(sample.command_pct * 100),
        _clamp_u16(sample.rpm),
        _clamp_u16(sample.thrust_n * 100),
        _clamp_u16(sample.load_cell_n * 100),
        _clamp_u16(sample.voltage_v * 100),
        _clamp_u16(sample.temp_c * 100),
        FAULT_CODES[sample.fault_active],
    ]


@dataclass(frozen=True)
class DecodedSample:
    command_pct: float
    rpm: float
    thrust_n: float
    load_cell_n: float
    voltage_v: float
    temp_c: float
    fault_active: str


def decode_registers(regs: list[int]) -> DecodedSample:
    if len(regs) != REGISTER_COUNT:
        raise ValueError(f"expected {REGISTER_COUNT} registers, got {len(regs)}")
    fault_code = regs[FAULT_CODE]
    if fault_code not in FAULT_NAMES_BY_CODE:
        raise ValueError(f"unknown fault code in registers: {fault_code}")
    return DecodedSample(
        command_pct=regs[COMMAND_PCT] / 100.0,
        rpm=float(regs[RPM]),
        thrust_n=regs[THRUST_N] / 100.0,
        load_cell_n=regs[LOAD_CELL_N] / 100.0,
        voltage_v=regs[VOLTAGE_V] / 100.0,
        temp_c=regs[TEMP_C] / 100.0,
        fault_active=FAULT_NAMES_BY_CODE[fault_code],
    )
