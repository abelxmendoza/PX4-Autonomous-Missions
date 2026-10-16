"""Live Modbus TCP round trip against the virtual rig.

Marked `integration`: real sockets and threads, seconds not milliseconds, so
it is excluded from the fast CI job (see pytest.ini). Skipped outright if
pymodbus is not installed. Polls with a timeout instead of fixed sleeps, so
it is not tied to exact thread-scheduling timing.
"""
from __future__ import annotations

import socket
import time

import pytest

pymodbus_client = pytest.importorskip("pymodbus.client")

from px4_offboard.bench.modbus_registers import decode_registers  # noqa: E402
from px4_offboard.bench.modbus_server import BenchModbusServer  # noqa: E402
from px4_offboard.bench.virtual_rig import FaultWindow, RigConfig, VirtualTestRig  # noqa: E402

pytestmark = pytest.mark.integration


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _poll(predicate, timeout_s: float = 5.0, interval_s: float = 0.05):
    deadline = time.monotonic() + timeout_s
    last = None
    while time.monotonic() < deadline:
        last = predicate()
        if last is not None:
            return last
        time.sleep(interval_s)
    raise AssertionError(f"condition not met within {timeout_s}s (last={last!r})")


def test_command_write_and_sensor_readback_over_real_modbus_tcp():
    rig = VirtualTestRig(RigConfig(dt_s=0.02, seed=11))
    server = BenchModbusServer(rig=rig, port=_free_port())
    client = pymodbus_client.ModbusTcpClient(server.host, port=server.port)
    try:
        server.start()
        assert client.connect()

        client.write_register(0, 8000, slave=0)  # 80.00% command

        def thrust_rose():
            regs = client.read_holding_registers(0, 7, slave=0).registers
            decoded = decode_registers(regs)
            return decoded if decoded.thrust_n > 5.0 else None

        decoded = _poll(thrust_rose, timeout_s=5.0)
        assert decoded.command_pct == pytest.approx(80.0, abs=0.01)
        assert decoded.thrust_n > 5.0
    finally:
        client.close()
        server.stop()


def test_a_fault_window_on_the_rig_is_visible_through_the_wire():
    rig = VirtualTestRig(RigConfig(dt_s=0.02, seed=12))
    rig.add_fault(FaultWindow("load_cell_dropout", start_s=0.1, duration_s=2.0))
    server = BenchModbusServer(rig=rig, port=_free_port())
    client = pymodbus_client.ModbusTcpClient(server.host, port=server.port)
    try:
        server.start()
        assert client.connect()
        client.write_register(0, 10000, slave=0)  # 100.00%

        def dropout_seen():
            regs = client.read_holding_registers(0, 7, slave=0).registers
            decoded = decode_registers(regs)
            return decoded if decoded.fault_active == "load_cell_dropout" else None

        decoded = _poll(dropout_seen, timeout_s=5.0)
        assert decoded.load_cell_n == 0.0
    finally:
        client.close()
        server.stop()
