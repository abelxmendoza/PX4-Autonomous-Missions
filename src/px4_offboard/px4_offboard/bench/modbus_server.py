"""Modbus TCP front end for the virtual test rig.

A real wire protocol (pymodbus), not a mock: a Modbus client sees the same
holding-register map a bench DAQ system would. A background thread steps the
rig at its configured tick rate, writes the sensor registers, and reads back
whatever command value the last Modbus write left in the command register.

Single server per process -- pymodbus's StartTcpServer/ServerStop are
module-level and only track one running server, so only one
BenchModbusServer may be started at a time in a given Python process.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

from pymodbus.datastore import ModbusSequentialDataBlock, ModbusServerContext, ModbusSlaveContext
from pymodbus.server import ServerStop, StartTcpServer

from .modbus_registers import COMMAND_PCT, REGISTER_COUNT, decode_command, encode_sample
from .virtual_rig import RigSample, VirtualTestRig

READ_HOLDING_FC = 3


@dataclass
class BenchModbusServer:
    rig: VirtualTestRig
    host: str = "127.0.0.1"
    port: int = 15020
    samples: list[RigSample] = field(default_factory=list, init=False)
    _store: ModbusSlaveContext = field(init=False, repr=False)
    _context: ModbusServerContext = field(init=False, repr=False)
    _listener_thread: threading.Thread | None = field(default=None, init=False, repr=False)
    _loop_thread: threading.Thread | None = field(default=None, init=False, repr=False)
    _stop_flag: threading.Event = field(default_factory=threading.Event, init=False, repr=False)

    def __post_init__(self) -> None:
        self._store = ModbusSlaveContext(hr=ModbusSequentialDataBlock(0, [0] * REGISTER_COUNT))
        self._context = ModbusServerContext(slaves=self._store, single=True)

    def start(self, listener_settle_s: float = 0.2) -> None:
        if self._listener_thread is not None:
            raise RuntimeError("BenchModbusServer already started")
        self._stop_flag.clear()
        self._listener_thread = threading.Thread(
            target=StartTcpServer,
            kwargs=dict(context=self._context, address=(self.host, self.port)),
            daemon=True,
        )
        self._listener_thread.start()
        time.sleep(listener_settle_s)  # let the TCP listener bind before stepping begins
        self._loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._loop_thread.start()

    def _run_loop(self) -> None:
        dt = self.rig.config.dt_s
        while not self._stop_flag.is_set():
            command_raw = self._store.getValues(READ_HOLDING_FC, COMMAND_PCT, 1)[0]
            sample = self.rig.step(decode_command(command_raw))
            self.samples.append(sample)
            self._store.setValues(READ_HOLDING_FC, 0, encode_sample(sample))
            time.sleep(dt)

    def stop(self, join_timeout_s: float = 5.0) -> None:
        self._stop_flag.set()
        if self._loop_thread is not None:
            self._loop_thread.join(timeout=join_timeout_s)
            self._loop_thread = None
        ServerStop()
        if self._listener_thread is not None:
            self._listener_thread.join(timeout=join_timeout_s)
            self._listener_thread = None
