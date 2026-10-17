# Virtual bench rig (Modbus TCP test stand)

[Project overview](../README.md) · [Browser replay](https://replay-sepia-tau.vercel.app/bench/) ·
[HIL architecture](HIL_ARCHITECTURE.md)

A software digital twin of a motor/propeller **thrust-test bench** -- not a flight controller, not
PX4. It exists to demonstrate bench DAQ/test-infrastructure work (scripted campaigns, fault
injection, pass/fail requirement evidence against a real industrial protocol) separately from the
PX4/MAVLink HIL path already covered in [HIL_ARCHITECTURE.md](HIL_ARCHITECTURE.md), which is about
a flight controller over serial/UDP, not a bench rig over Modbus.

**Status, stated plainly:** everything below has been run on this machine (macOS, no PX4/Gazebo
required). There is no physical rig, load cell, or motor behind it -- the model is a plausible
first-order approximation tuned for believable magnitudes, not fit to measured hardware.

## What it is

```
src/px4_offboard/px4_offboard/bench/
  virtual_rig.py       -- VirtualTestRig: command_pct -> rpm -> thrust_n/load_cell_n/voltage_v/temp_c
  modbus_registers.py  -- fixed-point encode/decode for a 7-register Modbus holding-register map
  modbus_server.py     -- BenchModbusServer: real Modbus TCP server (pymodbus) over the rig
  campaign.py           -- scripted setpoints + fault windows -> samples + REQ-BENCH-00x verdicts
scripts/run_bench_campaign.py   -- regenerates the committed golden evidence (deterministic, seeded)
web/replay/bench/                -- instrument-panel replay of the golden campaign
```

| Piece | What it models | What it does NOT model |
| --- | --- | --- |
| `VirtualTestRig` | RPM first-order lag, a quadratic thrust curve, battery sag under load, thermal climb toward a load-dependent steady state, Gaussian sensor noise | Motor electrical dynamics (current, back-EMF), real aerodynamics (induced drag, prop stall), vibration, any fit to a physical rig |
| `modbus_registers.py` | A 7-register unsigned-16-bit holding-register map, fixed-point x100 scaling, a small fault-code enum | Signed values (nothing this rig produces is negative), coils/discrete inputs, more than one Modbus device |
| `BenchModbusServer` | A real Modbus TCP server (`pymodbus`) backed by the rig, stepped by a background thread at the rig's tick rate | Serial (RTU) Modbus, more than one server per process (a `pymodbus` sync-API limitation) |
| `campaign.py` | A scripted throttle sweep with two injected sensor faults, scored against four requirements | Live operation over the wire (see below) |

## Requirements

| ID | Statement | How it's checked |
| --- | --- | --- |
| REQ-BENCH-001 | Thrust is never negative | No sample's modeled `thrust_n` is below zero across the sweep |
| REQ-BENCH-002 | A load-cell dropout fault is observable in the reading | Every injected `load_cell_dropout` window has at least one sample reading exactly zero while the fault is flagged active -- not trusted by declaration alone |
| REQ-BENCH-003 | Battery voltage stays above a 9.0 V floor outside an injected brownout | No sample below the floor unless `voltage_brownout` is the active fault |
| REQ-BENCH-004 | The sample timeline has no gaps beyond 1.5x the tick interval | Consecutive sample timestamps checked against the campaign's `dt_s` |
| REQ-BENCH-005 | A frozen temperature sensor is detected within 1 s, with no false alarms | `detect_stuck_sensor` flags a channel whose reading has not changed at all for 0.5 s (live readings always carry noise). Every injected `frozen_temp_sensor` window must be flagged within 1 s of its start, and nothing may be flagged outside one. The golden run flags it at 11.50 s (freeze injected at 11.0 s); the report's `events` list records it |

`src/px4_offboard/test/test_bench_campaign.py` includes a negative-regression test for
REQ-BENCH-002/003: hand-built sample lists where a declared fault has no visible effect, or an
unlabeled brownout slips through, plus REQ-BENCH-005 cases where a declared freeze never shows up or a healthy reading is flagged, confirming the requirement actually fails rather than passing by
trusting the fault declaration -- the same "deliberately re-break it and watch the check fail"
discipline used elsewhere in this repo's fault injection.

## Running it

In-process (deterministic, what produced the committed evidence):

```bash
PYTHONPATH=src/px4_offboard python3 scripts/run_bench_campaign.py
```

Writes `evidence/bench/golden_report.json`, `evidence/bench/golden_campaign.jsonl`, and
`web/replay/data/bench_campaign.json` (consumed by the browser replay).

Live, over the wire -- a real Modbus client against a real Modbus TCP server:

```python
from px4_offboard.bench.modbus_server import BenchModbusServer
from px4_offboard.bench.virtual_rig import VirtualTestRig

server = BenchModbusServer(rig=VirtualTestRig(), port=15020)
server.start()
# a Modbus client (e.g. pymodbus's ModbusTcpClient, or any Modbus tool) can now
# write register 0 (command_pct x100) and read registers 0-6 back.
server.stop()
```

Tests: `src/px4_offboard/test/test_bench_virtual_rig.py`, `test_bench_registers.py`,
`test_bench_campaign.py` are fast and deterministic (no network). `test_bench_modbus_server.py` is
marked `integration` (real sockets/threads, ~1 s) and is skipped automatically if `pymodbus` is not
installed:

```bash
PYTHONPATH=src/px4_offboard python3 -m pytest src/px4_offboard/test -k bench
```

## Known limits

- The golden campaign does not inject a battery brownout, so REQ-BENCH-003 is only exercised against normal load sag.

- The campaign runner drives the rig in-process; it does not drive the committed golden evidence
  over the Modbus wire, because real socket/thread timing is not something to pin as deterministic
  evidence. The live protocol path is exercised by its own integration test
  (`test_bench_modbus_server.py`), not by the golden recording.
- One fault active at a time: the rig's fault list is checked in declaration order and the model
  does not define what happens if two windows overlap. Campaigns here don't do that.
- Single Modbus server per process (`pymodbus`'s sync `StartTcpServer`/`ServerStop` are
  module-level), so only one `BenchModbusServer` may run at a time in a given Python process.
- Not part of `evidence/manifest.json`'s SHA-256-pinned case list (see
  [evidence/README.md](../evidence/README.md)): unlike the PX4/Gazebo flight recordings there,
  this campaign has no one-off recording session to pin -- the same seed always reproduces the
  same samples, so it is regenerated by re-running the script instead.
