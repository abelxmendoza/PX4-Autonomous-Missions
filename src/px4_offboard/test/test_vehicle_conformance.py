"""One behavioural contract, run against every VehicleInterface implementation.

If a test here needs ``if isinstance(vehicle, ...)`` the abstraction has leaked.
SITL and hardware vehicles run against a mock MAVLink autopilot over a mock
transport: that exercises the host-side software only. No physical flight
controller and no PX4 SITL process is involved in this file.
"""
from __future__ import annotations

import pytest

from px4_offboard.comms.clock import FakeClock
from px4_offboard.comms.serial_transport import MockSerialDevice, SerialConfig
from px4_offboard.vehicle.interface import (
    ActuationNotPermitted,
    CommandRejected,
    CommandTimeout,
    VehicleInterface,
    VehicleKind,
)
from px4_offboard.vehicle.mavlink_vehicle import PX4HardwareVehicle, PX4SITLVehicle
from px4_offboard.vehicle.mock_autopilot import MockAutopilot
from px4_offboard.vehicle.sim import SimVehicle


class World:
    """Advance the simulated world and let the vehicle process I/O."""

    def __init__(self, vehicle, clock, autopilot=None):
        self.vehicle, self.clock, self.autopilot = vehicle, clock, autopilot

    def run(self, seconds: float, dt: float = 0.05) -> None:
        for _ in range(int(round(seconds / dt))):
            if self.autopilot is not None:
                self.clock.advance(dt)
                self.autopilot.step(dt)
                self.vehicle.update(0.0)
            else:
                self.vehicle.update(dt)


def _mavlink_world(cls, **kwargs):
    clock = FakeClock()
    dev = MockSerialDevice(SerialConfig(device="mock0", baud=921600), clock)
    autopilot = MockAutopilot(dev, clock)
    vehicle = cls(transport=dev, clock=clock, **kwargs)
    return World(vehicle, clock, autopilot)


@pytest.fixture(params=["sim", "sitl", "hardware"])
def world(request):
    if request.param == "sim":
        clock = FakeClock()
        return World(SimVehicle(clock), clock)
    if request.param == "sitl":
        return _mavlink_world(PX4SITLVehicle)
    return _mavlink_world(PX4HardwareVehicle, allow_actuation=True)


def test_every_implementation_satisfies_the_interface(world):
    assert isinstance(world.vehicle, VehicleInterface)
    assert isinstance(world.vehicle.kind, VehicleKind)


def test_no_state_before_connect_and_not_alive(world):
    assert world.vehicle.state is None
    assert world.vehicle.link_health().alive is False


def test_telemetry_arrives_after_connect(world):
    world.vehicle.connect()
    world.run(1.0)
    state = world.vehicle.state
    assert state is not None
    assert state.armed is False
    assert len(state.position_ned) == 3 and len(state.velocity_ned) == 3
    health = world.vehicle.link_health()
    assert health.connected and health.alive


def test_arm_then_disarm(world):
    world.vehicle.connect()
    world.run(0.5)
    world.vehicle.arm()
    world.run(1.5)  # armed state travels in the 1 Hz heartbeat
    assert world.vehicle.state.armed is True
    world.vehicle.disarm()
    world.run(1.5)
    assert world.vehicle.state.armed is False


def test_velocity_command_moves_the_vehicle_north(world):
    world.vehicle.connect()
    world.run(0.5)
    world.vehicle.arm()
    start_n = world.vehicle.state.position_ned[0]
    for _ in range(40):  # setpoints must be streamed, like real offboard control
        world.vehicle.set_velocity(2.0, 0.0, 0.0)
        world.run(0.1)
    state = world.vehicle.state
    assert state.position_ned[0] - start_n > 2.0
    assert state.velocity_ned[0] == pytest.approx(2.0, abs=0.5)


def test_disconnect_marks_the_link_down(world):
    world.vehicle.connect()
    world.run(0.5)
    world.vehicle.disconnect()
    assert world.vehicle.link_health().connected is False


# --- behaviour that is specific by design, not by leakage --------------------

def test_hardware_vehicle_is_telemetry_only_unless_actuation_is_explicit():
    world = _mavlink_world(PX4HardwareVehicle)  # default: allow_actuation=False
    world.vehicle.connect()
    world.run(0.5)
    assert world.vehicle.state is not None  # reading is fine
    with pytest.raises(ActuationNotPermitted):
        world.vehicle.arm()
    with pytest.raises(ActuationNotPermitted):
        world.vehicle.set_velocity(1.0, 0.0, 0.0)
    assert world.autopilot.commands_received == []  # nothing reached the wire


def test_sitl_and_hardware_report_their_kind():
    assert _mavlink_world(PX4SITLVehicle).vehicle.kind is VehicleKind.SITL
    assert _mavlink_world(PX4HardwareVehicle).vehicle.kind is VehicleKind.HARDWARE
    assert SimVehicle(FakeClock()).kind is VehicleKind.SIM


def test_rejected_arm_raises_command_rejected():
    world = _mavlink_world(PX4SITLVehicle)
    world.autopilot.arm_result = 4  # MAV_RESULT_FAILED
    world.vehicle.connect()
    world.run(0.5)
    with pytest.raises(CommandRejected):
        world.vehicle.arm()


def test_unanswered_arm_times_out_instead_of_hanging():
    world = _mavlink_world(PX4SITLVehicle, command_timeout_s=0.5)
    world.autopilot.respond_to_commands = False
    world.vehicle.connect()
    world.run(0.5)
    with pytest.raises(CommandTimeout):
        world.vehicle.arm()
    assert world.clock.now() < 5.0  # bounded, not stuck


def test_mavlink_vehicle_recovers_after_the_cable_is_unplugged():
    world = _mavlink_world(PX4HardwareVehicle)
    world.vehicle.connect()
    world.run(1.0)
    world.autopilot.device.unplug()
    world.run(0.5)
    assert world.vehicle.link_health().connected is False
    world.autopilot.device.plug()
    assert world.vehicle.reconnect() is True
    world.run(1.0)
    assert world.vehicle.link_health().alive
