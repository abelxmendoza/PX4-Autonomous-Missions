"""Vehicle adapters: what a spec runs against, and an honest label for it.

An adapter owns a ``VehicleInterface`` and the clock that drives it, and states
in ``environment`` exactly what kind of target it is. The report copies that
label verbatim, so a mock run can never be presented as PX4 SITL or hardware.

Only the mock adapter exists in this slice. It wraps the existing
``vehicle.sim.SimVehicle`` (a first-order velocity-tracking point mass) on a
``FakeClock``: no PX4, no MAVLink, no EKF, no ground contact, no failsafes.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from px4_offboard.comms.clock import Clock, FakeClock
from px4_offboard.vehicle.interface import VehicleInterface
from px4_offboard.vehicle.sim import SimVehicle

from .schema import VehicleSpec


class AdapterError(RuntimeError):
    """The target could not be brought up or stopped responding (infrastructure)."""


class VehicleAdapter(ABC):
    name: str
    vehicle: VehicleInterface
    clock: Clock
    rate_hz: float

    @property
    @abstractmethod
    def environment(self) -> dict[str, Any]: ...

    @abstractmethod
    def start(self) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...

    def tick(self) -> None:
        """Advance the target by one control period and refresh ``vehicle.state``."""
        self.vehicle.update(1.0 / self.rate_hz)


class MockAdapter(VehicleAdapter):
    name = "mock"

    def __init__(self, rate_hz: float, velocity_tau_s: float) -> None:
        self.rate_hz = rate_hz
        self.clock = FakeClock()
        self.vehicle = SimVehicle(self.clock, velocity_tau_s=velocity_tau_s)
        self._tau = velocity_tau_s

    @property
    def environment(self) -> dict[str, Any]:
        return {
            "kind": "mock",
            "label": "MOCK: SimVehicle point-mass test double on simulated time. Not PX4, not SITL, not HIL.",
            "adapter": self.name,
            "vehicle_class": type(self.vehicle).__name__,
            "vehicle_kind": self.vehicle.kind.value,
            "clock": "FakeClock (simulated time; advances only when the runner ticks)",
            "px4_in_loop": False,
            "px4_sitl": False,
            "hil": False,
            "physical_flight": False,
            "model_limits": [
                "first-order velocity tracking (tau = %.3g s); no attitude dynamics" % self._tau,
                "no ground contact, wind, actuator limits, estimator or failsafes",
                "state is exact simulator truth, not an estimate",
            ],
        }

    def start(self) -> None:
        self.vehicle.connect()

    def stop(self) -> None:
        self.vehicle.disconnect()


def build_adapter(spec: VehicleSpec) -> VehicleAdapter:
    if spec.adapter == "mock":
        return MockAdapter(**spec.params)
    raise AdapterError(f"no adapter implementation for {spec.adapter!r}")
