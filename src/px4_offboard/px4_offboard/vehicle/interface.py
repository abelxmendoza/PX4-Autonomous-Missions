"""The contract validation code is written against. It must not mention SITL,
serial ports or simulators: anything target-specific lives in a subclass."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum


class VehicleKind(Enum):
    SIM = "sim"
    SITL = "px4_sitl"
    HARDWARE = "px4_hardware"


class VehicleError(Exception):
    pass


class CommandRejected(VehicleError):
    """The autopilot answered and refused."""


class CommandTimeout(VehicleError):
    """No answer within the command timeout."""


class ActuationNotPermitted(VehicleError):
    """A command that could move a real aircraft was attempted without opt-in."""


@dataclass(frozen=True)
class VehicleState:
    t_s: float
    position_ned: tuple[float, float, float]
    velocity_ned: tuple[float, float, float]
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    armed: bool


@dataclass(frozen=True)
class LinkHealth:
    connected: bool
    alive: bool
    rx_rate_hz: float
    last_rx_age_s: float | None


class VehicleInterface(ABC):
    kind: VehicleKind

    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def update(self, timeout_s: float = 0.1) -> None:
        """Process pending I/O for up to ``timeout_s``; refresh ``state``."""

    @property
    @abstractmethod
    def state(self) -> VehicleState | None: ...

    @abstractmethod
    def arm(self) -> None: ...

    @abstractmethod
    def disarm(self) -> None: ...

    @abstractmethod
    def set_velocity(self, vn: float, ve: float, vd: float) -> None:
        """One velocity setpoint (NED, m/s). Offboard control needs a stream."""

    @abstractmethod
    def link_health(self) -> LinkHealth: ...
