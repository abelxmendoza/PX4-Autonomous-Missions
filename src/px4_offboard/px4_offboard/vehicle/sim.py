"""SimVehicle: a deterministic offline stand-in for a vehicle.

Two modes behind one class: a first-order velocity-tracking point mass
(``SimVehicle(clock)``) or replay of recorded states (``SimVehicle(clock,
replay=states)``). It is a software test double, not a flight-dynamics model:
no attitude dynamics, wind, or actuator limits.
"""

from __future__ import annotations

from typing import Sequence

from px4_offboard.comms.clock import Clock

from .interface import LinkHealth, VehicleInterface, VehicleKind, VehicleState


class SimVehicle(VehicleInterface):
    kind = VehicleKind.SIM

    def __init__(
        self,
        clock: Clock,
        replay: Sequence[VehicleState] | None = None,
        velocity_tau_s: float = 0.3,
    ) -> None:
        self._clock = clock
        self._replay = list(replay) if replay is not None else None
        self._tau = velocity_tau_s
        self._connected = False
        self._armed = False
        self._pos = [0.0, 0.0, 0.0]
        self._vel = [0.0, 0.0, 0.0]
        self._cmd = [0.0, 0.0, 0.0]
        self._state: VehicleState | None = None
        self._t0 = 0.0
        self._last_update = 0.0
        self.commands_received: list[tuple] = []

    def connect(self) -> None:
        self._connected = True
        self._t0 = self._last_update = self._clock.now()

    def disconnect(self) -> None:
        self._connected = False

    @property
    def state(self) -> VehicleState | None:
        return self._state

    def update(self, timeout_s: float = 0.1) -> None:
        if not self._connected:
            return
        self._clock.sleep(timeout_s)
        now = self._clock.now()
        dt = now - self._last_update
        self._last_update = now
        if self._replay is not None:
            elapsed = now - self._t0
            current = None
            for sample in self._replay:
                if sample.t_s <= elapsed:
                    current = sample
                else:
                    break
            self._state = current
            return
        if dt > 0:
            alpha = min(1.0, dt / self._tau)
            target = self._cmd if self._armed else [0.0, 0.0, 0.0]
            for i in range(3):
                self._vel[i] += alpha * (target[i] - self._vel[i])
                self._pos[i] += self._vel[i] * dt
        self._state = VehicleState(
            t_s=now - self._t0,
            position_ned=tuple(self._pos),
            velocity_ned=tuple(self._vel),
            roll_deg=0.0,
            pitch_deg=0.0,
            yaw_deg=0.0,
            armed=self._armed,
        )

    def arm(self) -> None:
        self.commands_received.append(("arm",))
        self._armed = True

    def disarm(self) -> None:
        self.commands_received.append(("disarm",))
        self._armed = False

    def set_velocity(self, vn: float, ve: float, vd: float) -> None:
        self.commands_received.append(("velocity", vn, ve, vd))
        self._cmd = [vn, ve, vd]

    def link_health(self) -> LinkHealth:
        alive = self._connected and self._state is not None
        return LinkHealth(self._connected, alive, 0.0, 0.0 if alive else None)
