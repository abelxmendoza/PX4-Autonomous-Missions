"""Digital twin of a motor/propeller thrust-test bench.

Not a Pixhawk or flight-controller model: there is no attitude estimator,
mixer, or MAVLink in this module. It is the kind of bench a test engineer
would build to characterize a motor/ESC/propeller combination on the ground --
command in, RPM/thrust/load-cell/battery/temperature out -- which is why it
is modeled and exposed separately from the PX4/MAVLink HIL path in
vehicle/ and comms/ (see docs/HIL_ARCHITECTURE.md).

What this does NOT model: motor electrical dynamics (current, back-EMF),
real aerodynamics (induced drag, prop stall), vibration, or a validated fit
to any physical rig. The RPM lag, thrust curve and thermal/battery response
are plausible first-order approximations tuned for believable magnitudes,
not measured from hardware.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field

FAULT_NAMES = ("load_cell_dropout", "frozen_temp_sensor", "voltage_brownout")


@dataclass(frozen=True)
class RigConfig:
    dt_s: float = 0.05
    seed: int = 1
    max_rpm: float = 9000.0
    rpm_tau_s: float = 0.35  # first-order lag from commanded to actual RPM
    thrust_k: float = 0.37  # thrust_n = thrust_k * (rpm / 1000) ** 2
    volt0_v: float = 16.8  # nominal 4S pack, unloaded
    volt_sag_k: float = 2.0  # volts of sag at 100% load fraction
    voltage_noise_v: float = 0.03
    temp0_c: float = 25.0
    temp_rise_k: float = 55.0  # steady-state temperature rise at 100% load
    temp_tau_s: float = 25.0
    temp_noise_c: float = 0.1
    load_cell_noise_n: float = 0.03


@dataclass(frozen=True)
class FaultWindow:
    name: str
    start_s: float
    duration_s: float
    params: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.name not in FAULT_NAMES:
            raise ValueError(f"unknown bench fault type: {self.name!r}")
        if self.duration_s <= 0:
            raise ValueError("duration_s must be positive")


@dataclass(frozen=True)
class RigSample:
    t: float
    command_pct: float
    rpm: float
    thrust_n: float
    load_cell_n: float
    voltage_v: float
    temp_c: float
    fault_active: str  # "" when none


def sample_to_dict(s: RigSample) -> dict:
    return {
        "t": s.t,
        "command_pct": s.command_pct,
        "rpm": round(s.rpm, 2),
        "thrust_n": round(s.thrust_n, 4),
        "load_cell_n": round(s.load_cell_n, 4),
        "voltage_v": round(s.voltage_v, 3),
        "temp_c": round(s.temp_c, 2),
        "fault_active": s.fault_active,
    }


class VirtualTestRig:
    """Deterministic (seeded) stand-in for a physical motor/thrust bench."""

    def __init__(self, config: RigConfig = RigConfig()) -> None:
        self.config = config
        self._rng = random.Random(config.seed)
        self._rpm = 0.0
        self._temp_c = config.temp0_c
        self._step_count = 0  # time is derived from this, not accumulated by
        # repeated float addition, so a fault window boundary (e.g. exactly
        # start_s=1.0 at dt_s=0.1) can't drift onto the wrong sample.
        self._faults: list[FaultWindow] = []
        self._frozen_temp_latch: float | None = None

    def add_fault(self, fault: FaultWindow) -> None:
        self._faults.append(fault)

    @property
    def _t(self) -> float:
        return round(self._step_count * self.config.dt_s, 6)

    def _active_fault(self) -> FaultWindow | None:
        t = self._t
        for f in self._faults:
            if f.start_s <= t < f.start_s + f.duration_s:
                return f
        return None

    def step(self, command_pct: float) -> RigSample:
        cfg = self.config
        command_pct = max(0.0, min(100.0, command_pct))

        target_rpm = command_pct / 100.0 * cfg.max_rpm
        rpm_alpha = cfg.dt_s / (cfg.rpm_tau_s + cfg.dt_s)
        self._rpm += (target_rpm - self._rpm) * rpm_alpha
        thrust_n = cfg.thrust_k * (self._rpm / 1000.0) ** 2

        load_frac = self._rpm / cfg.max_rpm if cfg.max_rpm else 0.0
        voltage_true = cfg.volt0_v - cfg.volt_sag_k * load_frac

        temp_target = cfg.temp0_c + cfg.temp_rise_k * load_frac
        temp_alpha = cfg.dt_s / (cfg.temp_tau_s + cfg.dt_s)
        self._temp_c += (temp_target - self._temp_c) * temp_alpha

        load_cell_reading = thrust_n + self._rng.gauss(0.0, cfg.load_cell_noise_n)
        voltage_reading = voltage_true + self._rng.gauss(0.0, cfg.voltage_noise_v)
        temp_reading = self._temp_c + self._rng.gauss(0.0, cfg.temp_noise_c)

        fault = self._active_fault()
        fault_name = fault.name if fault else ""

        if fault_name == "load_cell_dropout":
            load_cell_reading = 0.0

        if fault_name == "frozen_temp_sensor":
            if self._frozen_temp_latch is None:
                self._frozen_temp_latch = temp_reading
            temp_reading = self._frozen_temp_latch
        else:
            self._frozen_temp_latch = None

        if fault_name == "voltage_brownout":
            voltage_reading = float(fault.params.get("voltage_v", 9.0)) if fault else voltage_reading

        sample = RigSample(
            t=self._t,
            command_pct=command_pct,
            rpm=self._rpm,
            thrust_n=thrust_n,
            load_cell_n=load_cell_reading,
            voltage_v=voltage_reading,
            temp_c=temp_reading,
            fault_active=fault_name,
        )
        self._step_count += 1
        return sample
