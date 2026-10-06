"""Fault injectors for a sensor message stream.

``process`` takes one measurement as the sensor produced it and returns what
the consumer actually receives: nothing (dropped), the value late (delayed),
with a perturbed stamp (jitter), with a spike (corrupted), or a stale value
that still looks fresh (frozen). All randomness comes from one seeded
generator, so a scenario replays identically.
"""

from __future__ import annotations

import random
from collections import Counter
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .schema import FaultSpec

SENSOR_FAULTS = {
    "vo_dropout",
    "dropped_messages",
    "delayed_messages",
    "timestamp_jitter",
    "corrupted_measurement",
    "frozen_sensor",
}


@dataclass(frozen=True)
class Delivery:
    deliver_t: float
    stamp_s: float
    value: np.ndarray
    corrupted: bool = False


class SensorFaultInjector:
    def __init__(self, faults: Sequence[FaultSpec], seed: int = 1) -> None:
        # Per-fault effect counters: evidence that the fault actually fired.
        self.counters: Counter[str] = Counter()
        self._faults = [f for f in faults if f.type in SENSOR_FAULTS or f.type == "estimator_reset"]
        self._rng = random.Random(seed)
        self._held: dict[tuple[int, str], np.ndarray] = {}
        self._last_stamp: dict[str, tuple[float, float]] = {}  # sensor -> (true stamp, jittered stamp)
        self._reset_done: set[int] = set()

    @staticmethod
    def _active(fault: FaultSpec, t: float) -> bool:
        return fault.start_s <= t < fault.end_s

    @staticmethod
    def _targets(fault: FaultSpec, sensor: str) -> bool:
        if fault.type == "vo_dropout":
            return sensor == "vo"
        return fault.params.get("sensor", "vo") == sensor

    def process(self, sensor: str, t: float, stamp_s: float, value: np.ndarray) -> list[Delivery]:
        value = np.array(value, dtype=float)
        deliver_t = t
        stamp = stamp_s
        corrupted = False
        for index, fault in enumerate(self._faults):
            if fault.type == "estimator_reset" or not self._active(fault, t) or not self._targets(fault, sensor):
                continue
            self.counters["in_window"] += 1
            if fault.type == "vo_dropout":
                self.counters["dropped"] += 1
                return []
            if fault.type == "dropped_messages":
                if self._rng.random() < fault.params["probability"]:
                    self.counters["dropped"] += 1
                    return []
            elif fault.type == "delayed_messages":
                deliver_t += fault.params["delay_s"]
                self.counters["delayed"] += 1
            elif fault.type == "timestamp_jitter":
                stamp += self._rng.gauss(0.0, fault.params["std_s"])
                self.counters["jittered"] += 1
            elif fault.type == "corrupted_measurement":
                if self._rng.random() < fault.params["probability"]:
                    # An IMU sample is [gyro(3), accel(3)]: spike the accelerometer only
                    # (a 20 rad/s gyro spike is not a physically meaningful fault).
                    lo = 3 if len(value) == 6 else 0
                    direction = np.array([self._rng.gauss(0, 1) for _ in range(len(value) - lo)])
                    direction /= np.linalg.norm(direction) or 1.0
                    value[lo:] = value[lo:] + direction * fault.params["magnitude"]
                    corrupted = True
                    self.counters["corrupted"] += 1
            elif fault.type == "frozen_sensor":
                key = (index, sensor)
                value = self._held.setdefault(key, value).copy()
                self.counters["frozen"] += 1
        # A VO velocity is displacement / stamp interval, so jittered stamps scale it.
        if sensor == "vo" and stamp != stamp_s:
            prev = self._last_stamp.get(sensor)
            if prev is not None:
                true_dt, seen_dt = stamp_s - prev[0], stamp - prev[1]
                if true_dt > 0 and seen_dt > 1e-3:
                    value = value * (true_dt / seen_dt)
        self._last_stamp[sensor] = (stamp_s, stamp)
        return [Delivery(deliver_t, stamp, value, corrupted)]

    def resets_due(self, t_prev: float, t_now: float) -> int:
        due = 0
        for index, fault in enumerate(self._faults):
            if (
                fault.type == "estimator_reset"
                and index not in self._reset_done
                and t_prev < fault.start_s <= t_now
            ):
                self._reset_done.add(index)
                self.counters["resets"] += 1
                due += 1
        return due
