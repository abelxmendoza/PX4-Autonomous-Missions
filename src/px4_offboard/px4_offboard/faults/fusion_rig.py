"""Deterministic offline rig for the stereo+IMU fusion estimator.

A synthetic flight (smooth heading changes at constant speed) produces IMU
samples (with bias and noise) and VO world-velocity measurements (with noise).
They pass through a :class:`SensorFaultInjector` and into the *real*
:class:`PoseVelocityEKF` with the same acceptance and health rules the ROS node
uses. Truth is known exactly, so error is measured against truth -- unlike the
SITL flights, where only PX4's own estimate is available.

What this does NOT model: image processing (VO failures are injected at the
measurement, not produced by feature tracking), IMU saturation, or
attitude-estimation error in the VO frame conversion.
"""

from __future__ import annotations

import heapq
import math
import random
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from px4_offboard.ekf_fusion import (
    GRAVITY_WORLD,
    PoseVelocityEKF,
    fusion_healthy,
    message_dt_s,
)

from .injectors import SensorFaultInjector
from .schema import FaultSpec


@dataclass(frozen=True)
class RigConfig:
    duration_s: float = 60.0
    seed: int = 1
    imu_hz: float = 100.0
    vo_hz: float = 5.0
    sample_dt_s: float = 0.1
    speed_mps: float = 2.0
    accel_noise: float = 0.05
    accel_bias: tuple[float, float, float] = (0.05, -0.03, 0.02)
    gyro_noise: float = 0.002
    gyro_bias_z: float = 0.001
    vo_noise_mps: float = 0.05
    vo_std_mps: float = 0.15  # what the filter is told (matches the node default)
    stale_timeout_s: float = 0.5


@dataclass(frozen=True)
class RigSample:
    t: float
    err_horiz_m: float
    err_down_m: float
    vel_err_mps: float
    healthy: bool


@dataclass
class RigTrace:
    config: RigConfig
    samples: list[RigSample] = field(default_factory=list)
    vo_accepted: int = 0
    vo_rejected: int = 0
    vo_delivered: int = 0


def _truth(t: float, speed: float):
    """(velocity, acceleration, yaw, yaw_rate) of the synthetic flight.

    Speed oscillates around ``speed`` (+-0.75 m/s^2 peak acceleration) and the
    heading swings +-46 degrees, so latency, jitter and frozen-sensor faults
    have something to corrupt -- a constant-velocity cruise would hide them.
    """
    yaw = 0.8 * math.sin(0.3 * t)
    yaw_rate = 0.24 * math.cos(0.3 * t)
    spd = speed + 1.25 * math.sin(0.6 * t)
    spd_rate = 0.75 * math.cos(0.6 * t)
    vd = 0.2 * math.sin(0.2 * t)
    c, s = math.cos(yaw), math.sin(yaw)
    vel = np.array([spd * c, spd * s, vd])
    acc = np.array(
        [spd_rate * c - spd * s * yaw_rate, spd_rate * s + spd * c * yaw_rate, 0.04 * math.cos(0.2 * t)]
    )
    return vel, acc, yaw, yaw_rate


def _yaw_quat(yaw: float) -> np.ndarray:
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def _rz(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class FusionRig:
    def __init__(self, config: RigConfig | None = None) -> None:
        self.config = config or RigConfig()
        self.last_counters: dict[str, int] = {}

    def run(self, faults: Sequence[FaultSpec]) -> RigTrace:
        cfg = self.config
        rng = random.Random(cfg.seed * 7919 + 17)  # sensor noise stream
        injector = SensorFaultInjector(faults, seed=cfg.seed)
        trace = RigTrace(cfg)

        vel0, _, yaw0, _ = _truth(0.0, cfg.speed_mps)
        ekf = PoseVelocityEKF(accel_noise_std=0.5, vo_attitude_blend=0.0)
        truth_pos = np.zeros(3)
        ekf.initialize(truth_pos, _yaw_quat(yaw0), vel0)

        dt = 1.0 / cfg.imu_hz
        vo_period = 1.0 / cfg.vo_hz
        sample_period = cfg.sample_dt_s
        steps = int(round(cfg.duration_s * cfg.imu_hz))
        next_vo, next_sample = 0.0, 0.0
        last_vo_t = last_imu_t = None
        prev_imu_stamp_us: int | None = None
        queue: list[tuple[float, int, str, float, np.ndarray]] = []
        order = 0
        prev_vel = vel0
        t_prev = 0.0
        bias = np.array(cfg.accel_bias)

        for k in range(steps + 1):
            t = k * dt
            vel, acc, yaw, yaw_rate = _truth(t, cfg.speed_mps)
            if k:
                truth_pos = truth_pos + 0.5 * (vel + prev_vel) * dt
            prev_vel = vel

            for _ in range(injector.resets_due(t_prev, t)):
                ekf.initialize(ekf.position, ekf.quat, np.zeros(3))
            t_prev = t

            # --- sensor side: what the hardware produced at time t ----------
            f_body = _rz(yaw).T @ (acc - GRAVITY_WORLD)
            gyro = np.array([0.0, 0.0, yaw_rate + cfg.gyro_bias_z]) + np.array(
                [rng.gauss(0, cfg.gyro_noise) for _ in range(3)]
            )
            accel = f_body + bias + np.array([rng.gauss(0, cfg.accel_noise) for _ in range(3)])
            for d in injector.process("imu", t, t, np.concatenate([gyro, accel])):
                heapq.heappush(queue, (d.deliver_t, order, "imu", d.stamp_s, d.value))
                order += 1
            if t + 1e-9 >= next_vo:
                next_vo += vo_period
                measured = vel + np.array([rng.gauss(0, cfg.vo_noise_mps) for _ in range(3)])
                for d in injector.process("vo", t, t, measured):
                    heapq.heappush(queue, (d.deliver_t, order, "vo", d.stamp_s, d.value))
                    order += 1

            # --- consumer side: what the estimator receives by time t -------
            while queue and queue[0][0] <= t + 1e-9:
                deliver_t, _, kind, stamp_s, value = heapq.heappop(queue)
                if kind == "imu":
                    stamp_us = int(round(stamp_s * 1e6))
                    step = message_dt_s(prev_imu_stamp_us, stamp_us, cfg.stale_timeout_s)
                    prev_imu_stamp_us = stamp_us
                    last_imu_t = deliver_t
                    if step is not None:
                        ekf.predict(value[:3], value[3:], step)
                else:
                    trace.vo_delivered += 1
                    if ekf.update_velocity(value, cfg.vo_std_mps):
                        trace.vo_accepted += 1
                        last_vo_t = deliver_t
                    else:
                        trace.vo_rejected += 1

            if t + 1e-9 >= next_sample:
                next_sample += sample_period
                err = ekf.position - truth_pos
                trace.samples.append(
                    RigSample(
                        t=round(t, 6),
                        err_horiz_m=float(np.hypot(err[0], err[1])),
                        err_down_m=float(abs(err[2])),
                        vel_err_mps=float(np.linalg.norm(ekf.velocity - vel)),
                        healthy=fusion_healthy(t, last_vo_t, last_imu_t, cfg.stale_timeout_s),
                    )
                )
        self.last_counters = dict(injector.counters)
        return trace
