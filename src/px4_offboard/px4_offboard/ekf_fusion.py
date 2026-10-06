"""Sensor-fusion localization: real IMU + real stereo visual odometry.

Deliberately scoped as "lightweight but honest," not a full 15/21-state
error-state Kalman filter with jointly-estimated attitude covariance and
IMU biases:

- Position/velocity (6-state) are a real linear Kalman filter: IMU
  specific-force drives the predict step, stereo-VO-derived velocity
  drives the update step.
- Attitude is gyro integration corrected by stereo VO's relative rotation
  via a complementary-filter blend, tracked *outside* the KF covariance.
  This is a known, real simplification (no formal position<->attitude
  covariance coupling) — not hidden: see ``PoseVelocityEKF`` docstring.

Nothing here reads simulator ground truth. Inputs are exactly what a real
vehicle would have: body-frame gyro/accelerometer (PX4 ``SensorCombined``)
and frame-to-frame stereo visual odometry (``stereo_depth.py``).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# NED convention (matches mission_logic.py / offboard_mission.py elsewhere in
# this package): Down is positive, so gravity accelerates objects toward
# +Down. Strapdown INS mechanization: a_true_world = R(q) @ f_body + g_world,
# where f_body is the accelerometer's specific-force reading and g_world =
# (0, 0, +G). At rest and level (R = I), a_true = 0, so f_body = -g_world =
# (0, 0, -G) -- the accelerometer reads negative-Down (i.e. "up") at rest,
# the reaction force against gravity. See test_ekf_fusion.py for a
# zero-drift-at-rest regression that would catch a sign error here.
GRAVITY_MPS2 = 9.80665
GRAVITY_WORLD = np.array([0.0, 0.0, GRAVITY_MPS2])

# Exact-repeat detection only applies above this speed (see update_velocity).
STUCK_MIN_SPEED_MPS = 0.2


def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product, scalar-first [w, x, y, z]."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def quat_from_rotvec(rotvec: np.ndarray) -> np.ndarray:
    """Small-angle-safe axis-angle -> quaternion (scalar-first)."""
    angle = np.linalg.norm(rotvec)
    if angle < 1e-9:
        return np.array([1.0, *(0.5 * rotvec)])
    axis = rotvec / angle
    half = angle / 2.0
    return np.array([math.cos(half), *(axis * math.sin(half))])


def quat_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = quat_normalize(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rotation_matrix_to_quat(r: np.ndarray) -> np.ndarray:
    """Shepperd's method — numerically stable for any rotation matrix."""
    trace = np.trace(r)
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (r[2, 1] - r[1, 2]) * s
        y = (r[0, 2] - r[2, 0]) * s
        z = (r[1, 0] - r[0, 1]) * s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return quat_normalize(np.array([w, x, y, z]))


def quat_slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    q0, q1 = quat_normalize(q0), quat_normalize(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1, dot = -q1, -dot
    if dot > 0.9995:
        return quat_normalize(q0 + t * (q1 - q0))
    theta0 = math.acos(min(1.0, max(-1.0, dot)))
    theta = theta0 * t
    q2 = quat_normalize(q1 - q0 * dot)
    return quat_normalize(q0 * math.cos(theta) + q2 * math.sin(theta))


def message_dt_s(
    previous_us: int | None, current_us: int, max_dt_s: float
) -> float | None:
    """Integration interval from consecutive message timestamps (microseconds).

    Uses the vehicle's own clock rather than when a Python callback happened
    to run: callbacks are batched under load and the simulator does not run at
    exactly real time, and both distort a wall-clock dt. Returns None for the
    first message, a non-increasing timestamp, or a gap longer than
    ``max_dt_s`` (a stall -- better to skip than to integrate across it).
    """
    if previous_us is None or current_us <= previous_us:
        return None
    dt = (current_us - previous_us) * 1e-6
    return dt if dt <= max_dt_s else None


def fusion_healthy(
    now: float,
    last_vo_t: float | None,
    last_imu_t: float | None,
    stale_timeout_s: float,
) -> bool:
    """"Healthy" means VO is currently *contributing* (an accepted update
    within the timeout -- rejected measurements must not keep the estimate
    looking fresh) and the IMU is still arriving. Shared by the ROS node and
    the offline fault-injection rig so both judge health identically."""
    return (
        last_vo_t is not None
        and now - last_vo_t <= stale_timeout_s
        and last_imu_t is not None
        and now - last_imu_t <= stale_timeout_s
    )


@dataclass
class FusionState:
    position_m: np.ndarray  # (3,) NED, metres
    velocity_mps: np.ndarray  # (3,) NED, m/s
    quat_wxyz: np.ndarray  # body-to-world orientation
    position_cov: np.ndarray  # (6,6) covariance over [position; velocity]


class PoseVelocityEKF:
    """See module docstring for the position/velocity-vs-attitude split.

    ``accel_noise_std`` is the process noise on acceleration, and it must
    cover more than raw accelerometer noise: attitude is tracked outside this
    filter's covariance, so every degree of attitude error leaks gravity into
    the horizontal acceleration (``g * sin(error)`` -- 0.85 m/s^2 for 5 degrees).
    The original 0.05 m/s^2 ignored that, which made the filter overconfident
    in its own dead reckoning and deaf to VO; live flight showed 39 m/s of
    velocity while VO reported ~2. ``test_realistic_attitude_error_*`` pins it.

    Velocity updates are gated on the Mahalanobis distance of the innovation
    and on an absolute speed bound: stereo VO produces occasional absurd
    outputs (hundreds of m/s on degenerate scenes), and one such update
    accepted at face value ruins the state.
    """

    def __init__(
        self,
        initial_position_m: np.ndarray | None = None,
        initial_quat_wxyz: np.ndarray | None = None,
        accel_noise_std: float = 0.5,
        vo_attitude_blend: float = 0.15,
        velocity_gate_chi2: float = 16.27,  # 99.9% of chi-square, 3 dof
        max_speed_mps: float = 15.0,
        attitude_gate_deg: float = 1.5,
        reference_attitude_tau_s: float = 0.0,
    ):
        self.position = (
            np.zeros(3) if initial_position_m is None else np.array(initial_position_m, dtype=float)
        )
        self.velocity = np.zeros(3)
        self.quat = (
            np.array([1.0, 0.0, 0.0, 0.0])
            if initial_quat_wxyz is None
            else quat_normalize(np.array(initial_quat_wxyz, dtype=float))
        )
        self.cov = np.eye(6) * 0.1
        self.accel_noise_std = accel_noise_std
        self.vo_attitude_blend = vo_attitude_blend
        self.reference_attitude_tau_s = reference_attitude_tau_s
        self.velocity_gate_chi2 = velocity_gate_chi2
        self.max_speed_mps = max_speed_mps
        self.attitude_gate_rad = math.radians(attitude_gate_deg)
        self.accepted_updates = 0
        self.rejected_updates = 0
        self.stuck_rejections = 0
        self._last_velocity_measurement: np.ndarray | None = None
        self._consecutive_rejections = 0
        # Attitude at the time of the last VO correction (or construction,
        # if none yet) -- the frame relative_rotation is measured against.
        self._vo_reference_quat = self.quat.copy()
        self._initialized = False

    def initialize(
        self,
        position_m: np.ndarray,
        quat_wxyz: np.ndarray,
        velocity_mps: np.ndarray | None = None,
    ) -> None:
        """Seed the state from the vehicle's own estimate (what a real
        vehicle has before VO has acquired). Velocity defaults to rest."""
        self.position = np.array(position_m, dtype=float)
        self.velocity = (
            np.zeros(3) if velocity_mps is None else np.array(velocity_mps, dtype=float)
        )
        self.quat = quat_normalize(np.array(quat_wxyz, dtype=float))
        self._vo_reference_quat = self.quat.copy()
        self.cov = np.eye(6) * 0.1

    def predict(self, gyro_rad_s: np.ndarray, accel_mps2: np.ndarray, dt_s: float) -> None:
        if dt_s <= 0.0:
            return
        gyro = np.asarray(gyro_rad_s, dtype=float)
        accel = np.asarray(accel_mps2, dtype=float)

        # Attitude: pure gyro integration (corrected later by VO, see
        # apply_vo_attitude_correction).
        dq = quat_from_rotvec(gyro * dt_s)
        self.quat = quat_normalize(quat_multiply(self.quat, dq))

        # Position/velocity: strapdown mechanization (see GRAVITY_WORLD
        # docstring for the sign convention).
        r = quat_to_rotation_matrix(self.quat)
        accel_world = r @ accel + GRAVITY_WORLD
        self.position = self.position + self.velocity * dt_s + 0.5 * accel_world * dt_s**2
        self.velocity = self.velocity + accel_world * dt_s

        # Constant-velocity process model (F) with accelerometer-noise-driven
        # process noise (Q) on velocity, propagated into position too.
        f = np.eye(6)
        f[0:3, 3:6] = np.eye(3) * dt_s
        q_accel = (self.accel_noise_std**2) * dt_s
        q = np.zeros((6, 6))
        q[0:3, 0:3] = np.eye(3) * q_accel * dt_s**2 / 3.0
        q[0:3, 3:6] = np.eye(3) * q_accel * dt_s / 2.0
        q[3:6, 0:3] = q[0:3, 3:6]
        q[3:6, 3:6] = np.eye(3) * q_accel
        self.cov = f @ self.cov @ f.T + q
        self._initialized = True

    def update_velocity(
        self, measured_velocity_world_mps: np.ndarray, measurement_std: float
    ) -> bool:
        """Fuse a stereo-VO-derived world-frame velocity. Returns whether it
        was accepted; a rejected measurement leaves the state untouched."""
        z = np.asarray(measured_velocity_world_mps, dtype=float)
        if not np.all(np.isfinite(z)) or np.linalg.norm(z) > self.max_speed_mps:
            return self._reject()
        # A real VO velocity carries float noise and never repeats bit-for-bit
        # while moving; an exact repeat is a frozen/stale sensor. Near-zero
        # repeats are normal (VO reports exactly 0 at rest) and are exempt.
        last = self._last_velocity_measurement
        self._last_velocity_measurement = z.copy()
        if (
            last is not None
            and np.array_equal(z, last)
            and np.linalg.norm(z) > STUCK_MIN_SPEED_MPS
        ):
            self.stuck_rejections += 1
            return self._reject()
        h = np.zeros((3, 6))
        h[:, 3:6] = np.eye(3)
        r_meas = np.eye(3) * measurement_std**2

        y = z - h @ np.concatenate([self.position, self.velocity])
        s = h @ self.cov @ h.T + r_meas
        if float(y @ np.linalg.solve(s, y)) > self.velocity_gate_chi2:
            return self._reject()
        k = self.cov @ h.T @ np.linalg.inv(s)
        state = np.concatenate([self.position, self.velocity]) + k @ y
        self.position, self.velocity = state[0:3], state[3:6]
        self.cov = (np.eye(6) - k @ h) @ self.cov
        self.accepted_updates += 1
        self._consecutive_rejections = 0
        return True

    def _reject(self) -> bool:
        self.rejected_updates += 1
        self._consecutive_rejections += 1
        # Persistent rejection means the *filter* has drifted away from a
        # consistent VO stream, not that every measurement is bad: open up the
        # velocity covariance so the next consistent measurement re-anchors it
        # instead of locking the filter out forever.
        if self._consecutive_rejections >= 8:
            self.cov[3:6, 3:6] += np.eye(3) * 4.0
            self._consecutive_rejections = 0
        return False

    def apply_vo_attitude_correction(self, relative_rotation: np.ndarray) -> bool:
        """Blend the gyro-integrated attitude toward what VO's relative
        rotation implies, at ``vo_attitude_blend`` weight. ``relative_rotation``
        is the body-frame rotation since the *previous* VO update.

        The correction is only applied when VO's rotation agrees with the
        gyro-integrated rotation over the same interval (within
        ``attitude_gate_deg``). Short-interval gyro integration is far more
        accurate than frame-to-frame PnP rotation, so a large disagreement
        means a bad VO frame, and blending it in would rotate every later
        velocity into the wrong direction. Returns whether it was applied.
        """
        dq_vo = rotation_matrix_to_quat(relative_rotation)
        ref = self._vo_reference_quat
        ref_conj = np.array([ref[0], -ref[1], -ref[2], -ref[3]])
        dq_gyro = quat_multiply(ref_conj, self.quat)
        disagreement = 2.0 * math.acos(min(1.0, abs(float(np.dot(dq_vo, dq_gyro)))))
        if disagreement > self.attitude_gate_rad:
            self._vo_reference_quat = self.quat.copy()
            return False
        q_vo_implied = quat_normalize(quat_multiply(ref, dq_vo))
        self.quat = quat_slerp(self.quat, q_vo_implied, self.vo_attitude_blend)
        self._vo_reference_quat = self.quat.copy()
        return True

    def apply_reference_attitude(self, quat_wxyz: np.ndarray, dt_s: float) -> None:
        """Complementary pull of the gyro-integrated attitude toward an
        external attitude estimate (the autopilot's IMU+magnetometer AHRS --
        not ground truth), with time constant ``reference_attitude_tau_s``.

        Raw gyro integration has no bias or scale correction, and it
        accumulated up to 17 degrees of yaw error through fast turns in live
        flights (BUG-017 follow-up); every later VO velocity was then rotated
        into the wrong direction. Short-term the gyro still dominates, so the
        fast dynamics stay smooth. Position and velocity remain IMU + VO only.
        """
        if self.reference_attitude_tau_s <= 0.0 or dt_s <= 0.0:
            return
        alpha = min(1.0, dt_s / self.reference_attitude_tau_s)
        self.quat = quat_normalize(
            quat_slerp(self.quat, quat_normalize(np.asarray(quat_wxyz, dtype=float)), alpha)
        )
        self._vo_reference_quat = self.quat.copy()

    def state(self) -> FusionState:
        return FusionState(
            position_m=self.position.copy(),
            velocity_mps=self.velocity.copy(),
            quat_wxyz=self.quat.copy(),
            position_cov=self.cov.copy(),
        )


class DriftTracker:
    """Fused-estimate error against a reference estimate, normalized by the
    distance the reference actually travelled.

    Drift is only meaningful relative to distance flown (a stationary
    vehicle that "drifts" 1 m is a different failure than a 100 m flight
    that ends 1 m off), so this reports both the absolute error and the
    error as a fraction of cumulative reference path length.
    """

    def __init__(self) -> None:
        self.path_length_m = 0.0
        self._last_ref: np.ndarray | None = None

    def update(
        self, fused_position_m: np.ndarray, reference_position_m: np.ndarray
    ) -> dict[str, float]:
        fused = np.asarray(fused_position_m, dtype=float)
        ref = np.asarray(reference_position_m, dtype=float)
        if self._last_ref is not None:
            self.path_length_m += float(np.linalg.norm(ref - self._last_ref))
        self._last_ref = ref.copy()
        err = fused - ref
        horiz = float(np.hypot(err[0], err[1]))
        return {
            "err_horiz_m": horiz,
            "err_down_m": float(err[2]),
            "path_length_m": self.path_length_m,
            "drift_frac": horiz / self.path_length_m if self.path_length_m > 1.0 else 0.0,
        }
