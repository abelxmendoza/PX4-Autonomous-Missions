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


@dataclass
class FusionState:
    position_m: np.ndarray  # (3,) NED, metres
    velocity_mps: np.ndarray  # (3,) NED, m/s
    quat_wxyz: np.ndarray  # body-to-world orientation
    position_cov: np.ndarray  # (6,6) covariance over [position; velocity]


class PoseVelocityEKF:
    """See module docstring for the position/velocity-vs-attitude split.

    accel_noise_std / vo_velocity_noise_std are process/measurement noise
    tuning knobs, not free parameters chosen to make a demo pass — they
    should reflect the actual simulated IMU noise and the actual observed
    scatter of stereo-VO velocity estimates. Defaults here are reasonable
    starting points for a simulated MEMS-class IMU and a ~30Hz VO update,
    not a calibrated result.
    """

    def __init__(
        self,
        initial_position_m: np.ndarray | None = None,
        initial_quat_wxyz: np.ndarray | None = None,
        accel_noise_std: float = 0.05,
        vo_attitude_blend: float = 0.15,
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
        # Attitude at the time of the last VO correction (or construction,
        # if none yet) -- the frame relative_rotation is measured against.
        self._vo_reference_quat = self.quat.copy()
        self._initialized = False

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
    ) -> None:
        """Fuse a stereo-VO-derived world-frame velocity measurement."""
        z = np.asarray(measured_velocity_world_mps, dtype=float)
        h = np.zeros((3, 6))
        h[:, 3:6] = np.eye(3)
        r_meas = np.eye(3) * measurement_std**2

        y = z - h @ np.concatenate([self.position, self.velocity])
        s = h @ self.cov @ h.T + r_meas
        k = self.cov @ h.T @ np.linalg.inv(s)
        state = np.concatenate([self.position, self.velocity]) + k @ y
        self.position, self.velocity = state[0:3], state[3:6]
        self.cov = (np.eye(6) - k @ h) @ self.cov

    def apply_vo_attitude_correction(self, relative_rotation: np.ndarray) -> None:
        """Blend the gyro-integrated attitude toward what VO's relative
        rotation implies, at ``vo_attitude_blend`` weight. ``relative_rotation``
        is the camera-frame rotation since the *previous* VO update, in the
        camera's own prior frame -- the same body frame the gyro operates
        in, since the camera is rigidly mounted to the airframe.
        """
        dq_vo = rotation_matrix_to_quat(relative_rotation)
        q_vo_implied = quat_normalize(quat_multiply(self._vo_reference_quat, dq_vo))
        self.quat = quat_slerp(self.quat, q_vo_implied, self.vo_attitude_blend)
        self._vo_reference_quat = self.quat.copy()

    def state(self) -> FusionState:
        return FusionState(
            position_m=self.position.copy(),
            velocity_mps=self.velocity.copy(),
            quat_wxyz=self.quat.copy(),
            position_cov=self.cov.copy(),
        )
