"""Lightweight 2-D LiDAR SLAM: an occupancy-grid map plus scan-to-map pose correction.

What it is: each scan is first placed with odometry (in flight, PX4's own estimate), then
its pose is refined by a correlative search for the small shift and rotation at which the
scan's returns best line up with the map built so far (a likelihood field around occupied
cells), and only then is it added to the map. The same idea as Hector SLAM's scan-to-map
matching, written small enough to read and test.

What it is not: there is no loop closure or pose graph, so error that the matcher cannot
see (a long featureless corridor, a scan with too few returns) still accumulates; it is a
horizontal 2-D map at the flight altitude, not 3-D; and it assumes the vehicle is roughly
level, so a scan is a horizontal slice.

Conventions: poses are (north, east, yaw) in NED metres/radians; beam angles are relative to
the heading, positive to the right; an out-of-range beam is +inf (free space, no return) and
a NaN beam is ignored entirely (see ``level_scan``).
"""

from __future__ import annotations

import math
from typing import Sequence

import cv2
import numpy as np

Pose = tuple[float, float, float]


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(a: Pose, b: Pose) -> Pose:
    """Pose b, given in a's frame (forward, right, dyaw), expressed in the world."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], _wrap(a[2] + b[2]))


def relative(a: Pose, b: Pose) -> Pose:
    """b in a's frame: the inverse of compose."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dn, de = b[0] - a[0], b[1] - a[1]
    return (c * dn + s * de, -s * dn + c * de, _wrap(b[2] - a[2]))


def level_scan(angles: np.ndarray, ranges: np.ndarray, roll: float, pitch: float, height_m: float,
               min_hit_height_m: float = 0.3, min_range_m: float = 0.6) -> tuple[np.ndarray, np.ndarray]:
    """Project a scan taken by a tilted drone onto the horizontal plane.

    A 2-D LiDAR on a multicopter tilts with it: while it accelerates, beams on one side dip
    and can hit the ground, which would map the ground as a wall. Each return is rotated by
    roll and pitch (body FRD to a level frame), and a return whose point lies less than
    ``min_hit_height_m`` above the ground (``height_m`` is the sensor's height) becomes NaN,
    as does anything nearer than ``min_range_m`` (the drone's own frame). Returns the
    horizontal bearings and ranges of the remaining points; no-return beams stay +inf.
    """
    a = np.asarray(angles, dtype=float)
    r = np.asarray(ranges, dtype=float)
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    x, y = np.cos(a), np.sin(a)                 # unit beam in body FRD (z = 0)
    # level = R_pitch @ R_roll @ body  (FRD: x fwd, y right, z down)
    lx = cp * x + sp * sr * y
    ly = cr * y
    lz = -sp * x + cp * sr * y
    horiz = np.hypot(lx, ly)
    bearing = np.arctan2(ly, lx)
    finite = np.isfinite(r)
    rf = np.where(finite, r, 0.0)  # keep inf and NaN out of the arithmetic
    out = np.where(finite, rf * horiz, np.inf)
    drop = finite & ((height_m - rf * lz < min_hit_height_m) | (rf < min_range_m))
    out[drop | np.isnan(r)] = np.nan
    return bearing, out


class OccupancyGrid:
    def __init__(self, resolution: float, north_min: float, north_max: float, east_min: float,
                 east_max: float, l_occ: float = 0.85, l_free: float = -0.4, l_limit: float = 4.0,
                 max_range: float = 30.0, no_return_clear_m: float | None = None) -> None:
        self.res = resolution
        self.north_min, self.east_min = north_min, east_min
        self.rows = int(math.ceil((north_max - north_min) / resolution))
        self.cols = int(math.ceil((east_max - east_min) / resolution))
        self.logodds = np.zeros((self.rows, self.cols), dtype=np.float32)
        self.l_occ, self.l_free, self.l_limit = l_occ, l_free, l_limit
        self.max_range = max_range
        # How far a beam with no return clears. A slightly upward-tilted beam can pass over a
        # low obstacle, so in flight this is kept shorter than the sensor's range.
        self.no_return_clear_m = max_range if no_return_clear_m is None else no_return_clear_m
        # Where inside each cell the returns actually landed (sum and count): the matcher
        # aligns scans to these sub-cell surface estimates, not to cell centres, which
        # would bias every pose by up to half a cell.
        self.hit_sum = np.zeros((self.rows * self.cols, 2))
        self.hit_count = np.zeros(self.rows * self.cols)
        self.field_subdiv = 4
        self.coarse_sigma_m, self.fine_sigma_m = 0.4, 0.15
        self.field_threshold = 0.65
        self._fields: dict[float, np.ndarray] = {}

    # --- coordinates ---------------------------------------------------------------
    def _cells(self, n: np.ndarray, e: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        i = np.floor((n - self.north_min) / self.res).astype(np.int64)
        j = np.floor((e - self.east_min) / self.res).astype(np.int64)
        ok = (i >= 0) & (i < self.rows) & (j >= 0) & (j < self.cols)
        return i, j, ok

    def probability_at(self, north: float, east: float) -> float:
        i, j, ok = self._cells(np.array([north]), np.array([east]))
        if not ok[0]:
            return 0.5
        return float(1.0 / (1.0 + math.exp(-self.logodds[i[0], j[0]])))

    def probabilities(self) -> np.ndarray:
        return 1.0 / (1.0 + np.exp(-self.logodds))

    def occupied_points(self, threshold: float = 0.7) -> list[tuple[float, float]]:
        ii, jj = np.nonzero(self.probabilities() > threshold)
        return [(self.north_min + (i + 0.5) * self.res, self.east_min + (j + 0.5) * self.res)
                for i, j in zip(ii, jj)]

    def occupied_count(self, threshold: float = 0.7) -> int:
        return int(np.count_nonzero(self.probabilities() > threshold))

    # --- mapping ---------------------------------------------------------------------
    def update(self, pose: Pose, angles: np.ndarray, ranges: np.ndarray) -> None:
        n0, e0, yaw = pose
        ranges = np.asarray(ranges, dtype=float)
        use = ~np.isnan(ranges)
        ranges, angles = ranges[use], np.asarray(angles, dtype=float)[use]
        hit = np.isfinite(ranges) & (ranges > 0.1)
        reach = np.where(hit, np.minimum(ranges, self.max_range), self.no_return_clear_m)
        bearing = yaw + angles
        cn, ce = np.cos(bearing), np.sin(bearing)

        # Free: sample each beam every half cell, stopping half a cell short of its return.
        step = self.res * 0.5
        t = np.arange(0.0, self.max_range, step)[None, :]
        along = t < (reach[:, None] - self.res * 0.5)
        fn = (n0 + cn[:, None] * t)[along]
        fe = (e0 + ce[:, None] * t)[along]
        fi, fj, fok = self._cells(fn, fe)
        free = np.unique(fi[fok] * self.cols + fj[fok])

        # Occupied: the cell each return lands in.
        hn, he = n0 + cn[hit] * ranges[hit], e0 + ce[hit] * ranges[hit]
        oi, oj, ook = self._cells(hn, he)
        hit_idx = oi[ook] * self.cols + oj[ook]
        np.add.at(self.hit_sum, hit_idx, np.column_stack([hn[ook], he[ook]]))
        np.add.at(self.hit_count, hit_idx, 1.0)
        occ = np.unique(hit_idx)
        free = np.setdiff1d(free, occ, assume_unique=True)

        flat = self.logodds.reshape(-1)
        flat[free] += self.l_free
        flat[occ] += self.l_occ
        np.clip(self.logodds, -self.l_limit, self.l_limit, out=self.logodds)
        self._fields = {}

    # --- localisation -------------------------------------------------------------------
    def likelihood_field(self, sigma_m: float = 0.2) -> np.ndarray:
        """exp(-d^2 / 2 sigma^2), d = distance to the nearest mapped surface point (metres).

        Sampled ``field_subdiv`` times finer than the map, with each occupied cell's
        surface at the mean of the returns that landed in it.
        """
        if sigma_m not in self._fields:
            k = self.field_subdiv
            occ = np.nonzero((self.probabilities() > self.field_threshold).reshape(-1) & (self.hit_count > 0))[0]
            surf = self.hit_sum[occ] / self.hit_count[occ, None]
            fi = np.floor((surf[:, 0] - self.north_min) / self.res * k).astype(np.int64)
            fj = np.floor((surf[:, 1] - self.east_min) / self.res * k).astype(np.int64)
            src = np.full((self.rows * k, self.cols * k), 255, dtype=np.uint8)
            src[np.clip(fi, 0, self.rows * k - 1), np.clip(fj, 0, self.cols * k - 1)] = 0
            d = cv2.distanceTransform(src, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * (self.res / k)
            self._fields[sigma_m] = np.exp(-(d ** 2) / (2 * sigma_m ** 2)).astype(np.float32)
        return self._fields[sigma_m]

    def _score(self, field: np.ndarray, pn: float, pe: float, yaw: np.ndarray,
               f: np.ndarray, r: np.ndarray, offsets: np.ndarray) -> np.ndarray:
        """Mean field value of the scan returns for every (yaw, offset) candidate."""
        c, s = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
        dn = c * f[None, :] - s * r[None, :]            # (yaws, points)
        de = s * f[None, :] + c * r[None, :]
        n = pn + offsets[None, :, 0, None] + dn[:, None, :]   # (yaws, offsets, points)
        e = pe + offsets[None, :, 1, None] + de[:, None, :]
        # Bilinear lookup between field samples: a nearest-sample lookup scores every pose
        # inside a sample the same, and that dead band lets error creep in unseen.
        fres = self.res / self.field_subdiv
        rows, cols = field.shape
        x = (n - self.north_min) / fres - 0.5
        y = (e - self.east_min) / fres - 0.5
        i0, j0 = np.floor(x).astype(np.int64), np.floor(y).astype(np.int64)
        fx, fy = (x - i0).astype(np.float32), (y - j0).astype(np.float32)
        ok = (i0 >= 0) & (i0 < rows - 1) & (j0 >= 0) & (j0 < cols - 1)
        i0, j0 = np.where(ok, i0, 0), np.where(ok, j0, 0)
        v = (field[i0, j0] * (1 - fx) * (1 - fy) + field[i0 + 1, j0] * fx * (1 - fy)
             + field[i0, j0 + 1] * (1 - fx) * fy + field[i0 + 1, j0 + 1] * fx * fy)
        return np.where(ok, v, 0.0).mean(axis=2)

    def match(self, guess: Pose, angles: np.ndarray, ranges: np.ndarray,
              window_m: float = 0.75, yaw_window_deg: float = 3.0, prior_weight: float = 0.01,
              prior_sigma_m: float = 0.5, prior_sigma_deg: float = 2.0) -> tuple[Pose, float]:
        """Refine ``guess`` so the scan best overlays the map; returns (pose, score in 0..1).

        Two stages. Coarse: the whole window on a blurred field (wide basins, so the
        right one is not stepped over) with every 4th return. Fine: around the coarse
        answer on the sharp field with every return. Both grids are symmetric and
        include their centre, so a correct guess is a candidate as it stands.

        Candidates are ranked by fit minus a small Gaussian motion prior around the guess,
        so where the scene is nearly symmetric (a long wall seen side-on) a slightly better
        fit far from the odometry does not win. The returned score is the fit alone.
        """
        hit = np.isfinite(ranges) & (ranges > 0.1)
        a, rr = np.asarray(angles)[hit], np.asarray(ranges)[hit]
        f, r = rr * np.cos(a), rr * np.sin(a)  # beam returns in the sensor frame (forward, right)
        stages = (  # (field sigma, point stride, xy half-width, xy step, yaw half-width, yaw step)
            (self.coarse_sigma_m, 4, window_m, self.res / 2, math.radians(yaw_window_deg), math.radians(0.25)),
            (self.fine_sigma_m, 1, self.res / 2, self.res / 8, math.radians(0.25), math.radians(0.05)),
        )
        centre, score = guess, 0.0
        for sigma, stride, win, step, ywin, ystep in stages:
            ticks = np.arange(-round(win / step), round(win / step) + 1) * step
            offsets = np.array([(dn, de) for dn in ticks for de in ticks])
            yaws = centre[2] + np.arange(-round(ywin / ystep), round(ywin / ystep) + 1) * ystep
            scores = self._score(self.likelihood_field(sigma), centre[0], centre[1], yaws,
                                 f[::stride], r[::stride], offsets)
            dxy2 = (centre[0] + offsets[:, 0] - guess[0]) ** 2 + (centre[1] + offsets[:, 1] - guess[1]) ** 2
            dyaw = np.array([_wrap(float(y) - guess[2]) for y in yaws])
            prior = prior_weight * 0.5 * (dxy2[None, :] / prior_sigma_m ** 2
                                          + (dyaw[:, None] / math.radians(prior_sigma_deg)) ** 2)
            k, m = np.unravel_index(int(np.argmax(scores - prior)), scores.shape)
            centre = (centre[0] + offsets[m, 0], centre[1] + offsets[m, 1], _wrap(float(yaws[k])))
            score = float(scores[k, m])
        return centre, score


class Slam2D:
    """Predict with odometry, correct by scan-to-map matching, then map."""

    def __init__(self, grid: OccupancyGrid, min_map_cells: int = 40, min_returns: int = 30,
                 min_score: float = 0.15, initial_pose: Pose | None = None) -> None:
        """``initial_pose`` fixes the map frame (e.g. an averaged heading); by default the
        first odometry pose does, noise and all, and nothing later can rotate it back."""
        self.grid = grid
        self.initial_pose = initial_pose
        self.min_map_cells, self.min_returns, self.min_score = min_map_cells, min_returns, min_score
        self.pose: Pose | None = None
        self._last_odom: Pose | None = None
        self.stats = {"scans": 0, "matched": 0, "rejected": 0, "unmatched": 0, "last_score": None}

    def step(self, odom: Sequence[float], angles: np.ndarray, ranges: np.ndarray) -> Pose:
        odom = (float(odom[0]), float(odom[1]), float(odom[2]))
        self.stats["scans"] += 1
        if self.pose is None:
            pose = self.initial_pose if self.initial_pose is not None else odom
        else:
            predicted = compose(self.pose, relative(self._last_odom, odom))
            returns = int(np.count_nonzero(np.isfinite(ranges)))
            if self.grid.occupied_count() >= self.min_map_cells and returns >= self.min_returns:
                found, score = self.grid.match(predicted, angles, ranges)
                self.stats["last_score"] = score
                if score >= self.min_score:
                    pose = found
                    self.stats["matched"] += 1
                else:  # a poor match is worse than odometry: keep the prediction
                    pose = predicted
                    self.stats["rejected"] += 1
            else:
                pose = predicted
                self.stats["unmatched"] += 1
        self._last_odom = odom
        self.grid.update(pose, angles, ranges)
        self.pose = pose
        return pose
