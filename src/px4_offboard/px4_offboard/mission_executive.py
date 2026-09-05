"""Mission executive: resource-aware mode management for onboard autonomy.

Pure Python (no ROS). Models a spacecraft-style mission executive on top of
an aerial autonomy sandbox:

  NOMINAL  → full science + navigation
  DEGRADED → skip non-critical (science) waypoints to conserve resources
  SAFE     → hold / stop advancing; prepare recovery
  ABORT    → terminate mission (land / failsafe)

Resources (simulated):
  battery_frac      [0, 1]
  link_quality      [0, 1]
  compute_load      [0, 1]
  propellant_time_s remaining propulsion budget (seconds)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto


class MissionMode(Enum):
    NOMINAL = auto()
    DEGRADED = auto()
    SAFE = auto()
    ABORT = auto()


class ExecutiveAction(Enum):
    CONTINUE = auto()
    SKIP_SCIENCE = auto()
    HOLD_SAFE = auto()
    ABORT_LAND = auto()


def path_suffix_costs_m(waypoints: list[list[float]] | tuple) -> list[float]:
    """Suffix path length (meters) on the waypoint graph: cost[i] = Σ_{j≥i} ‖wp[j+1]-wp[j]‖."""
    n = len(waypoints)
    costs = [0.0] * n
    for i in range(n - 2, -1, -1):
        a, b = waypoints[i], waypoints[i + 1]
        costs[i] = costs[i + 1] + math.sqrt(
            (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2
        )
    return costs


@dataclass
class ResourceBudgets:
    """Thresholds for mode transitions (fractions unless noted)."""

    battery_degraded: float = 0.45
    battery_safe: float = 0.25
    battery_abort: float = 0.12
    link_degraded: float = 0.55
    link_safe: float = 0.30
    link_abort: float = 0.10
    compute_degraded: float = 0.80
    compute_safe: float = 0.92
    propellant_degraded_s: float = 90.0
    propellant_safe_s: float = 45.0
    propellant_abort_s: float = 20.0
    link_loss_abort_s: float = 8.0


@dataclass
class ResourceState:
    battery_frac: float = 1.0
    link_quality: float = 1.0
    compute_load: float = 0.15
    propellant_time_s: float = 180.0
    link_loss_timer_s: float = 0.0

    def as_dict(self) -> dict:
        return {
            "battery_frac": round(self.battery_frac, 3),
            "link_quality": round(self.link_quality, 3),
            "compute_load": round(self.compute_load, 3),
            "propellant_time_s": round(self.propellant_time_s, 1),
            "link_loss_timer_s": round(self.link_loss_timer_s, 2),
        }


@dataclass
class ExecutiveDecision:
    mode: MissionMode
    action: ExecutiveAction
    reason: str
    resources: ResourceState
    skipped_waypoints: list[int] = field(default_factory=list)


@dataclass
class MissionExecutiveConfig:
    enable: bool = True
    science_waypoints: tuple[int, ...] = (2, 4, 6)  # optional / skippable
    initial_battery: float = 1.0
    initial_propellant_s: float = 180.0
    # Drain rates (per second) while airborne / maneuvering
    battery_idle_drain: float = 0.0008
    battery_move_drain: float = 0.0025
    battery_avoid_drain: float = 0.0040
    propellant_idle_drain: float = 0.15
    propellant_move_drain: float = 0.55
    propellant_avoid_drain: float = 0.85
    compute_base: float = 0.18
    compute_move: float = 0.35
    compute_avoid: float = 0.55
    compute_lidar: float = 0.15
    # Simulated link: decays when "stale", recovers when healthy
    link_recover_rate: float = 0.25
    link_decay_rate: float = 0.35
    cruise_speed_mps: float = 2.0  # for path-budget vs propellant time
    path_budget_margin: float = 0.85  # skip science when path > margin * range
    budgets: ResourceBudgets = field(default_factory=ResourceBudgets)


class MissionExecutive:
    """Resource monitor + mode manager for mission-level decisions."""

    def __init__(self, config: MissionExecutiveConfig | None = None) -> None:
        self.config = config or MissionExecutiveConfig()
        self.mode = MissionMode.NOMINAL
        self.resources = ResourceState(
            battery_frac=self.config.initial_battery,
            propellant_time_s=self.config.initial_propellant_s,
        )
        self.last_reason = "startup"
        self.skipped_waypoints: list[int] = []
        self._mode_history: list[tuple[str, str]] = [("NOMINAL", "startup")]

    def update(
        self,
        dt_s: float,
        *,
        airborne: bool,
        moving: bool,
        avoiding: bool,
        link_ok: bool,
        lidar_active: bool = False,
    ) -> ResourceState:
        """Advance simulated resources for one control tick."""
        if not self.config.enable or dt_s <= 0.0:
            return self.resources

        cfg = self.config
        res = self.resources

        if airborne:
            drain_b = cfg.battery_idle_drain
            drain_p = cfg.propellant_idle_drain
            compute = cfg.compute_base
            if moving:
                drain_b = cfg.battery_move_drain
                drain_p = cfg.propellant_move_drain
                compute = cfg.compute_move
            if avoiding:
                drain_b = cfg.battery_avoid_drain
                drain_p = cfg.propellant_avoid_drain
                compute = cfg.compute_avoid
            if lidar_active:
                compute = min(1.0, compute + cfg.compute_lidar)
            res.battery_frac = max(0.0, res.battery_frac - drain_b * dt_s)
            res.propellant_time_s = max(0.0, res.propellant_time_s - drain_p * dt_s)
            res.compute_load = min(1.0, 0.7 * res.compute_load + 0.3 * compute)
        else:
            res.compute_load = max(cfg.compute_base, res.compute_load * 0.95)

        if link_ok:
            res.link_quality = min(
                1.0, res.link_quality + cfg.link_recover_rate * dt_s
            )
            res.link_loss_timer_s = 0.0
        else:
            res.link_quality = max(
                0.0, res.link_quality - cfg.link_decay_rate * dt_s
            )
            res.link_loss_timer_s += dt_s

        return res

    def evaluate(
        self,
        *,
        current_wp_index: int,
        remaining_waypoints: int,
        remaining_path_m: float | None = None,
    ) -> ExecutiveDecision:
        """Select mission mode + action from current resources.

        ``remaining_path_m`` is the waypoint-graph suffix cost from the current
        index (see ``path_suffix_costs_m``). When path length exceeds the
        propellant range budget, science waypoints are skipped even before a
        full DEGRADED resource trip — a greedy path-cost heuristic.
        """
        if not self.config.enable:
            return ExecutiveDecision(
                mode=MissionMode.NOMINAL,
                action=ExecutiveAction.CONTINUE,
                reason="executive disabled",
                resources=self.resources,
            )

        desired = self._classify_mode()
        reason = self._mode_reason(desired)

        if desired is not self.mode:
            self._mode_history.append((desired.name, reason))
            self.mode = desired
            self.last_reason = reason

        action = ExecutiveAction.CONTINUE
        skipped: list[int] = []
        path_tight = False
        if remaining_path_m is not None:
            reachable = (
                self.resources.propellant_time_s
                * self.config.cruise_speed_mps
                * self.config.path_budget_margin
            )
            path_tight = remaining_path_m > reachable

        if desired is MissionMode.ABORT:
            action = ExecutiveAction.ABORT_LAND
        elif desired is MissionMode.SAFE:
            action = ExecutiveAction.HOLD_SAFE
        elif desired is MissionMode.DEGRADED or path_tight:
            if (
                remaining_waypoints > 0
                and current_wp_index in self.config.science_waypoints
                and current_wp_index not in self.skipped_waypoints
            ):
                action = ExecutiveAction.SKIP_SCIENCE
                skipped = [current_wp_index]
                if path_tight and desired is MissionMode.NOMINAL:
                    reason = (
                        f"path budget tight ({remaining_path_m:.0f}m > "
                        f"{reachable:.0f}m range) — skip science"
                    )
                    self.last_reason = reason

        return ExecutiveDecision(
            mode=desired,
            action=action,
            reason=reason,
            resources=self.resources,
            skipped_waypoints=skipped,
        )

    def mark_skipped(self, wp_index: int) -> None:
        if wp_index not in self.skipped_waypoints:
            self.skipped_waypoints.append(wp_index)

    def is_science_waypoint(self, wp_index: int) -> bool:
        return wp_index in self.config.science_waypoints

    def status_dict(self) -> dict:
        return {
            "mode": self.mode.name,
            "reason": self.last_reason,
            "resources": self.resources.as_dict(),
            "science_waypoints": list(self.config.science_waypoints),
            "skipped_waypoints": list(self.skipped_waypoints),
            "mode_history": list(self._mode_history[-8:]),
            "enabled": self.config.enable,
        }

    def _classify_mode(self) -> MissionMode:
        b = self.config.budgets
        r = self.resources

        if (
            r.battery_frac <= b.battery_abort
            or r.link_quality <= b.link_abort
            or r.propellant_time_s <= b.propellant_abort_s
            or r.link_loss_timer_s >= b.link_loss_abort_s
        ):
            return MissionMode.ABORT

        if (
            r.battery_frac <= b.battery_safe
            or r.link_quality <= b.link_safe
            or r.propellant_time_s <= b.propellant_safe_s
            or r.compute_load >= b.compute_safe
        ):
            return MissionMode.SAFE

        if (
            r.battery_frac <= b.battery_degraded
            or r.link_quality <= b.link_degraded
            or r.propellant_time_s <= b.propellant_degraded_s
            or r.compute_load >= b.compute_degraded
        ):
            return MissionMode.DEGRADED

        return MissionMode.NOMINAL

    def _mode_reason(self, mode: MissionMode) -> str:
        b = self.config.budgets
        r = self.resources
        if mode is MissionMode.ABORT:
            if r.link_loss_timer_s >= b.link_loss_abort_s:
                return f"link lost {r.link_loss_timer_s:.1f}s"
            if r.battery_frac <= b.battery_abort:
                return f"battery critical ({r.battery_frac:.0%})"
            if r.propellant_time_s <= b.propellant_abort_s:
                return f"propellant critical ({r.propellant_time_s:.0f}s)"
            return f"link critical ({r.link_quality:.0%})"
        if mode is MissionMode.SAFE:
            if r.battery_frac <= b.battery_safe:
                return f"battery low ({r.battery_frac:.0%})"
            if r.propellant_time_s <= b.propellant_safe_s:
                return f"propellant low ({r.propellant_time_s:.0f}s)"
            if r.compute_load >= b.compute_safe:
                return f"compute saturated ({r.compute_load:.0%})"
            return f"link weak ({r.link_quality:.0%})"
        if mode is MissionMode.DEGRADED:
            if r.battery_frac <= b.battery_degraded:
                return f"battery degraded ({r.battery_frac:.0%}) — skip science"
            if r.propellant_time_s <= b.propellant_degraded_s:
                return f"propellant tight ({r.propellant_time_s:.0f}s) — skip science"
            if r.compute_load >= b.compute_degraded:
                return f"compute high ({r.compute_load:.0%}) — skip science"
            return f"link degraded ({r.link_quality:.0%}) — skip science"
        return "resources healthy"
