"""Single-host SITL survey coordination in a shared north/east/down frame.

Original integration using the existing A* planner. Route reservations are
conservative horizontal capsules, not a general dynamic collision guarantee.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from .mission_logic import DEFAULT_OBSTACLE_COURSE, Fence, Obstacle
from .path_planner import plan_path

Point = tuple[float, float, float]
# Launch pads aligned across the front of the obstacle course, inside FENCE.
HOMES = {"px4_1": (0.0, -3.0, 0.0), "px4_2": (0.0, 7.0, 0.0)}
FENCE = Fence(-5.0, 55.0, -6.0, 18.0, 6.0)
# Far-side landing pad (worlds/obstacle_world.sdf visual at N=50 E=0).
LANDINGS = {"px4_1": (50.0, -2.0, 0.0), "px4_2": (50.0, 5.0, 0.0)}
# Visual GPS-denied box N[15.5,31.5] E[-8,8] — keep swarm A* out of it.
GPS_DENIED_ZONE = Obstacle(0.0, 23.5, 16.0, 16.0, 20.0)


def point(value: Iterable[float]) -> Point:
    result = tuple(float(x) for x in value)
    if len(result) != 3 or not all(math.isfinite(x) for x in result):
        raise ValueError("expected three finite coordinates")
    return result


def enu_to_ned(value: Iterable[float]) -> Point:
    east, north, up = point(value)
    return north, east, -up


def local_to_world(local: Point, origin: Point, calibration: Point) -> Point:
    return tuple(local[i] - calibration[i] + origin[i] for i in range(3))


def world_to_local(world: Point, origin: Point, calibration: Point) -> Point:
    return tuple(world[i] - origin[i] + calibration[i] for i in range(3))


def inside(point_: Point) -> bool:
    n, e, d = point_
    return FENCE.north_min <= n <= FENCE.north_max and FENCE.east_min <= e <= FENCE.east_max and -FENCE.altitude_max <= d <= 1.0


def segment_distance(a: Point, b: Point, c: Point, d: Point) -> float:
    """Minimum 2-D distance, including crossing and degenerate segments."""
    def cross(u, v):
        return u[0] * v[1] - u[1] * v[0]
    def sub(u, v):
        return u[0] - v[0], u[1] - v[1]
    ab, cd, ac = sub(b, a), sub(d, c), sub(c, a)
    denom = cross(ab, cd)
    if abs(denom) > 1e-10:
        t, u = cross(ac, cd) / denom, cross(ac, ab) / denom
        if 0 <= t <= 1 and 0 <= u <= 1:
            return 0.0
    def to_segment(p, x, y):
        delta = sub(y, x)
        norm = delta[0] ** 2 + delta[1] ** 2
        t = max(0.0, min(1.0, sum(sub(p, x)[i] * delta[i] for i in (0, 1)) / norm)) if norm else 0.0
        return math.hypot(p[0] - x[0] - t * delta[0], p[1] - x[1] - t * delta[1])
    return min(to_segment(a, c, d), to_segment(b, c, d), to_segment(c, a, b), to_segment(d, a, b))


@dataclass(frozen=True)
class Telemetry:
    vehicle: str
    session: str
    seq: int
    sent: float
    position: Point
    velocity: Point
    state: str
    valid: bool
    armed: bool
    landed: bool
    fault: str = ""

    @classmethod
    def parse(cls, data: dict, vehicles: Iterable[str] | None = None) -> "Telemetry":
        if not isinstance(data, dict):
            raise ValueError("expected telemetry object")
        known = HOMES if vehicles is None else set(vehicles)
        if data.get("vehicle") not in known or not isinstance(data.get("session"), str):
            raise ValueError("unknown vehicle/session")
        if type(data.get("seq")) is not int or data["seq"] < 0:
            raise ValueError("invalid sequence")
        if data.get("state") not in {"WAITING", "TAKEOFF", "READY", "MOVING", "LANDING", "LANDED"}:
            raise ValueError("unknown flight state")
        if not all(type(data.get(k)) is bool for k in ("valid", "armed", "landed")):
            raise ValueError("invalid flags")
        sent = float(data["sent"])
        if not math.isfinite(sent):
            raise ValueError("invalid timestamp")
        return cls(data["vehicle"], data["session"], data["seq"], sent,
                   point(data["position"]), point(data["velocity"]), data["state"],
                   data["valid"], data["armed"], data["landed"], str(data.get("fault", "")))


@dataclass
class SurveyTask:
    name: str
    owner: str
    start: Point
    end: Point
    completed: bool = False


@dataclass(frozen=True)
class FleetTask:
    name: str
    owner: str
    start: Point
    end: Point


@dataclass(frozen=True)
class Fleet:
    """Which vehicles exist, where they start and land, and the initial task owners."""
    homes: dict[str, Point]
    landings: dict[str, Point]
    tasks: tuple[FleetTask, ...]

    def __post_init__(self):
        if not self.homes:
            raise ValueError("a fleet needs at least one vehicle")
        if set(self.landings) != set(self.homes):
            raise ValueError("a fleet needs a landing for every vehicle and no others")
        names = [t.name for t in self.tasks]
        if len(names) != len(set(names)):
            raise ValueError("duplicate task name")
        for t in self.tasks:
            if t.owner not in self.homes:
                raise ValueError(f"task {t.name} has unknown owner {t.owner!r}")


# The original two-vehicle mission. Gate then pad for each vehicle; staggered east
# corridors so both can move in parallel (not serialized on one shared A* funnel).
DEFAULT_FLEET = Fleet(
    homes=HOMES,
    landings=LANDINGS,
    tasks=(
        FleetTask("gate_1", "px4_1", (28.0, 11.0, -3.0), (28.0, 11.0, -3.0)),
        FleetTask("land_1", "px4_1", (50.0, -2.0, -3.0), (50.0, -2.0, -3.0)),
        FleetTask("gate_2", "px4_2", (28.0, 16.5, -3.0), (28.0, 16.5, -3.0)),
        FleetTask("land_2", "px4_2", (50.0, 5.0, -3.0), (50.0, 5.0, -3.0)),
    ),
)

# Reassignment cost, in metres: distance from a healthy vehicle to the released task plus
# this much per task it already owns and has not completed. 10 m means a vehicle with one
# more pending task must be over 10 m closer to win. A tunable, not a derived constant.
WORKLOAD_PENALTY_M = 10.0


class SurveyCoordinator:
    """Deterministic planner; all time inputs use the same host monotonic clock."""
    def __init__(self, *, minimum_separation: float = 2.5, timeout: float = 330.0,
                 fleet: Fleet = DEFAULT_FLEET):
        self.fleet = fleet
        self.homes = dict(fleet.homes)
        self.minimum_separation = minimum_separation
        self.reservation = minimum_separation + 1.0
        # Deliberately smaller than self.reservation: that value guards live
        # inter-vehicle separation (a moving, uncertain hazard) and is too
        # generous for routing around fixed, stationary world geometry —
        # applying it there ate most of the safe gap between OB1 and OB2 and
        # made legitimate commute legs unroutable. Matches the clearance the
        # single-vehicle mission already uses around the same obstacles.
        self.obstacle_clearance = 1.5
        self.timeout = timeout
        self.telemetry: dict[str, Telemetry] = {}
        self.routes: dict[str, list[Point]] = {v: [] for v in self.homes}
        self.active: dict[str, str | None] = {v: None for v in self.homes}
        # Coordinated transit to the far landing pad: A* plans the whole
        # commute (task start == end), avoiding obstacles + GPS-denied keep-out.
        self.tasks = [SurveyTask(t.name, t.owner, t.start, t.end) for t in fleet.tasks]
        self.retired: set[str] = set()
        self.retire_since: dict[str, float] = {}
        self.phase = "WAITING"
        self.reason = "waiting for both vehicles"
        self.started: float | None = None
        self.reassignments = 0
        self.last_progress = 0.0
        self.last_commands: dict[str, dict] = {}

    def update(self, sample: Telemetry, now: float) -> bool:
        if not 0 <= now - sample.sent <= 0.75:
            return False
        old = self.telemetry.get(sample.vehicle)
        if old and sample.session == old.session and sample.seq <= old.seq:
            return False
        if old and sample.session != old.session and self.phase != "WAITING":
            self._abort("vehicle process restarted")
        self.telemetry[sample.vehicle] = sample
        return True

    def _reassign_from(self, failed: str) -> None:
        """A retired vehicle's unfinished tasks go back to the pool and are handed out
        at once, so no task is ever left without an owner (swarm invariant #2).
        Eligible: vehicles that are neither faulted nor retired. Cost: distance to the
        task + WORKLOAD_PENALTY_M per task already owned and unfinished; ties by name.
        With two vehicles there is only ever one candidate, as before."""
        candidates = [o for o in self.homes
                      if o != failed and o not in self.retired and not self.telemetry[o].fault]
        if not candidates:
            return  # nobody can take them; the caller aborts with "no available vehicles"
        for task in self.tasks:
            if task.owner != failed or task.completed:
                continue
            def cost(o: str) -> tuple:
                load = sum(1 for t in self.tasks if t.owner == o and not t.completed)
                return (math.dist(self.telemetry[o].position, task.start) + WORKLOAD_PENALTY_M * load, o)
            task.owner = min(candidates, key=cost)
            self.reassignments += 1

    def _abort(self, reason: str):
        self.phase, self.reason = "ABORTED", reason

    def _hold(self) -> dict[str, dict]:
        return {v: {"action": "hold"} for v in self.homes}

    def _route(self, vehicle: str, goal: Point) -> list[Point]:
        current = self.telemetry[vehicle].position
        # Retired vehicles stay reserved on the ground. No overflight shortcut.
        # The physical course (DEFAULT_OBSTACLE_COURSE) is always avoided too,
        # so commute legs (home->task, task->task, task->home) route around
        # the same obstacles the single-vehicle mission avoids — the straight
        # survey transects themselves (task.start->task.end) are laid out by
        # hand to clear them instead, since those legs bypass planning.
        # A retired vehicle's effective clearance must still reach
        # self.reservation even though this call applies the smaller
        # obstacle_clearance uniformly — the extra margin is baked into its
        # placeholder box size instead (half-size = reservation - obstacle_clearance).
        retired_half_size = max(0.0, self.reservation - self.obstacle_clearance)
        obstacles = list(DEFAULT_OBSTACLE_COURSE) + [GPS_DENIED_ZONE] + [
            Obstacle(t.position[1], t.position[0], retired_half_size * 2, retired_half_size * 2, 10.0)
            for v, t in self.telemetry.items() if v != vehicle and v in self.retired
        ]
        route = plan_path(current[:2], goal[:2], obstacles, FENCE,
                          clearance_m=self.obstacle_clearance, resolution_m=0.5,
                          fence_margin_m=0.5)
        return [(n, e, goal[2]) for n, e in route]

    def step(self, now: float) -> dict[str, dict]:
        commands = self._step(now)
        self.last_commands = commands
        return commands

    def _step(self, now: float) -> dict[str, dict]:
        if self.phase in {"ABORTED", "COMPLETE"}:
            return {v: {"action": "land"} for v in self.homes}
        if len(self.telemetry) != len(self.homes):
            return self._hold()
        if any(now - t.sent > 0.75 or not t.valid for t in self.telemetry.values()):
            if self.phase != "WAITING":
                self._abort("stale telemetry or invalid world position")
                return {v: {"action": "land"} for v in self.homes}
            return self._hold()
        samples = list(self.telemetry.values())
        # Every pair. (With two vehicles this was samples[0] vs samples[1]; for N it has
        # to be all N*(N-1)/2 pairs or a breach between, say, the 1st and 3rd goes unseen.)
        pairs = [(a, b) for i, a in enumerate(samples) for b in samples[i + 1:]]
        if any(math.dist(a.position[:2], b.position[:2]) < self.minimum_separation for a, b in pairs):
            self._abort("minimum horizontal separation breached")
            return {v: {"action": "land"} for v in self.homes}
        # Constant-velocity closest approach over the next second catches
        # unexpected closing motion even when planned segments are disjoint.
        for a, b in pairs:
            relative = tuple(a.position[i] - b.position[i] for i in (0, 1))
            velocity = tuple(a.velocity[i] - b.velocity[i] for i in (0, 1))
            speed_sq = sum(x * x for x in velocity)
            closest_t = max(0.0, min(1.0, -sum(relative[i] * velocity[i] for i in (0, 1)) / speed_sq)) if speed_sq else 0.0
            predicted = math.hypot(*(relative[i] + closest_t * velocity[i] for i in (0, 1)))
            if predicted < self.minimum_separation:
                self._abort("predicted separation breach")
                return {v: {"action": "land"} for v in self.homes}
        if self.started is not None and now - self.started > self.timeout:
            self._abort("mission timeout / blocked route")
            return {v: {"action": "land"} for v in self.homes}
        if self.phase == "WAITING":
            if any(t.state != "WAITING" or t.armed or not t.landed for t in samples):
                self._abort("vehicles must start landed and disarmed")
                return {v: {"action": "land"} for v in self.homes}
            self.phase, self.reason = "STARTING", "takeoff"
            self.started = now
        # A fault never releases its assignment until fresh land + disarm
        # evidence persists for a full second. Missing telemetry aborts above.
        for v, t in self.telemetry.items():
            if t.fault and v not in self.retired:
                if t.state == "LANDED" and t.landed and not t.armed:
                    self.retire_since.setdefault(v, now)
                    if now - self.retire_since[v] >= 1.0:
                        self.retired.add(v)
                        self.routes[v] = []
                        self.active[v] = None
                        self._reassign_from(v)
                else:
                    self.retire_since.pop(v, None)
        if any(t.fault and v not in self.retired for v, t in self.telemetry.items()):
            self.reason = "waiting for failed vehicle to land and disarm"
            return {v: {"action": "land" if t.fault else "hold"} for v, t in self.telemetry.items()}
        healthy = [v for v in self.homes if v not in self.retired]
        if not healthy:
            self._abort("no available vehicles")
            return {v: {"action": "land"} for v in self.homes}
        if self.phase == "STARTING":
            if not all(self.telemetry[v].state in {"READY", "MOVING"} for v in healthy):
                return {v: {"action": "land" if v in self.retired else "takeoff"} for v in self.homes}
            self.phase = "SURVEY"
        # Consume reached route points only from fresh position and low speed.
        for v in healthy:
            t = self.telemetry[v]
            # Transit corners may be consumed early, but a task endpoint must
            # actually be visited. The evidence verifier requires <0.6 m;
            # 0.5 m leaves sampling margin before turning toward the next task.
            # Using the transit radius here completed recovery tasks ~0.9 m
            # away, so COMPLETE did not imply independently observed coverage.
            task_endpoint = self.active[v] is not None and len(self.routes[v]) == 1
            acceptance_m = 0.5 if task_endpoint else 1.0
            if self.routes[v] and math.dist(t.position, self.routes[v][0]) < acceptance_m and math.dist(t.velocity, (0, 0, 0)) < 4.0:
                self.routes[v].pop(0)
                self.last_progress = now
                if not self.routes[v] and self.active[v]:
                    next(task for task in self.tasks if task.name == self.active[v]).completed = True
                    self.active[v] = None
        if all(task.completed for task in self.tasks):
            self.phase = "RETURNING"
        commands = {v: {"action": "land"} for v in self.retired}
        for v in healthy:
            t = self.telemetry[v]
            if self.phase == "RETURNING":
                home = (*self.fleet.landings[v][:2], -3.0)
                if t.state in {"LANDING", "LANDED"} or math.dist(t.position, home) < 0.4:
                    commands[v] = {"action": "land"}
                    continue
                if not self.routes[v]:
                    try:
                        self.routes[v] = self._route(v, home)
                    except (ValueError, RuntimeError):
                        commands[v] = {"action": "hold"}
                        continue
            elif not self.routes[v]:
                pending = sorted((task for task in self.tasks if task.owner == v and not task.completed),
                                 key=lambda task: math.dist(t.position, task.start))
                for task in pending:
                    try:
                        route = self._route(v, task.start)
                        # Survey itself must remain a straight transect, not
                        # a detour that silently leaves a coverage gap.
                        if any(segment_distance(task.start, task.end, self.telemetry[r].position,
                                                self.telemetry[r].position) <= self.reservation for r in self.retired):
                            continue
                        self.routes[v] = route + [task.end]
                        self.active[v] = task.name
                        break
                    except (ValueError, RuntimeError):
                        continue
            commands[v] = {"action": "hold"}
        # Persistent, conservative reservations use each peer's last issued
        # segment as well as its current location. Newly issued segments are
        # considered immediately, before telemetry can acknowledge motion.
        reservations = {}
        for v, t in self.telemetry.items():
            previous = self.last_commands.get(v, {})
            end = tuple(previous["target"]) if previous.get("action") == "move" else t.position
            reservations[v] = (t.position, end)
        for v in healthy:
            if commands[v]["action"] == "land" or not self.routes[v]:
                continue
            start, end = self.telemetry[v].position, self.routes[v][0]
            if any(segment_distance(start, end, *segment) <= self.reservation
                   for other, segment in reservations.items() if other != v):
                continue
            commands[v] = {"action": "move", "target": list(end)}
            reservations[v] = (start, end)
        if self.phase == "RETURNING" and all(t.state == "LANDED" and t.landed and not t.armed for t in samples):
            self.phase, self.reason = "COMPLETE", "both vehicles landed at far pad"
        else:
            self.reason = "transiting to far pad" if self.phase == "SURVEY" else "landing at far pad"
        return commands

    def report(self) -> dict:
        return {"phase": self.phase, "reason": self.reason,
                "completed": [t.name for t in self.tasks if t.completed],
                "assignments": {t.name: t.owner for t in self.tasks},
                "retired": sorted(self.retired), "reassignments": self.reassignments}
