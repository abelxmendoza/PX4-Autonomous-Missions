"""Single-host SITL survey coordination in a shared north/east/down frame.

Original integration using the existing A* planner. Route reservations are
conservative horizontal capsules, not a general dynamic collision guarantee.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from .mission_logic import DEFAULT_OBSTACLE_COURSE, Fence, Obstacle, segment_hits_expanded_aabb
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

# A circular hold that lasts this long is a deadlock, not a drone waiting for
# another to pass. Short enough that the mission does not sit in silence until
# the 330 s timeout, and long enough that a yield already underway is not
# aborted. This does not change the mission timeout.
DEADLOCK_HOLD_S = 15.0

# How much of an issued move is reserved for other drones. The vehicle still
# flies the whole leg; only the conflict check is limited to the part it can
# reach inside the 1 s predicted-separation window at the harness's fastest
# speed (1.5 m/s) plus the 3.5 m reservation. Reserving the entire leg made
# the drone behind wait until the leg was finished, which is the gridlock.
RESERVATION_HORIZON_M = 5.0


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
        # Right-of-way bookkeeping. blocked_since is the first time a drone
        # with a route was refused a move; yield_target is a committed sidestep
        # so a yielder does not pick a new point every tick.
        self.blocked_since: dict[str, float] = {}
        self.yield_target: dict[str, Point] = {}
        self._idle_since: float | None = None
        self._cycle_since: float | None = None
        # Set while drones are stuck, cleared when they move again. A circular
        # wait also aborts; a plain stall is reported and left to the timeout
        # so a physically impossible task (a grounded drone on the goal) still
        # ends the way the two-vehicle characterization records it.
        self.deadlock: str | None = None
        self.deadlock_events: list[str] = []
        self._replan_not_before: dict[str, float] = {}

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

    def _approach_limit(self, other: str) -> float:
        """Distance another drone's body must be kept outside.

        Airborne drones keep the 3.5 m reservation. A drone that has landed and
        disarmed is not going to close the gap, and the pads are 5.5 m apart, so
        a 3.5 m capsule on each side overlaps and leaves no way back to a pad.
        The 2.5 m floor is unchanged, as is the predicted-separation check.
        """
        sample = self.telemetry[other]
        if sample.state == "LANDED" and sample.landed and not sample.armed:
            return self.minimum_separation
        return self.reservation

    def _route(self, vehicle: str, goal: Point, avoid: Iterable[str] = (),
               avoid_keep_out: float | None = None) -> list[Point]:
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
        avoid_half_size = retired_half_size if avoid_keep_out is None else max(
            0.0, avoid_keep_out - self.obstacle_clearance)
        avoid = set(avoid)
        obstacles = list(DEFAULT_OBSTACLE_COURSE) + [GPS_DENIED_ZONE] + [
            Obstacle(t.position[1], t.position[0],
                     (avoid_half_size if v in avoid and v not in self.retired else retired_half_size) * 2,
                     (avoid_half_size if v in avoid and v not in self.retired else retired_half_size) * 2,
                     10.0)
            for v, t in self.telemetry.items()
            if v != vehicle and (v in self.retired or v in avoid)
        ]
        route = plan_path(current[:2], goal[:2], obstacles, FENCE,
                          clearance_m=self.obstacle_clearance, resolution_m=0.5,
                          fence_margin_m=0.5)
        return [(n, e, goal[2]) for n, e in route]

    @staticmethod
    def _vehicle_rank(name: str) -> tuple:
        """Deterministic priority key. Lower sorts first: px4_1 before px4_2 before px4_10."""
        tail = name.rsplit("_", 1)[-1]
        if tail.isdigit():
            return (int(tail), name)
        return (1_000_000_000, name)

    def _outranks(self, first: str, second: str) -> bool:
        """Who moves first when two drones want the same space.

        A drone that still has a route outranks one that is only hovering,
        because the hoverer is what keeps the mission from ever reaching
        RETURNING. Otherwise the lower vehicle rank wins, so the order is
        total and a wait cannot be circular.
        """
        first_busy = bool(self.routes.get(first)) and first not in self.retired
        second_busy = bool(self.routes.get(second)) and second not in self.retired
        if first_busy != second_busy:
            return first_busy
        return self._vehicle_rank(first) < self._vehicle_rank(second)

    @staticmethod
    def _horizon(start: Point, end: Point) -> Point:
        """The point at most RESERVATION_HORIZON_M along start→end."""
        distance = math.dist(start[:2], end[:2])
        if distance <= RESERVATION_HORIZON_M or distance < 1e-9:
            return end
        frac = RESERVATION_HORIZON_M / distance
        return tuple(start[i] + frac * (end[i] - start[i]) for i in range(3))

    def _in_free_space(self, start: Point, end: Point, ignore_vehicles: set[str]) -> bool:
        """Yield targets stay inside the fence and off the known obstacles.

        A retired vehicle we are stepping away from is ignored: the start point
        is already inside its inflated box, so every segment would 'hit' it.
        """
        margin = 0.5
        north, east, down = end
        if not (FENCE.north_min + margin <= north <= FENCE.north_max - margin
                and FENCE.east_min + margin <= east <= FENCE.east_max - margin
                and -FENCE.altitude_max <= down <= 1.0):
            return False
        obstacles = list(DEFAULT_OBSTACLE_COURSE) + [GPS_DENIED_ZONE]
        half = max(0.0, self.reservation - self.obstacle_clearance)
        for vehicle, sample in self.telemetry.items():
            if vehicle not in self.retired or vehicle in ignore_vehicles:
                continue
            obstacles.append(Obstacle(
                sample.position[1], sample.position[0], half * 2, half * 2, 10.0))
        return not any(
            segment_hits_expanded_aabb(start, end, obstacle, self.obstacle_clearance)[0]
            for obstacle in obstacles
        )

    def _yield_ok(self, vehicle: str, start: Point, end: Point, threat_segments: list[tuple[Point, Point]],
                  reservations: dict[str, tuple[Point, Point]], escaping: set[str]) -> bool:
        """A sidestep is legal when it leaves the threatened segment and does not
        enter anyone else's reservation. A body we are already inside the
        reservation of (but still outside the 2.5 m safety floor) may be left;
        the move has to increase that distance and must not cross the floor.
        """
        if not self._in_free_space(start, end, escaping):
            return False
        for other, segment in reservations.items():
            if other == vehicle:
                continue
            if other in escaping and math.dist(start[:2], self.telemetry[other].position[:2]) <= self.reservation:
                body = self.telemetry[other].position
                if segment_distance(start, end, body, body) < self.minimum_separation:
                    return False
                if math.dist(end[:2], body[:2]) <= math.dist(start[:2], body[:2]):
                    return False
                continue
            if segment_distance(start, end, *segment) <= self.reservation:
                return False
        return all(segment_distance(end, end, *segment) > self.reservation for segment in threat_segments)

    def _choose_yield(self, vehicle: str, threat_segments: list[tuple[Point, Point]],
                      reservations: dict[str, tuple[Point, Point]], escaping: set[str],
                      goal: Point | None = None) -> Point | None:
        start = self.telemetry[vehicle].position
        locked = self.yield_target.get(vehicle)
        # Stay on the committed sidestep until it is reached. Re-picking every
        # tick, while still a metre short, walked drones backwards along the
        # launch line for minutes.
        if locked is not None and math.dist(start, locked) >= 0.5 and self._yield_ok(
                vehicle, start, locked, threat_segments, reservations, escaping):
            return locked
        best: Point | None = None
        best_key: tuple | None = None
        # A drone that still has a route should step toward that route, not
        # merely the shortest legal hop. The shortest hop is what pushed a
        # blocked drone away from its gate for the whole mission. An idle
        # hoverer has no route, so the short hop that leaves the bubble is enough.
        # Never step past the goal: a 12 m hop toward a pad 8 m away lands in
        # the next row of landed drones, and the gaps there are not routable.
        goal_limit = math.dist(start[:2], goal[:2]) if goal is not None else None
        for distance in (1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0):
            if goal_limit is not None and distance > goal_limit + 0.25:
                continue
            for step in range(16):
                angle = step * math.pi / 8.0
                candidate = (start[0] + distance * math.cos(angle),
                             start[1] + distance * math.sin(angle), start[2])
                if not self._yield_ok(vehicle, start, candidate, threat_segments, reservations, escaping):
                    continue
                clearance = min(segment_distance(candidate, candidate, *segment) for segment in threat_segments)
                if goal is None:
                    key = (distance, -clearance, step)
                else:
                    key = (math.dist(candidate[:2], goal[:2]), distance, -clearance, step)
                if best_key is None or key < best_key:
                    best_key, best = key, candidate
            if goal is None and best is not None:
                break
        if best is not None:
            self.yield_target[vehicle] = best
        else:
            self.yield_target.pop(vehicle, None)
        return best

    def _holding_cycle(self, denied: dict[str, list[str]], commands: dict[str, dict]) -> list[str] | None:
        """A cycle in the 'who am I waiting for' graph, among drones that are actually holding.

        A drone that has finished its own tasks and is hovering in someone else's
        path will not move until RETURNING, and RETURNING will not start until
        that someone finishes — so the hoverer waits on the drone it is blocking.
        """
        holding = {v for v, cmd in commands.items()
                   if cmd.get("action") == "hold" and v not in self.retired}
        graph: dict[str, list[str]] = {}
        for vehicle, blockers in denied.items():
            if vehicle not in holding:
                continue
            live = [b for b in blockers if b in holding]
            if live:
                graph[vehicle] = live
        for vehicle, blockers in list(graph.items()):
            for blocker in blockers:
                if not self.routes.get(blocker):
                    graph.setdefault(blocker, [])
                    if vehicle not in graph[blocker]:
                        graph[blocker].append(vehicle)
        return _first_cycle(graph)

    def _abort_if_circular_wait(self, now: float, commands: dict[str, dict],
                                denied: dict[str, list[str]]) -> bool:
        if self.phase not in {"SURVEY", "RETURNING"}:
            self._cycle_since = None
            return False
        cycle = self._holding_cycle(denied, commands)
        if not cycle:
            self._cycle_since = None
            return False
        if self._cycle_since is None:
            self._cycle_since = now
        if now - self._cycle_since < DEADLOCK_HOLD_S:
            return False
        text = "circular wait: " + " -> ".join(cycle)
        if self.deadlock != text:
            self.deadlock_events.append(text)
        self.deadlock = text
        self._abort(f"deadlock: {text}")
        return True

    def _note_idle(self, now: float, commands: dict[str, dict]) -> None:
        """Say so when the mission is simply not moving. Does not abort: an
        unreachable task (grounded drone on the goal) is already characterized
        as a timeout, and this only stops it being a silent one."""
        if self.phase not in {"SURVEY", "RETURNING"}:
            self._idle_since = None
            return
        healthy = [v for v in self.homes if v not in self.retired]
        holds = [v for v in healthy if commands.get(v, {}).get("action") == "hold"]
        moves = [v for v in healthy if commands.get(v, {}).get("action") == "move"]
        if moves or not holds:
            self._idle_since = None
            if self.deadlock and self.deadlock.startswith("no mission progress"):
                self.deadlock = None
            return
        if self._idle_since is None:
            self._idle_since = now
            return
        if now - self._idle_since < DEADLOCK_HOLD_S:
            return
        text = f"no mission progress for {DEADLOCK_HOLD_S:.0f}s while drones hold"
        if not (self.deadlock or "").startswith("no mission progress"):
            self.deadlock_events.append(text)
        self.deadlock = text

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
        # Grants go out in right-of-way order (a drone with a route before an
        # idle hoverer, then lower vehicle rank). The reservation test itself
        # is unchanged: a drone still may not enter the 3.5 m capsule.
        reservations: dict[str, tuple[Point, Point]] = {}
        for v, t in self.telemetry.items():
            previous = self.last_commands.get(v, {})
            if previous.get("action") == "move":
                reservations[v] = (t.position, self._horizon(t.position, tuple(previous["target"])))
            else:
                reservations[v] = (t.position, t.position)
        intents = {
            v: (self.telemetry[v].position, self.routes[v][0])
            for v in healthy if commands[v]["action"] != "land" and self.routes[v]
        }
        granted: set[str] = set()
        denied: dict[str, list[str]] = {}
        for v in sorted(intents, key=self._vehicle_rank):
            start, end = intents[v]
            blockers = [other for other, segment in reservations.items()
                        if other != v and segment_distance(start, self._horizon(start, end), *segment) <= self._approach_limit(other)]
            if blockers:
                denied[v] = blockers
                self.blocked_since.setdefault(v, now)
                continue
            commands[v] = {"action": "move", "target": list(end)}
            reservations[v] = (start, self._horizon(start, end))
            granted.add(v)
            self.blocked_since.pop(v, None)
            self.yield_target.pop(v, None)
        # Drones that lost the right-of-way, and idle drones sitting on a
        # winner's next segment, step aside. They do not get to fly through
        # the winner, and the winner does not fly until the sidestep has
        # cleared the reservation. A landed drone cannot step aside; if it is
        # the only thing on the polyline, drop that polyline so the next tick
        # plans around it (the planner already treats retired vehicles as
        # obstacles, but only at the moment a route is built).
        needs_yield: dict[str, list[tuple[Point, Point]]] = {}
        escaping: dict[str, set[str]] = {}
        for v, blockers in denied.items():
            if all(b in self.retired for b in blockers):
                self.routes[v] = []
                self.active[v] = None
            for b in blockers:
                if b in granted or commands.get(b, {}).get("action") == "land":
                    continue
                if b in self.retired:
                    body = self.telemetry[b].position
                    if segment_distance(self.telemetry[v].position, self.telemetry[v].position,
                                        body, body) <= self.reservation:
                        needs_yield.setdefault(v, []).append((body, body))
                        escaping.setdefault(v, set()).add(b)
                    continue
                if self._outranks(v, b):
                    needs_yield.setdefault(b, []).append(intents[v])
                    escaping.setdefault(b, set()).add(v)
        for v in list(self.yield_target):
            if v not in needs_yield:
                self.yield_target.pop(v, None)
        for v in sorted(needs_yield, key=self._vehicle_rank, reverse=True):
            # Aim at the end of the route, not the next fence-hugging corner.
            # Steering toward that corner walked drones into the east fence.
            # With no route, a short hop is enough to leave the bubble.
            goal = self.routes[v][-1] if self.routes.get(v) else None
            target = self._choose_yield(v, needs_yield[v], reservations, escaping.get(v, set()), goal)
            if target is None:
                continue
            start = self.telemetry[v].position
            commands[v] = {"action": "move", "target": list(target)}
            reservations[v] = (start, target)
            self.blocked_since.pop(v, None)
        # Right-of-way cannot move a drone that is landed, landing, or boxed
        # into a pad with no legal sidestep. Those peers are stationary, so
        # the blocked drone replans around them. A landed, disarmed peer is
        # given the 2.5 m floor rather than the 3.5 m reservation: the pads are
        # 5.5 m apart, and two 3.5 m capsules overlap, so there is no path
        # home that honours 3.5 m. Airborne peers still keep 3.5 m. The new
        # leg is refused when it enters that limit. Eight seconds of holding
        # come first so a drone that is merely passing through is not turned
        # into a detour.
        for v, blockers in list(denied.items()):
            if commands[v]["action"] != "hold" or not self.routes.get(v):
                continue
            if now - self.blocked_since.get(v, now) < 8.0:
                continue
            if now < self._replan_not_before.get(v, 0.0):
                continue
            # Yield already had its chance this tick. What remains is a peer
            # with nowhere to sidestep: no route, not moving. Replan around
            # those bodies only — replanning around every holder packed the
            # corridor solid and made A* run every tick.
            idle = [b for b in blockers if b not in self.retired and not self.routes.get(b)
                    and commands.get(b, {}).get("action") != "move"]
            if not idle:
                continue
            if any(commands.get(b, {}).get("action") == "move" for b in blockers if b not in self.retired):
                continue
            live = idle
            goal = self.routes[v][-1]
            if self.active[v]:
                task = next(task for task in self.tasks if task.name == self.active[v])
                if task.start != task.end and math.dist(self.telemetry[v].position, task.start) > 0.5:
                    goal = task.start
            settled = [b for b in live
                       if self.telemetry[b].state == "LANDED" and self.telemetry[b].landed
                       and not self.telemetry[b].armed]
            try:
                # Landed peers use the 2.5 m floor. The 3.5 m box overlaps the
                # 5.5 m pad spacing and the planner then reports no path home.
                route = self._route(
                    v, goal, avoid=live,
                    avoid_keep_out=self.minimum_separation if settled and len(settled) == len(live) else None)
            except (ValueError, RuntimeError):
                # The start cell snapped into a stationary peer's inflated box
                # (common when a drone is just outside the 3.5 m reservation and
                # hard against the fence). Step away, then replan next tick.
                # A landed peer cannot yield, and right-of-way does not let us
                # fly through it.
                bodies = live or [b for b in blockers if b in self.telemetry]
                if bodies:
                    nearest = min(bodies, key=lambda b: math.dist(
                        self.telemetry[v].position, self.telemetry[b].position))
                    threat = (self.telemetry[nearest].position, self.telemetry[nearest].position)
                    target = self._choose_yield(v, [threat], reservations, {nearest})
                    if target is not None:
                        start = self.telemetry[v].position
                        commands[v] = {"action": "move", "target": list(target)}
                        reservations[v] = (start, target)
                self._replan_not_before[v] = now + 2.0
                continue
            if not route:
                continue
            start = self.telemetry[v].position
            end = route[0]
            if any(segment_distance(start, end, *segment) <= self._approach_limit(other)
                   for other, segment in reservations.items() if other != v):
                self._replan_not_before[v] = now + 2.0
                continue
            self.routes[v] = route
            commands[v] = {"action": "move", "target": list(end)}
            reservations[v] = (start, end)
            self.blocked_since.pop(v, None)
            self.yield_target.pop(v, None)
        if self.phase == "RETURNING" and all(t.state == "LANDED" and t.landed and not t.armed for t in samples):
            self.phase, self.reason = "COMPLETE", "both vehicles landed at far pad"
            self.deadlock = None
            return commands
        if self._abort_if_circular_wait(now, commands, denied):
            return {v: {"action": "land"} for v in self.homes}
        self._note_idle(now, commands)
        if self.deadlock:
            self.reason = self.deadlock
        else:
            self.reason = "transiting to far pad" if self.phase == "SURVEY" else "landing at far pad"
        return commands

    def report(self) -> dict:
        return {"phase": self.phase, "reason": self.reason,
                "completed": [t.name for t in self.tasks if t.completed],
                "assignments": {t.name: t.owner for t in self.tasks},
                "retired": sorted(self.retired), "reassignments": self.reassignments,
                "deadlock": self.deadlock, "deadlock_events": list(self.deadlock_events)}


def _first_cycle(graph: dict[str, list[str]]) -> list[str] | None:
    """One cycle as [a, b, ..., a], or None. Deterministic in the start node."""
    color: dict[str, int] = {}
    parent: dict[str, str] = {}

    def walk(node: str) -> list[str] | None:
        color[node] = 1
        for nxt in graph.get(node, ()):
            if color.get(nxt, 0) == 0:
                parent[nxt] = node
                found = walk(nxt)
                if found:
                    return found
            elif color.get(nxt) == 1:
                cycle = [nxt]
                cursor = node
                while cursor != nxt and cursor in parent:
                    cycle.append(cursor)
                    cursor = parent[cursor]
                cycle.append(nxt)
                cycle.reverse()
                return cycle
        color[node] = 2
        return None

    for node in sorted(graph):
        if color.get(node, 0) == 0:
            found = walk(node)
            if found:
                return found
    return None
