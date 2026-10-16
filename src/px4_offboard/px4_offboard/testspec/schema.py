"""Test specification schema (YAML), validated strictly before anything runs.

A spec names a vehicle adapter, an ordered mission of catalogued actions, an
(optional) fault list and a list of catalogued assertions. Everything is data:
there is no expression language, no eval and no import of Python from YAML.
Unknown keys, unknown actions, unknown assertion types, unsupported faults,
out-of-range or non-finite numbers and impossible action orders are rejected
with a SpecError that names the offending entry.

The parameter tables below are the single source of truth for what a spec may
say; the runner and the assertion evaluators are checked against them in tests.
"""

from __future__ import annotations

import hashlib
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from px4_offboard.faults.schema import ParamSpec
from px4_offboard.mission_geofence import GeofenceConfig
from px4_offboard.altitude_reference import AltitudeFrame

SCHEMA_VERSION = 1
NAME_RE = re.compile(r"[a-z0-9][a-z0-9_]{0,63}")
REQ_RE = re.compile(r"REQ-[A-Z]+-\d{3}")


class SpecError(ValueError):
    pass


class Polygon:
    """Marker kind: a list of >= 3 finite [north_m, east_m] pairs."""


def _f(lo: float, hi: float, default: Any = None, required: bool = False) -> ParamSpec:
    return ParamSpec(float, required=required, default=default, lo=lo, hi=hi)


# --- catalogues ---------------------------------------------------------------

# Vehicle adapters this build can execute against. Phase 3 adds a PX4 SITL
# adapter; until then a spec naming anything else is rejected.
ADAPTERS: dict[str, dict[str, ParamSpec]] = {
    "mock": {
        "rate_hz": _f(1.0, 200.0, default=20.0),
        "velocity_tau_s": _f(0.05, 5.0, default=0.3),
    },
}

# Fault types each adapter can inject. Empty: this slice injects no faults, so
# any fault entry is rejected rather than silently ignored.
SUPPORTED_FAULTS: dict[str, frozenset[str]] = {"mock": frozenset()}

_TIMEOUT = lambda default: _f(0.001, 3600.0, default=default)  # noqa: E731

ACTIONS: dict[str, dict[str, ParamSpec]] = {
    "arm": {"timeout_s": _TIMEOUT(5.0)},
    "takeoff": {
        "altitude_m": _f(0.5, 120.0, required=True),
        "speed_mps": _f(0.1, 5.0, default=1.0),
        "tolerance_m": _f(0.01, 5.0, default=0.2),
        "timeout_s": _TIMEOUT(30.0),
    },
    "goto": {
        "north_m": _f(-1000.0, 1000.0, required=True),
        "east_m": _f(-1000.0, 1000.0, required=True),
        "altitude_m": _f(0.5, 120.0, required=True),
        "speed_mps": _f(0.1, 15.0, default=2.0),
        "vertical_speed_mps": _f(0.1, 5.0, default=1.0),
        "tolerance_m": _f(0.01, 10.0, default=0.3),
        "timeout_s": _TIMEOUT(60.0),
    },
    "hold": {"duration_s": _f(0.0, 600.0, required=True)},
    "land": {
        "speed_mps": _f(0.1, 3.0, default=0.7),
        "tolerance_m": _f(0.01, 1.0, default=0.1),
        "timeout_s": _TIMEOUT(60.0),
    },
    "disarm": {"timeout_s": _TIMEOUT(5.0)},
}

ASSERTIONS: dict[str, dict[str, ParamSpec]] = {
    # Every sample is inside the local-NED keep-in, classified by mission_geofence.
    "geofence": {
        "polygon_ned": ParamSpec(Polygon, required=True),
        "min_altitude_m": _f(-100.0, 1000.0, required=True),
        "max_altitude_m": _f(-100.0, 1000.0, required=True),
    },
    # Height above the local origin never exceeds the limit.
    "max_altitude": {"limit_m": _f(0.0, 1000.0, required=True)},
    # Horizontal and/or vertical speed never exceed their limits (at least one given).
    "max_speed": {
        "horizontal_mps": _f(0.0, 100.0),
        "vertical_mps": _f(0.0, 100.0),
    },
    # The last sample is within tolerance of a point.
    "final_position": {
        "north_m": _f(-1000.0, 1000.0, required=True),
        "east_m": _f(-1000.0, 1000.0, required=True),
        "altitude_m": _f(-10.0, 1000.0, required=True),
        "tolerance_m": _f(0.01, 100.0, required=True),
    },
    # The last sample reports the vehicle disarmed.
    "final_disarmed": {},
}

# Which actions are legal in which (armed, airborne) condition, and what they lead to.
_SEQUENCE = {
    "arm": ((False, False), (True, False)),
    "takeoff": ((True, False), (True, True)),
    "goto": ((True, True), (True, True)),
    "hold": ((True, True), (True, True)),
    "land": ((True, True), (True, False)),
    "disarm": ((True, False), (False, False)),
}


# --- parsed form --------------------------------------------------------------

@dataclass(frozen=True)
class VehicleSpec:
    adapter: str
    params: dict[str, Any]


@dataclass(frozen=True)
class Timeouts:
    run_wall_s: float  # whole run, wall clock
    telemetry_s: float  # first telemetry sample, adapter clock


@dataclass(frozen=True)
class Step:
    index: int
    id: str
    action: str
    params: dict[str, Any]


@dataclass(frozen=True)
class AssertionSpec:
    id: str
    type: str
    params: dict[str, Any]
    requirements: tuple[str, ...]
    note: str = ""  # free text copied into the report, e.g. how much of a requirement this covers


@dataclass(frozen=True)
class Spec:
    name: str
    description: str
    vehicle: VehicleSpec
    timeouts: Timeouts
    mission: tuple[Step, ...]
    assertions: tuple[AssertionSpec, ...]
    raw: dict[str, Any]
    sha256: str
    path: str | None = None

    @property
    def requirement_ids(self) -> tuple[str, ...]:
        seen: dict[str, None] = {}
        for a in self.assertions:
            for r in a.requirements:
                seen.setdefault(r, None)
        return tuple(seen)


# --- parsing ------------------------------------------------------------------

def _mapping(value: Any, where: str) -> dict:
    if not isinstance(value, dict):
        raise SpecError(f"{where} must be a mapping")
    return dict(value)


def _keys(raw: dict, allowed: set[str], where: str) -> None:
    unknown = set(raw) - allowed
    if unknown:
        raise SpecError(f"{where}: unknown key(s) {sorted(map(str, unknown))}; allowed: {sorted(allowed)}")


def _number(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SpecError(f"{where} must be a number, got {value!r}")
    value = float(value)
    if not math.isfinite(value):
        raise SpecError(f"{where} must be finite, got {value!r}")
    return value


def _name(value: Any, where: str) -> str:
    if not isinstance(value, str) or not NAME_RE.fullmatch(value):
        raise SpecError(f"{where} must match {NAME_RE.pattern}, got {value!r}")
    return value


def _polygon(value: Any, where: str) -> tuple[tuple[float, float], ...]:
    if not isinstance(value, list) or len(value) < 3:
        raise SpecError(f"{where} must be a list of at least 3 [north_m, east_m] pairs")
    points = []
    for i, p in enumerate(value):
        if not isinstance(p, list) or len(p) != 2:
            raise SpecError(f"{where}[{i}] must be a [north_m, east_m] pair")
        points.append((_number(p[0], f"{where}[{i}][0]"), _number(p[1], f"{where}[{i}][1]")))
    return tuple(points)


def _params(raw: dict, table: dict[str, ParamSpec], where: str) -> dict[str, Any]:
    _keys(raw, set(table), where)
    out: dict[str, Any] = {}
    for key, spec in table.items():
        label = f"{where}: {key}"
        if key not in raw:
            if spec.required:
                raise SpecError(f"{label} is required")
            if spec.default is not None:
                out[key] = spec.default
            continue
        value = raw[key]
        if spec.kind is Polygon:
            value = _polygon(value, label)
        elif spec.kind is float:
            value = _number(value, label)
            if (spec.lo is not None and value < spec.lo) or (spec.hi is not None and value > spec.hi):
                raise SpecError(f"{label} must be within [{spec.lo}, {spec.hi}], got {value}")
        if spec.choices is not None and value not in spec.choices:
            raise SpecError(f"{label} must be one of {list(spec.choices)}")
        out[key] = value
    return out


def _vehicle(raw: Any) -> VehicleSpec:
    raw = _mapping(raw, "vehicle")
    adapter = raw.pop("adapter", None)
    if adapter not in ADAPTERS:
        raise SpecError(f"vehicle: unsupported adapter {adapter!r}; supported: {sorted(ADAPTERS)}")
    return VehicleSpec(adapter, _params(raw, ADAPTERS[adapter], f"vehicle ({adapter})"))


def _timeouts(raw: Any) -> Timeouts:
    raw = _mapping(raw if raw is not None else {}, "timeouts")
    p = _params(raw, {"run_wall_s": _f(0.1, 86400.0, default=60.0), "telemetry_s": _f(0.01, 600.0, default=2.0)},
                "timeouts")
    return Timeouts(p["run_wall_s"], p["telemetry_s"])


def _mission(raw: Any) -> tuple[Step, ...]:
    if not isinstance(raw, list) or not raw:
        raise SpecError("mission must be a non-empty list of steps")
    steps, ids = [], set()
    condition = (False, False)  # (armed, airborne)
    for i, entry in enumerate(raw):
        where = f"mission[{i}]"
        entry = _mapping(entry, where)
        action = entry.pop("action", None)
        if action not in ACTIONS:
            raise SpecError(f"{where}: unknown action {action!r}; known: {sorted(ACTIONS)}")
        where = f"mission[{i}] ({action})"
        step_id = _name(entry.pop("id", f"{i}_{action}"), f"{where}: id")
        if step_id in ids:
            raise SpecError(f"{where}: duplicate step id {step_id!r}")
        ids.add(step_id)
        params = _params(entry, ACTIONS[action], where)
        needs, leads_to = _SEQUENCE[action]
        if condition != needs:
            armed, airborne = condition
            raise SpecError(
                f"{where}: not allowed while {'armed' if armed else 'disarmed'} and "
                f"{'airborne' if airborne else 'on the ground'}"
            )
        condition = leads_to
        steps.append(Step(i, step_id, action, params))
    return tuple(steps)


def _assertions(raw: Any, known_requirements: set[str] | None) -> tuple[AssertionSpec, ...]:
    if not isinstance(raw, list) or not raw:
        raise SpecError("assertions must be a non-empty list: a run with nothing to check cannot pass")
    out, ids = [], set()
    for i, entry in enumerate(raw):
        where = f"assertions[{i}]"
        entry = _mapping(entry, where)
        atype = entry.pop("type", None)
        if atype not in ASSERTIONS:
            raise SpecError(f"{where}: unknown assertion type {atype!r}; known: {sorted(ASSERTIONS)}")
        aid = _name(entry.pop("id", None), f"{where} ({atype}): id")
        where = f"assertions[{i}] ({aid})"
        if aid in ids:
            raise SpecError(f"{where}: duplicate assertion id")
        ids.add(aid)
        reqs = entry.pop("requirements", [])
        if not isinstance(reqs, list) or not all(isinstance(r, str) and REQ_RE.fullmatch(r) for r in reqs):
            raise SpecError(f"{where}: requirements must be a list of ids like REQ-GEOFENCE-001")
        if len(set(reqs)) != len(reqs):
            raise SpecError(f"{where}: duplicate requirement id")
        if known_requirements is not None:
            unknown = [r for r in reqs if r not in known_requirements]
            if unknown:
                raise SpecError(f"{where}: requirement(s) {unknown} are not in requirements/registry.yaml")
        note = entry.pop("note", "")
        if not isinstance(note, str):
            raise SpecError(f"{where}: note must be a string")
        params = _params(entry, ASSERTIONS[atype], where)
        _check_assertion(atype, params, where)
        out.append(AssertionSpec(aid, atype, params, tuple(reqs), " ".join(note.split())))
    return tuple(out)


def _check_assertion(atype: str, params: dict[str, Any], where: str) -> None:
    if atype == "max_speed" and not params:
        raise SpecError(f"{where}: give horizontal_mps, vertical_mps or both")
    if atype == "geofence":
        problem = geofence_config(params, "spec-validation").problems()
        if problem:
            raise SpecError(f"{where}: invalid keep-in: {problem}")


def geofence_config(params: dict[str, Any], source: str) -> GeofenceConfig:
    """The keep-in in mission_geofence's own form, so the spec and the mission code
    classify positions with the same function."""
    return GeofenceConfig(
        source=source,
        polygon_ned=params["polygon_ned"],
        altitude_frame=AltitudeFrame.LOCAL,
        min_altitude_m=params["min_altitude_m"],
        max_altitude_m=params["max_altitude_m"],
    )


def _faults(raw: Any, adapter: str) -> None:
    if raw is None:
        return
    if not isinstance(raw, list):
        raise SpecError("faults must be a list")
    supported = SUPPORTED_FAULTS[adapter]
    for i, entry in enumerate(raw):
        ftype = entry.get("type") if isinstance(entry, dict) else None
        if ftype not in supported:
            raise SpecError(
                f"faults[{i}]: unsupported fault {ftype!r} for adapter {adapter!r}; "
                f"supported: {sorted(supported) or 'none'}"
            )


class _StrictLoader(yaml.SafeLoader):
    """SafeLoader that rejects a key given twice in one mapping (PyYAML keeps the last)."""


def _no_duplicates(loader: _StrictLoader, node: yaml.MappingNode, deep: bool = False) -> dict:
    seen = set()
    for key_node, _ in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in seen:
            raise SpecError(f"duplicate key {key!r} at line {key_node.start_mark.line + 1}")
        seen.add(key)
    return loader.construct_mapping(node, deep)


_StrictLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _no_duplicates)

TOP_LEVEL = {"schema", "name", "description", "vehicle", "timeouts", "mission", "faults", "assertions"}


def parse_spec_text(
    text: str, known_requirements: set[str] | None = None, path: str | None = None
) -> Spec:
    try:
        doc = yaml.load(text, Loader=_StrictLoader)  # SafeLoader: no Python object tags
    except yaml.YAMLError as exc:
        raise SpecError(f"invalid YAML: {exc}") from exc
    doc = _mapping(doc, "spec")
    _keys(doc, TOP_LEVEL, "spec")
    if doc.get("schema") != SCHEMA_VERSION or isinstance(doc.get("schema"), bool):
        raise SpecError(f"spec: schema must be {SCHEMA_VERSION}")
    for key in ("name", "vehicle", "mission", "assertions"):
        if key not in doc:
            raise SpecError(f"spec: {key} is required")
    description = doc.get("description", "")
    if not isinstance(description, str):
        raise SpecError("spec: description must be a string")
    vehicle = _vehicle(doc["vehicle"])
    _faults(doc.get("faults"), vehicle.adapter)
    return Spec(
        name=_name(doc["name"], "spec: name"),
        description=" ".join(description.split()),
        vehicle=vehicle,
        timeouts=_timeouts(doc.get("timeouts")),
        mission=_mission(doc["mission"]),
        assertions=_assertions(doc["assertions"], known_requirements),
        raw=doc,
        sha256=hashlib.sha256(text.encode()).hexdigest(),
        path=path,
    )


def load_spec(path: str | Path, known_requirements: set[str] | None = None) -> Spec:
    path = Path(path)
    return parse_spec_text(path.read_text(), known_requirements, str(path))
