"""Fault scenario schema (YAML) and the catalogue of fault types.

Every fault type carries its own plain-language expected behaviour and default
machine-checkable expectations, so a scenario file can be as short as the
example in docs/FAULT_INJECTION.md and the evidence still states what was
expected. A scenario may override any expectation with an ``expect:`` block;
the override is recorded in the evidence, never silently applied.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

SENSOR = "sensor"
LINK = "link"


class ScenarioError(ValueError):
    pass


@dataclass(frozen=True)
class ParamSpec:
    kind: type
    required: bool = False
    default: Any = None
    choices: tuple | None = None
    lo: float | None = None
    hi: float | None = None


@dataclass(frozen=True)
class FaultType:
    domain: str
    description: str
    expected_text: str
    params: dict[str, ParamSpec] = field(default_factory=dict)
    expect: dict[str, Any] = field(default_factory=dict)  # default expectations
    instant: bool = False  # no duration (a one-shot event)


_SENSOR_PARAM = ParamSpec(str, default="vo", choices=("vo", "imu"))
_PROB = ParamSpec(float, required=True, lo=0.0, hi=1.0)

FAULT_TYPES: dict[str, FaultType] = {
    "vo_dropout": FaultType(
        SENSOR,
        "Stereo VO stops delivering any measurement.",
        "Fusion health drops once VO goes stale; the estimate dead-reckons on the IMU "
        "with bounded error growth; health and accuracy return after VO resumes.",
        expect={"must_detect": True, "detect_within_s": 1.0, "recover_within_s": 2.0, "max_err_growth_m": 2.0},
    ),
    "dropped_messages": FaultType(
        SENSOR,
        "A fraction of one sensor's messages never arrive.",
        "The estimate tolerates random loss with bounded error growth and recovers when loss stops.",
        {"sensor": _SENSOR_PARAM, "probability": _PROB},
        {"recover_within_s": 2.0, "max_err_growth_m": 1.0},
    ),
    "delayed_messages": FaultType(
        SENSOR,
        "One sensor's messages arrive late but carry their original timestamps.",
        "Bounded error growth from the latency; recovery once the delay ends.",
        {"sensor": _SENSOR_PARAM, "delay_s": ParamSpec(float, required=True, lo=0.0, hi=5.0)},
        {"recover_within_s": 2.0, "max_err_growth_m": 1.5},
    ),
    "timestamp_jitter": FaultType(
        SENSOR,
        "Gaussian jitter on one sensor's message timestamps.",
        "Bounded error growth (integration intervals are wrong by the jitter); recovery after.",
        {"sensor": _SENSOR_PARAM, "std_s": ParamSpec(float, required=True, lo=0.0, hi=1.0)},
        {"recover_within_s": 2.0, "max_err_growth_m": 1.0},
    ),
    "corrupted_measurement": FaultType(
        SENSOR,
        "Random spikes added to one sensor's measurements.",
        "Spikes are rejected or absorbed without the estimate diverging; recovery after.",
        {
            "sensor": _SENSOR_PARAM,
            "probability": _PROB,
            "magnitude": ParamSpec(float, required=True, lo=0.0, hi=1000.0),
        },
        {"recover_within_s": 2.0, "max_err_growth_m": 2.0},
    ),
    "frozen_sensor": FaultType(
        SENSOR,
        "One sensor keeps repeating its last value while still 'arriving'.",
        "A stuck sensor is detected (health drops or measurements are rejected) rather than "
        "silently trusted; recovery after.",
        {"sensor": _SENSOR_PARAM},
        {"must_detect": True, "detect_within_s": 1.0, "recover_within_s": 2.0, "max_err_growth_m": 2.0},
    ),
    "estimator_reset": FaultType(
        SENSOR,
        "The estimator is re-initialised mid-flight (state lost, covariance reset).",
        "Position continuity is kept and velocity re-converges once VO re-anchors it.",
        expect={"recover_within_s": 3.0, "max_err_growth_m": 1.5},
        instant=True,
    ),
    "mavlink_packet_loss": FaultType(
        LINK,
        "A fraction of MAVLink frames from the autopilot are lost.",
        "The link stays alive, telemetry rate falls roughly in proportion, and full rate "
        "returns promptly when loss stops.",
        {"probability": _PROB},
        {"must_stay_alive": True, "min_rx_rate_ratio": 0.4, "recover_within_s": 1.5},
    ),
    "comm_disconnect": FaultType(
        LINK,
        "The transport disappears (cable pulled) and later returns.",
        "The loss is detected immediately, the supervisor reconnects with bounded backoff "
        "and telemetry resumes.",
        expect={"must_detect": True, "detect_within_s": 1.0, "recover_within_s": 4.0},
    ),
}

EXPECT_KEYS = {
    "must_detect": bool,
    "detect_within_s": (int, float),
    "recover_within_s": (int, float),
    "max_err_growth_m": (int, float),
    "must_stay_alive": bool,
    "min_rx_rate_ratio": (int, float),
}


@dataclass(frozen=True)
class FaultSpec:
    type: str
    start_s: float
    duration_s: float
    params: dict[str, Any]
    expect: dict[str, Any]  # defaults merged with overrides
    expect_overrides: dict[str, Any]
    known_gap: str | None = None

    @property
    def end_s(self) -> float:
        return self.start_s + self.duration_s

    @property
    def info(self) -> FaultType:
        return FAULT_TYPES[self.type]


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    duration_s: float
    seed: int
    faults: tuple[FaultSpec, ...]
    background: tuple[FaultSpec, ...] = ()  # applied to baseline and fault runs alike


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ScenarioError(f"{label} must be a number, got {value!r}")
    return float(value)


def _parse_fault(raw: Any, index: int, scenario_duration: float) -> FaultSpec:
    where = f"fault #{index + 1}"
    if not isinstance(raw, dict):
        raise ScenarioError(f"{where} must be a mapping")
    raw = dict(raw)
    ftype = raw.pop("type", None)
    if ftype not in FAULT_TYPES:
        raise ScenarioError(f"{where}: unknown fault type {ftype!r}; known: {sorted(FAULT_TYPES)}")
    info = FAULT_TYPES[ftype]
    where = f"fault #{index + 1} ({ftype})"

    if "start_s" not in raw:
        raise ScenarioError(f"{where}: start_s is required")
    start = _number(raw.pop("start_s"), f"{where}: start_s")
    if start < 0:
        raise ScenarioError(f"{where}: start_s must be >= 0")

    if info.instant:
        duration = _number(raw.pop("duration_s", 0.0), f"{where}: duration_s")
    else:
        if "duration_s" not in raw:
            raise ScenarioError(f"{where}: duration_s is required")
        duration = _number(raw.pop("duration_s"), f"{where}: duration_s")
        if duration <= 0:
            raise ScenarioError(f"{where}: duration_s must be > 0")
    if start + duration > scenario_duration + 1e-9:
        raise ScenarioError(
            f"{where} ends after the scenario does ({start + duration} > {scenario_duration})"
        )

    known_gap = raw.pop("known_gap", None)
    overrides = raw.pop("expect", {}) or {}
    if not isinstance(overrides, dict):
        raise ScenarioError(f"{where}: expect must be a mapping")
    for key, value in overrides.items():
        if key not in EXPECT_KEYS:
            raise ScenarioError(f"{where}: unknown expectation {key!r}; known: {sorted(EXPECT_KEYS)}")
        if isinstance(value, bool) != (EXPECT_KEYS[key] is bool) or not isinstance(value, EXPECT_KEYS[key]):
            raise ScenarioError(f"{where}: expectation {key} has the wrong type")

    params: dict[str, Any] = {}
    for name, spec in info.params.items():
        if name in raw:
            value = raw.pop(name)
            if spec.kind is float:
                value = _number(value, f"{where}: {name}")
                if (spec.lo is not None and value < spec.lo) or (spec.hi is not None and value > spec.hi):
                    raise ScenarioError(f"{where}: {name} must be within [{spec.lo}, {spec.hi}]")
            if spec.choices is not None and value not in spec.choices:
                raise ScenarioError(f"{where}: {name} must be one of {list(spec.choices)}")
            params[name] = value
        elif spec.required:
            raise ScenarioError(f"{where}: {name} is required")
        else:
            params[name] = spec.default
    if raw:
        raise ScenarioError(f"{where}: unknown parameter(s) {sorted(raw)}")

    return FaultSpec(
        type=ftype,
        start_s=start,
        duration_s=duration,
        params=params,
        expect={**info.expect, **overrides},
        expect_overrides=dict(overrides),
        known_gap=known_gap,
    )


def load_scenario_text(text: str, default_name: str = "scenario") -> Scenario:
    try:
        doc = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ScenarioError(f"invalid YAML: {exc}") from exc
    if not isinstance(doc, dict):
        raise ScenarioError("scenario must be a mapping with a 'faults' list")
    allowed = {"name", "description", "duration_s", "seed", "faults", "background"}
    if set(doc) - allowed:
        raise ScenarioError(f"unknown top-level key(s) {sorted(set(doc) - allowed)}")
    if "faults" not in doc:
        raise ScenarioError("scenario needs a 'faults' list")
    faults = doc["faults"]
    if not isinstance(faults, list):
        raise ScenarioError("'faults' must be a list")
    if not faults:
        raise ScenarioError("scenario needs at least one fault")
    duration = _number(doc.get("duration_s", 60.0), "duration_s")
    if duration <= 0:
        raise ScenarioError("duration_s must be > 0")
    seed = doc.get("seed", 1)
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise ScenarioError("seed must be an integer")
    background = doc.get("background", []) or []
    if not isinstance(background, list):
        raise ScenarioError("'background' must be a list")
    return Scenario(
        background=tuple(_parse_fault(f, i, duration) for i, f in enumerate(background)),
        name=str(doc.get("name", default_name)),
        description=str(doc.get("description", "")),
        duration_s=duration,
        seed=seed,
        faults=tuple(_parse_fault(f, i, duration) for i, f in enumerate(faults)),
    )


def load_scenario(path: str | Path) -> Scenario:
    path = Path(path)
    return load_scenario_text(path.read_text(), default_name=path.stem)
