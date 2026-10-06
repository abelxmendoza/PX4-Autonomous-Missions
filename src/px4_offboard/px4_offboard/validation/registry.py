"""Requirement registry loader (requirements/registry.yaml)."""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[4]
DEFAULT_PATH = ROOT / "requirements" / "registry.yaml"

KINDS = ("flight", "fault", "pytest", "external")
LEVELS = ("MUST", "SHOULD")
REQUIRED_TEXT = ("title", "description", "rationale", "threshold", "measurement")


class RegistryError(ValueError):
    pass


@dataclass(frozen=True)
class RegistryRequirement:
    id: str
    title: str
    level: str
    description: str
    rationale: str
    threshold: str
    measurement: str
    verification: tuple[dict[str, Any], ...]
    tests: tuple[str, ...] = ()
    legacy_ids: tuple[str, ...] = ()
    known_open: str | None = None


@dataclass(frozen=True)
class Registry:
    requirements: tuple[RegistryRequirement, ...]

    def get(self, req_id: str) -> RegistryRequirement:
        return next(r for r in self.requirements if r.id == req_id)


def _clean(text: Any) -> str:
    return " ".join(str(text).split())


def _parse_entry(raw: dict[str, Any]) -> RegistryRequirement:
    rid = raw.get("id", "?")
    if not isinstance(rid, str) or not re.fullmatch(r"REQ-[A-Z]+-\d{3}", rid):
        raise RegistryError(f"bad requirement id {rid!r}")
    for key in REQUIRED_TEXT:
        if not str(raw.get(key, "")).strip():
            raise RegistryError(f"{rid}: {key} is required")
    if raw.get("level") not in LEVELS:
        raise RegistryError(f"{rid}: level must be one of {LEVELS}")
    verification = raw.get("verification")
    if not isinstance(verification, list) or not verification:
        raise RegistryError(f"{rid}: verification must be a non-empty list")
    for v in verification:
        if not isinstance(v, dict) or v.get("kind") not in KINDS:
            raise RegistryError(f"{rid}: verification kind must be one of {KINDS}: {v!r}")
        if v["kind"] == "flight" and not (v.get("cases") and v.get("checks")):
            raise RegistryError(f"{rid}: flight verification needs cases and checks")
        if v["kind"] == "fault" and not (v.get("scenario") and v.get("faults")):
            raise RegistryError(f"{rid}: fault verification needs scenario and faults")
        if v["kind"] == "external" and not all(v.get(k) for k in ("requires", "results_file", "reason")):
            raise RegistryError(f"{rid}: external verification needs requires, results_file and reason")
    return RegistryRequirement(
        id=rid,
        title=_clean(raw["title"]),
        level=raw["level"],
        description=_clean(raw["description"]),
        rationale=_clean(raw["rationale"]),
        threshold=_clean(raw["threshold"]),
        measurement=_clean(raw["measurement"]),
        verification=tuple(verification),
        tests=tuple(raw.get("tests") or ()),
        legacy_ids=tuple(raw.get("legacy_ids") or ()),
        known_open=_clean(raw["known_open"]) if raw.get("known_open") else None,
    )


def parse_registry_text(text: str) -> Registry:
    try:
        doc = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise RegistryError(f"invalid YAML: {exc}") from exc
    if not isinstance(doc, dict) or doc.get("schema") != 1 or not isinstance(doc.get("requirements"), list):
        raise RegistryError("registry must be a mapping with schema: 1 and a requirements list")
    entries = [_parse_entry(r) for r in doc["requirements"]]
    ids = [e.id for e in entries]
    if len(ids) != len(set(ids)):
        raise RegistryError("duplicate requirement ids")
    return Registry(tuple(entries))


def load_registry(path: str | Path | None = None) -> Registry:
    return parse_registry_text(Path(path or DEFAULT_PATH).read_text())
