"""System prompt built from the live schema catalogues, so it cannot drift from the parser."""

from __future__ import annotations

from pathlib import Path

from px4_offboard.testspec.schema import ACTIONS, ADAPTERS, ASSERTIONS, SCHEMA_VERSION, Polygon

ROOT = Path(__file__).resolve().parents[4]
EXAMPLE_SPEC = ROOT / "config" / "test_specs" / "basic_mission.yaml"


def _param_line(name: str, p) -> str:
    if p.kind is Polygon:
        rng = "list of >= 3 [north_m, east_m] pairs"
    else:
        rng = f"{p.lo:g}..{p.hi:g}"
    tag = "required" if p.required else (f"default {p.default:g}" if p.default is not None else "optional")
    return f"    - {name}: {rng} ({tag})"


def catalogue_text() -> str:
    lines = ["ACTIONS (mission steps, in this legal order: arm -> takeoff -> goto/hold ... -> land -> disarm):"]
    for action, params in ACTIONS.items():
        lines.append(f"  {action}:" + ("" if params else " (no parameters)"))
        lines += [_param_line(n, p) for n, p in params.items()]
    lines.append("ASSERTIONS (checked against recorded telemetry; give each an `id`):")
    for atype, params in ASSERTIONS.items():
        lines.append(f"  {atype}:" + ("" if params else " (no parameters)"))
        lines += [_param_line(n, p) for n, p in params.items()]
    lines.append(f"VEHICLE ADAPTERS: {', '.join(ADAPTERS)} (use `mock`)")
    return "\n".join(lines)


def system_prompt() -> str:
    example = EXAMPLE_SPEC.read_text() if EXAMPLE_SPEC.exists() else ""
    return f"""You convert a drone operator's plain-English request into a YAML test spec
for a PX4 autonomy verification harness. Coordinates are local NED metres from the
take-off point: north_m forward, east_m right, altitude_m is height above the origin.

Output EXACTLY ONE of:
  1. A single ```yaml fenced block containing the complete spec, and nothing else.
  2. A single line `REFUSE: <short reason>` if the request cannot be expressed
     faithfully with the catalogue below (a value outside the allowed range, an
     action that does not exist, or anything unsafe or ambiguous).

Rules:
- Never silently change what the operator asked for. If a number is out of range,
  REFUSE; do not clamp it.
- schema must be {SCHEMA_VERSION}; name is lower_snake_case; vehicle.adapter is mock.
- The mission must start with arm and takeoff and end with land and disarm unless the
  operator says otherwise.
- Always include assertions that reflect the request: a geofence that contains every
  waypoint with a few metres of margin, a max_altitude a little above the highest
  altitude flown, a max_speed matching the commanded speeds, and final_disarmed.
- Use only the keys listed in the catalogue. No comments are needed.

{catalogue_text()}

EXAMPLE SPEC:
```yaml
{example.strip()}
```
"""


def messages(request: str) -> list[dict]:
    return [
        {"role": "system", "content": system_prompt()},
        {"role": "user", "content": request.strip()},
    ]
