"""Machine-readable (JSON) and human-readable (Markdown) validation reports,
with Requirement -> Test -> Evidence -> Result traceability."""

from __future__ import annotations

from typing import Any

from .evaluate import RequirementResult, gate

SCHEMA = 1


def _verification(v) -> dict[str, Any]:
    return {
        "kind": v.kind,
        "ref": v.ref,
        "status": v.status,
        "detail": v.detail,
        "evidence": list(v.evidence),
        "children": [_verification(c) for c in v.children],
    }


def build_report(results: list[RequirementResult], meta: dict[str, Any] | None = None) -> dict[str, Any]:
    ok, problems = gate(results)
    count = lambda s: sum(1 for r in results if r.status == s)  # noqa: E731
    return {
        "schema": SCHEMA,
        "meta": meta or {},
        "summary": {
            "pass": count("PASS"),
            "fail": count("FAIL"),
            "partial": count("PARTIAL"),
            "not_run": count("NOT_RUN"),
            "known_open": sum(1 for r in results if r.known_open),
            "gate_ok": ok,
            "gate_problems": problems,
        },
        "requirements": [
            {
                "id": r.id,
                "title": r.title,
                "level": r.level,
                "description": r.requirement.description if r.requirement else "",
                "rationale": r.requirement.rationale if r.requirement else "",
                "threshold": r.requirement.threshold if r.requirement else "",
                "measurement": r.requirement.measurement if r.requirement else "",
                "legacy_ids": list(r.requirement.legacy_ids) if r.requirement else [],
                "result": r.status,
                "known_open": r.known_open,
                "verification": [_verification(v) for v in r.verifications],
                "tests": [{"id": ref, "outcome": outcome} for ref, outcome in r.tests],
            }
            for r in results
        ],
    }


def _cell(text: str) -> str:
    return str(text).replace("|", "\\|")


def to_markdown(report: dict[str, Any]) -> str:
    s = report["summary"]
    lines = [
        "# Validation report",
        "",
        f"**Gate: {'OK' if s['gate_ok'] else 'FAIL'}** -- "
        f"{s['pass']} PASS, {s['fail']} FAIL ({s['known_open']} declared known-open), "
        f"{s['partial']} PARTIAL, {s['not_run']} NOT_RUN.",
        "",
        "PASS means something was measured or executed and met the threshold. NOT_RUN means "
        "the verification needs a physical flight controller or a running PX4 SITL and was not "
        "available; it is never counted as passing.",
        "",
    ]
    if s["gate_problems"]:
        lines += ["## Gate problems", ""] + [f"- {p}" for p in s["gate_problems"]] + [""]
    lines += [
        "## Summary",
        "",
        "| ID | Requirement | Level | Threshold | Result |",
        "| --- | --- | --- | --- | --- |",
    ]
    for r in report["requirements"]:
        mark = f"**{r['result']}**" + (" (known open)" if r["known_open"] else "")
        lines.append(f"| {r['id']} | {_cell(r['title'])} | {r['level']} | {_cell(r['threshold'])} | {mark} |")
    lines += ["", "## Traceability: requirement -> test -> evidence -> result", ""]
    for r in report["requirements"]:
        lines += [f"### {r['id']} -- {r['title']}", ""]
        lines += [f"- **Result:** {r['result']}" + (f" -- known open: {r['known_open']}" if r["known_open"] else "")]
        lines += [f"- **Rationale:** {r['rationale']}", f"- **Threshold:** {r['threshold']}",
                  f"- **Measurement:** {r['measurement']}"]
        if r["legacy_ids"]:
            lines.append(f"- **Legacy verifier IDs:** {', '.join(r['legacy_ids'])}")
        for v in r["verification"]:
            ev = ", ".join(f"`{e}`" for e in v["evidence"]) or "none"
            lines.append(f"- **Verification ({v['kind']}) {v['ref']}:** {v['status']} -- {v['detail']} (evidence: {ev})")
        if r["tests"]:
            lines.append("- **Tests:**")
            lines += [f"  - `{t['id']}` -- {t['outcome']}" for t in r["tests"]]
        lines.append("")
    return "\n".join(lines) + "\n"
