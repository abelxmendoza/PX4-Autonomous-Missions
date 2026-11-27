"""Draft a spec from a request and validate it with the unchanged strict parser."""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Callable

from px4_offboard.testspec.schema import Spec, SpecError, parse_spec_text

from .client import ChatError, ChatResult
from .prompt import messages

ChatFn = Callable[[list[dict]], ChatResult]

YAML_BLOCK = re.compile(r"```(?:yaml|yml)?[ \t]*\n(.*?)```", re.S | re.I)
REFUSE_LINE = re.compile(r"^\s*REFUSE:\s*(.+?)\s*$", re.M)

# Outcomes. Only VALID can ever reach the runner.
VALID = "VALID"          # parsed by the strict schema
INVALID = "INVALID"      # model wrote a spec; the parser rejected it
REFUSED = "REFUSED"      # model declined
NO_SPEC = "NO_SPEC"      # neither a spec nor a refusal
CHAT_ERROR = "CHAT_ERROR"


@dataclass
class Generation:
    request: str
    status: str
    raw_text: str = ""
    yaml_text: str | None = None
    spec: Spec | None = None
    error: str | None = None
    refusal: str | None = None
    chat: ChatResult | None = None


def extract(text: str) -> tuple[str | None, str | None]:
    """Return (yaml_text, refusal). A refusal wins: if the model refuses, nothing runs."""
    refuse = REFUSE_LINE.search(text)
    if refuse:
        return None, refuse.group(1)
    block = YAML_BLOCK.search(text)
    if block:
        return block.group(1), None
    # Unfenced: accept only if the first non-comment, non-blank line is the schema key.
    lines = text.strip().splitlines()
    first = next((ln for ln in lines if ln.strip() and not ln.lstrip().startswith("#")), "")
    if first.startswith("schema:"):
        return "\n".join(lines) + "\n", None
    return None, None


def generate(request: str, chat: ChatFn, known_requirements: set[str] | None = None) -> Generation:
    try:
        result = chat(messages(request))
    except ChatError as exc:
        return Generation(request, CHAT_ERROR, error=str(exc))
    gen = Generation(request, NO_SPEC, raw_text=result.text, chat=result)
    yaml_text, refusal = extract(result.text)
    if refusal is not None:
        gen.status, gen.refusal = REFUSED, refusal
        return gen
    if yaml_text is None:
        gen.error = "response contained neither a ```yaml block nor a REFUSE line"
        return gen
    gen.yaml_text = yaml_text
    try:
        gen.spec = parse_spec_text(yaml_text, known_requirements, path="<llm>")
        gen.status = VALID
    except SpecError as exc:
        gen.status, gen.error = INVALID, str(exc)
    return gen
