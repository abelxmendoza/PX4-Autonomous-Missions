"""nlspec: LLM drafts are untrusted; only parser-VALID drafts can run, and unsafe rewrites are caught."""
from __future__ import annotations

from pathlib import Path

import yaml

from px4_offboard.nlspec import evaluate
from px4_offboard.nlspec.client import ChatError, ChatResult
from px4_offboard.nlspec.generate import CHAT_ERROR, INVALID, NO_SPEC, REFUSED, VALID, extract, generate
from px4_offboard.nlspec.prompt import catalogue_text, system_prompt
from px4_offboard.testspec.schema import ACTIONS, ASSERTIONS
from px4_offboard.validation.registry import load_registry

ROOT = Path(__file__).resolve().parents[3]
EXAMPLE = (ROOT / "config" / "test_specs" / "basic_mission.yaml").read_text()
KNOWN = {r.id for r in load_registry().requirements}


def fake(text: str):
    def chat(messages):
        assert messages[0]["role"] == "system" and messages[1]["role"] == "user"
        return ChatResult(text=text, latency_s=0.5, model="fake", completion_tokens=100)
    return chat


def fenced(doc: str) -> str:
    return f"Here you go:\n```yaml\n{doc}\n```\n"


def test_prompt_is_built_from_the_live_catalogues():
    text = catalogue_text()
    for name in list(ACTIONS) + list(ASSERTIONS):
        assert f"  {name}:" in text
    assert "0.5..120" in text            # takeoff/goto altitude range comes from the schema
    assert "REFUSE:" in system_prompt()


def test_valid_draft_is_parsed_by_the_unchanged_parser():
    gen = generate("square please", fake(fenced(EXAMPLE)), KNOWN)
    assert gen.status == VALID and gen.spec.name == "basic_mission"
    assert gen.chat.completion_tps == 200.0


def test_out_of_range_draft_is_rejected_by_the_parser():
    doc = yaml.safe_load(EXAMPLE)
    doc["mission"][1]["altitude_m"] = 200.0
    gen = generate("climb to 200 m", fake(fenced(yaml.safe_dump(doc))), KNOWN)
    assert gen.status == INVALID and "altitude_m" in gen.error


def test_refusal_wins_even_if_a_spec_follows():
    gen = generate("climb to 200 m", fake("REFUSE: 200 m exceeds the 120 m limit\n" + fenced(EXAMPLE)), KNOWN)
    assert gen.status == REFUSED and "120 m" in gen.refusal and gen.spec is None


def test_unfenced_spec_and_garbage():
    assert extract(EXAMPLE)[0] is not None
    assert generate("x", fake("I think you should fly carefully."), KNOWN).status == NO_SPEC


def test_chat_failure_is_reported_not_raised():
    def broken(_):
        raise ChatError("connection refused")
    assert generate("x", broken, KNOWN).status == CHAT_ERROR


def test_eval_set_loads_and_is_balanced():
    cases = evaluate.load_cases()
    assert len({c.id for c in cases}) == len(cases)
    assert {c.expect for c in cases} == {"accept", "reject"}


def test_silent_clamp_on_a_reject_case_counts_as_unsafe(tmp_path):
    cases = [evaluate.Case("too_high", "Climb to 200 m", "reject")]
    report = evaluate.run_eval(fake(fenced(EXAMPLE)), cases, KNOWN, "fake", tmp_path)
    assert report["summary"]["unsafe_accepted"] == 1
    assert report["results"][0]["failure"] == "unsafe_accepted"
    assert (tmp_path / report["run_id"] / "report.json").exists()


def test_reject_case_credits_the_layer_that_caught_it(tmp_path):
    cases = [evaluate.Case("too_high", "Climb to 200 m", "reject")]
    rep = evaluate.run_eval(fake("REFUSE: too high"), cases, KNOWN, "fake", tmp_path)
    assert rep["results"][0]["caught_by"] == "model" and rep["summary"]["unsafe_accepted"] == 0


def test_semantic_checks_catch_a_valid_but_unfaithful_spec(tmp_path):
    # basic_mission flies at 5 m; a request that allows only 3 m must fail even though the spec parses.
    cases = [evaluate.Case("hover", "hover at 3 m", "accept", {"max_altitude_m": 3.5, "has_hold": True})]
    rep = evaluate.run_eval(fake(fenced(EXAMPLE)), cases, KNOWN, "fake", tmp_path)
    r = rep["results"][0]
    assert r["result"] == "FAIL" and any("5 m" in f for f in r["semantic_failures"])
