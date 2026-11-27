"""Natural language -> declarative test spec, with the LLM treated as untrusted input.

A language model drafts a test spec from a plain-English request. Nothing it
writes is trusted: the draft goes through the same strict parser as a
hand-written spec (``testspec.schema.parse_spec_text``) and, optionally, through
the same runner and assertion evaluators. The model may also refuse. See
docs/nl_mission_specs.md.
"""
