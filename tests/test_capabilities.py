"""Capability reporting (design_capability_reporting_2026-08-20), pure half.

The wire shape, the client merge, the preamble rendering — all without ROS. The
predicate-coverage test imports the node and therefore runs only where rclpy
exists (the Pi); it is the loud failure for a new tool added without a
predicate.
"""

import json

import pytest

from sphero_rvr_core.task_agent import TOOL_SCHEMAS, availability_note
from sphero_rvr_core.task_tools import assemble_capabilities, merge_capabilities


def _report(**overrides):
    predicates = {name: (True, None) for name in TOOL_SCHEMAS}
    predicates.update(overrides)
    return assemble_capabilities(predicates, "2026-08-20T23:00:00")


# ---------------------------------------------------------------- assembly

def test_assembly_shape_and_stamp():
    data = json.loads(_report())
    assert data["ok"] is True and data["tool"] == "capabilities"
    assert data["stamp"] == "2026-08-20T23:00:00"          # the consensus pin
    assert set(data["tools"]) == set(TOOL_SCHEMAS)
    assert all(info == {"ready": True} for info in data["tools"].values())


def test_assembly_not_ready_must_say_why():
    data = json.loads(_report(observe=(False, "semantic_map not running")))
    assert data["tools"]["observe"] == {"ready": False,
                                        "why": "semantic_map not running"}
    # a ready tool never carries a why -- explanation for working tools is noise
    assert "why" not in data["tools"]["goto"]
    # an unexplained refusal still says SOMETHING rather than nothing
    data = json.loads(_report(turn=(False, None)))
    assert data["tools"]["turn"]["why"] == "unavailable (no reason given)"


# ---------------------------------------------------------------- merge

def test_merge_demotes_with_reason():
    avail = {name: True for name in TOOL_SCHEMAS}
    merged, reasons = merge_capabilities(
        avail, _report(look_and_recognize=(False, "recognition node not running")))
    assert merged["look_and_recognize"] is False
    assert reasons == {"look_and_recognize": "recognition node not running"}
    assert merged["goto"] is True


def test_merge_never_promotes():
    # the exists-probe found it missing; the report cannot resurrect it
    avail = {"observe": False, "goto": True}
    merged, _ = merge_capabilities(avail, _report())
    assert merged["observe"] is False


def test_merge_degrades_on_missing_or_malformed_report():
    avail = {"goto": True, "observe": True}
    for message in (None, "", "not json", '{"ok": true}', '{"tools": 3}'):
        merged, reasons = merge_capabilities(avail, message)
        assert merged == avail and reasons == {}
    # unknown tools in the report are ignored, not added
    merged, _ = merge_capabilities(avail, _report(bogus=(False, "x")))
    assert set(merged) == {"goto", "observe"}


# ---------------------------------------------------------------- preamble

def test_note_renders_reasons_inline():
    note = availability_note(
        {"observe": False, "goto": True, "turn": False},
        {"observe": "semantic_map not running"})
    assert "observe (semantic_map not running)" in note
    assert "turn" in note and "turn (" not in note
    assert "goto" not in note


def test_note_unchanged_without_reasons():
    # older task_nodes: exactly the certified preamble, byte for byte
    assert availability_note({"observe": False}) == (
        "Unavailable in this configuration (do not call): observe.\n\n")
    assert availability_note({"observe": True}) == ""


# ---------------------------------------------------------------- coverage

def test_every_tool_has_a_predicate():
    """A tool added to TOOL_SCHEMAS without a capability predicate must fail
    LOUDLY here (ratified cert plan). Needs rclpy → runs on the Pi."""
    pytest.importorskip("rclpy")
    import ast, inspect, textwrap  # noqa: E401
    from sphero_rvr_driver import task_node

    source = textwrap.dedent(
        inspect.getsource(task_node.TaskNode._capability_predicates))
    keys = {node.value for node in ast.walk(ast.parse(source))
            if isinstance(node, ast.Constant) and isinstance(node.value, str)}
    missing = set(TOOL_SCHEMAS) - keys
    assert not missing, f"tools without capability predicates: {sorted(missing)}"
