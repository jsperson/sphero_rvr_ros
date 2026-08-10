"""The instruction loop, driven by canned model transcripts. No network, no ROS.

`run_instruction` is the control flow that decides how many times a misbehaving
model gets to try again and when an instruction stops. Those decisions cost money
and, for goto, motion — so they are tested against scripted models rather than
discovered live.

`run_instruction` lives in the pure core precisely so this file needs no ROS: the
runner here is a stand-in that records calls and returns canned tool results, which
exercises every branch. The ROS execution path is covered separately against the
fake-server harness.
"""

import json

import pytest

from sphero_rvr_core.task_agent import Budget, run_instruction


class ScriptedModel:
    """Returns the next canned reply, and records the prompts it was given."""

    def __init__(self, *replies):
        self.replies = list(replies)
        self.prompts = []

    def __call__(self, system, prompt):
        self.prompts.append(prompt)
        if not self.replies:
            raise AssertionError("the loop asked the model more times than scripted")
        return self.replies.pop(0)


class RecordingRunner:
    def __init__(self, results=None):
        self.calls = []
        self.results = list(results or [])

    def run(self, tool, args):
        self.calls.append((tool, args))
        if self.results:
            return self.results.pop(0)
        return json.dumps({"ok": True, "tool": tool})


def transcript():
    lines = []
    return lines, lines.append


def test_single_tool_then_say():
    model = ScriptedModel(
        '{"tool": "observe", "args": {}}',
        '{"say": "I saw a shoe."}',
    )
    runner = RecordingRunner()
    lines, out = transcript()
    run_instruction("look around", model, runner, Budget(8), out=out)
    assert runner.calls == [("observe", {})]
    assert any("I saw a shoe." in line for line in lines)


def test_multi_step_instruction_threads_results_back_to_the_model():
    model = ScriptedModel(
        '{"tool": "query_semantic_map", "args": {"label": "shoe"}}',
        '{"tool": "goto", "args": {"x": 1.0, "y": 0.0}}',
        '{"say": "Arrived at the shoe."}',
    )
    runner = RecordingRunner([
        json.dumps({"ok": True, "count": 1,
                    "objects": [{"label": "pink shoe", "x": 1.0, "y": 0.0}]}),
        json.dumps({"ok": True, "message": "arrived"}),
    ])
    lines, out = transcript()
    run_instruction("go to the shoe", model, runner, Budget(8), out=out)
    assert [c[0] for c in runner.calls] == ["query_semantic_map", "goto"]
    # The second prompt must contain the first tool's result, or the model is
    # deciding blind and the loop is theatre.
    assert "pink shoe" in model.prompts[1]


def test_hallucinated_tool_is_reprompted_once_and_can_self_correct():
    model = ScriptedModel(
        '{"tool": "teleport", "args": {"x": 9}}',
        '{"tool": "observe", "args": {}}',
        '{"say": "done"}',
    )
    runner = RecordingRunner()
    lines, out = transcript()
    run_instruction("look", model, runner, Budget(8), out=out)
    assert runner.calls == [("observe", {})], "the invented tool must never execute"
    assert any("[reprompt]" in line for line in lines)
    # The correction has to reach the model, or a reprompt is just a retry.
    assert "teleport" in model.prompts[1]


def test_two_bad_replies_in_a_row_end_the_instruction():
    """A model that cannot follow the contract will not start on attempt five, and
    every attempt is a paid call."""
    model = ScriptedModel(
        "I will drive forward.",
        "Definitely driving forward now.",
    )
    runner = RecordingRunner()
    lines, out = transcript()
    run_instruction("drive", model, runner, Budget(8), out=out)
    assert runner.calls == []
    assert any("did not follow the tool contract" in line for line in lines)


def test_the_reprompt_counter_resets_after_a_good_reply():
    """One bad reply early must not make a later, unrelated bad reply fatal."""
    model = ScriptedModel(
        "prose",                                  # bad -> reprompt
        '{"tool": "observe", "args": {}}',        # good -> counter resets
        "more prose",                             # bad -> reprompt again, not fatal
        '{"say": "ok"}',
    )
    runner = RecordingRunner()
    lines, out = transcript()
    run_instruction("look", model, runner, Budget(8), out=out)
    assert runner.calls == [("observe", {})]
    assert sum("[reprompt]" in line for line in lines) == 2


def test_budget_stops_a_model_that_never_finishes():
    model = ScriptedModel(*(['{"tool": "observe", "args": {}}'] * 10))
    runner = RecordingRunner()
    lines, out = transcript()
    run_instruction("look forever", model, runner, Budget(max_tool_calls=3), out=out)
    assert len(runner.calls) == 3
    assert any("[budget]" in line for line in lines)


def test_a_failed_tool_result_is_fed_back_verbatim_for_self_correction():
    """The envelope refusal path: task_node says WHY, and that reason must reach the
    model or it cannot correct itself. This is why tool_result carries prose."""
    refusal = json.dumps({"ok": False,
                          "message": "goal is 49.00 m away, beyond the 5.00 m envelope"})
    model = ScriptedModel(
        '{"tool": "goto", "args": {"x": 50.0, "y": 0.0}}',
        '{"tool": "goto", "args": {"x": 2.0, "y": 0.0}}',
        '{"say": "Moved as far as allowed."}',
    )
    runner = RecordingRunner([refusal, json.dumps({"ok": True, "message": "arrived"})])
    lines, out = transcript()
    run_instruction("drive 50 metres", model, runner, Budget(8), out=out)
    assert "beyond the 5.00 m envelope" in model.prompts[1]
    assert runner.calls[1] == ("goto", {"x": 2.0, "y": 0.0})


def test_exhausted_budget_reports_rather_than_silently_stopping():
    model = ScriptedModel('{"tool": "observe", "args": {}}')
    runner = RecordingRunner()
    lines, out = transcript()
    b = Budget(max_tool_calls=1)
    run_instruction("look", model, runner, b, out=out)
    assert b.exhausted
    assert any("[budget] stopping after 1 tool calls" in line for line in lines)
