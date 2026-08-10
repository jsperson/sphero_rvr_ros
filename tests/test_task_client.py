"""End-to-end: a fake model drives the real client against the real task_node.

Everything is real except the model and the robot — real rclpy, real task_node with
its envelope, real action/service round trips, and the same fake Nav2/observe/objects
servers the node's own harness uses. The model is scripted, so the suite NEVER
touches the network: a test that calls a paid endpoint is a test that fails when the
network does, and its result would not be reproducible anyway.

This is the layer that proves the pieces fit. The contract logic is covered purely in
test_task_agent.py, the loop policy in test_task_client_loop.py, and the safety
boundary by source scan in test_task_node_safety.py.
"""

import json
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.executors import MultiThreadedExecutor  # noqa: E402

from sphero_rvr_core.task_agent import Budget, run_instruction  # noqa: E402
from sphero_rvr_driver.task_client import ToolRunner  # noqa: E402
from sphero_rvr_driver.task_node import TaskNode  # noqa: E402

from test_task_node import FakeWorld  # noqa: E402


class ScriptedModel:
    def __init__(self, *replies):
        self.replies = list(replies)
        self.prompts = []

    def __call__(self, system, prompt):
        self.prompts.append(prompt)
        if not self.replies:
            raise AssertionError("the loop asked the model more times than scripted")
        return self.replies.pop(0)


class Stack:
    def __init__(self, params=None):
        args = ["--ros-args"]
        for key, value in (params or {}).items():
            args += ["-p", f"{key}:={value}"]
        rclpy.init(args=args)
        self.world = FakeWorld()
        self.task = TaskNode()
        self.executor = MultiThreadedExecutor(num_threads=8)
        self.executor.add_node(self.world)
        self.executor.add_node(self.task)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()
        # The client spins itself inside run(), exactly as it does in production.
        self.runner = ToolRunner(timeout_s=20.0)

    def close(self):
        self.executor.shutdown(timeout_sec=5.0)
        for n in (self.runner, self.task, self.world):
            n.destroy_node()
        rclpy.shutdown()
        self._thread.join(timeout=5.0)


@pytest.fixture
def stack():
    s = Stack({"max_goal_distance_m": 5.0, "goal_timeout_s": 6.0})
    time.sleep(0.5)
    yield s
    s.close()


def run(stack, model, budget=8):
    lines = []
    run_instruction("test instruction", model, stack.runner, Budget(budget),
                    out=lines.append)
    return lines


def test_query_then_goto_then_finish(stack):
    """The multi-step shape the demo is meant to show: ask what is known, drive to
    it, report back — with the real envelope and real action round trips."""
    stack.world.publish_objects()
    time.sleep(0.5)
    model = ScriptedModel(
        '{"tool": "query_semantic_map", "args": {"label": "shoe"}}',
        '{"tool": "goto", "args": {"x": 1.0, "y": 0.0}}',
        '{"tool": "observe", "args": {}}',
        '{"say": "I drove to the pink shoe and had a look."}',
    )
    lines = run(stack, model)
    assert stack.world.nav_goals == [(1.0, 0.0)]
    assert stack.world.observe_calls == 1
    assert any("pink shoe" in p for p in model.prompts), \
        "the query result must reach the model"
    assert any("I drove to the pink shoe" in line for line in lines)


def test_out_of_envelope_goto_is_refused_by_the_node_and_self_corrected(stack):
    """The refusal has to survive the whole round trip — node envelope to tool
    result to prompt — or the model cannot correct itself."""
    model = ScriptedModel(
        '{"tool": "goto", "args": {"x": 50.0, "y": 0.0}}',
        '{"tool": "goto", "args": {"x": 2.0, "y": 0.0}}',
        '{"say": "That was too far, so I went as far as allowed."}',
    )
    run(stack, model)
    assert stack.world.nav_goals == [(2.0, 0.0)], \
        "the 50 m goal must never reach Nav2"
    assert "beyond the" in model.prompts[1], \
        "the envelope's reason must be fed back verbatim"


def test_hallucinated_tool_never_reaches_ros(stack):
    model = ScriptedModel(
        '{"tool": "set_motor_duty", "args": {"left": 255, "right": 255}}',
        '{"say": "I cannot do that."}',
    )
    run(stack, model)
    assert stack.world.nav_goals == []
    assert stack.world.observe_calls == 0


def test_query_arguments_do_not_leak_between_instructions(stack):
    """A stale filter from an earlier question must not silently narrow a later
    one. Observed for real while demoing the CLI: a leftover radius made a
    subsequent unfiltered query return a refusal."""
    stack.world.publish_objects()
    time.sleep(0.5)
    narrow = ScriptedModel(
        '{"tool": "query_semantic_map", "args": {"label": "backpack"}}',
        '{"say": "found it"}',
    )
    run(stack, narrow)
    wide = ScriptedModel(
        '{"tool": "query_semantic_map", "args": {}}',
        '{"say": "listed everything"}',
    )
    lines = run(stack, wide)
    result = [l for l in lines if l.startswith("[result]")][-1]
    payload = json.loads(result[len("[result] "):])
    assert payload["count"] == 2, "the previous label filter leaked into this query"


def test_a_failing_tool_does_not_abort_the_instruction(stack):
    """Nav2 aborting is information, not an exception: the model gets the typed
    failure and decides what to do."""
    stack.world.nav_mode = "abort"
    model = ScriptedModel(
        '{"tool": "goto", "args": {"x": 1.0, "y": 0.0}}',
        '{"say": "I could not get there."}',
    )
    lines = run(stack, model)
    assert any("ABORTED" in p for p in model.prompts[1:]), \
        "the typed failure must reach the model"
    assert any("could not get there" in line for line in lines)
