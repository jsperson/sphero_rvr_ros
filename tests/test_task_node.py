"""Rehearsal harness for the task node: the real node against fake servers.

Same pattern as `test_coverage_explorer_mission.py` -- real rclpy, real
MultiThreadedExecutor, real action/service machinery, with scriptable fakes standing
in for Nav2, the semantic_map `observe` service, and the `/semantic_map/objects`
publisher. No chassis, nothing motor-capable; the driver is absent, so even a
runaway goal moves nothing.

The source-scan safety boundary lives in `test_task_node_safety.py`, NOT here:
`pytest.importorskip` skips a whole module regardless of where it appears, so a
boundary test in this file would silently not run on a machine without ROS -- which
is where most commits are made. Caught by seeing this file report "1 skipped" when
it was supposed to run two ROS-free tests.
"""

import json
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from action_msgs.msg import GoalStatus  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav2_msgs.action import ComputePathToPose, NavigateToPose  # noqa: E402
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402

from sphero_rvr_driver.task_node import TaskNode  # noqa: E402


CANNED_OBJECTS = json.dumps({
    "objects": [
        {"label": "pink shoe", "x": 1.0, "y": 0.0, "confidence": 0.9, "count": 3,
         "first_seen": 0.0, "last_seen": 1.0},
        {"label": "backpack", "x": 4.0, "y": 0.0, "confidence": 0.8, "count": 2,
         "first_seen": 0.0, "last_seen": 1.0},
    ]
})


class FakeWorld(Node):
    """Fake Nav2 + observe service + semantic objects, scriptable per test."""

    def __init__(self):
        super().__init__("fake_world_tasks")
        cbg = ReentrantCallbackGroup()
        self.nav_mode = "succeed"        # "succeed" | "abort" | "hold"
        self.plannable = True
        self.observe_ok = True
        self.observe_message = "3 objects folded into the map"
        self.observe_calls = 0
        self.nav_goals = []
        self.nav_cancels = 0
        self.pose = (0.0, 0.0)

        self._nav_srv = ActionServer(
            self, NavigateToPose, "navigate_to_pose", self._nav_execute,
            goal_callback=lambda r: GoalResponse.ACCEPT,
            cancel_callback=lambda gh: CancelResponse.ACCEPT, callback_group=cbg,
        )
        self._plan_srv = ActionServer(
            self, ComputePathToPose, "compute_path_to_pose", self._plan_execute,
            goal_callback=lambda r: GoalResponse.ACCEPT,
            cancel_callback=lambda gh: CancelResponse.ACCEPT, callback_group=cbg,
        )
        self.create_service(Trigger, "observe", self._observe, callback_group=cbg)
        self._objects_pub = self.create_publisher(String, "/semantic_map/objects", 10)
        self._tf = TransformBroadcaster(self)
        # Switchable so a test can let the pose GO STALE: tf2 keeps serving the last
        # transform it heard, with an ever-growing stamp age that lookup_transform
        # will not mention. That silence is what move_relative refuses.
        self.tf_enabled = True
        self.create_timer(0.02, self._broadcast_tf, callback_group=cbg)

    def publish_objects(self, text=CANNED_OBJECTS):
        self._objects_pub.publish(String(data=text))

    def _broadcast_tf(self):
        if not self.tf_enabled:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = float(self.pose[0])
        t.transform.translation.y = float(self.pose[1])
        t.transform.rotation.w = 1.0
        self._tf.sendTransform(t)

    def _nav_execute(self, goal_handle):
        p = goal_handle.request.pose.pose.position
        self.nav_goals.append((p.x, p.y))
        if self.nav_mode == "succeed":
            self.pose = (p.x, p.y)
            time.sleep(0.05)
            goal_handle.succeed()
        elif self.nav_mode == "abort":
            goal_handle.abort()
        else:
            while not goal_handle.is_cancel_requested:
                time.sleep(0.02)
            self.nav_cancels += 1
            goal_handle.canceled()
        return NavigateToPose.Result()

    def _plan_execute(self, goal_handle):
        result = ComputePathToPose.Result()
        if self.plannable:
            from geometry_msgs.msg import PoseStamped
            result.path.poses = [PoseStamped(), PoseStamped()]
        goal_handle.succeed()
        return result

    def _observe(self, request, response):
        self.observe_calls += 1
        response.success = self.observe_ok
        response.message = self.observe_message
        return response


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
        self.client = Node("test_client")
        self.executor.add_node(self.client)
        self.goto = ActionClient(self.client, NavigateToPose, "task/goto")
        self.observe = self.client.create_client(Trigger, "task/observe")
        self.query = self.client.create_client(Trigger, "task/query_semantic_map")

    def call(self, client, timeout=10.0):
        assert client.wait_for_service(timeout_sec=5.0), "service never appeared"
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(0.02)
        assert future.done(), "service call timed out"
        return json.loads(future.result().message)

    def send_goto(self, x, y, timeout=20.0):
        assert self.goto.wait_for_server(timeout_sec=5.0), "task/goto never appeared"
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        send = self.goto.send_goal_async(goal)
        end = time.monotonic() + timeout
        while not send.done() and time.monotonic() < end:
            time.sleep(0.02)
        assert send.done(), "goto goal was never acknowledged"
        handle = send.result()
        if not handle.accepted:
            return None, None
        result_future = handle.get_result_async()
        while not result_future.done() and time.monotonic() < end:
            time.sleep(0.02)
        assert result_future.done(), "goto never returned a result"
        res = result_future.result()
        return res.status, json.loads(res.result.error_msg)

    def close(self):
        self.executor.shutdown(timeout_sec=5.0)
        for n in (self.client, self.task, self.world):
            n.destroy_node()
        rclpy.shutdown()
        self._thread.join(timeout=5.0)


@pytest.fixture
def stack(request):
    params = {"max_goal_distance_m": 5.0, "goal_timeout_s": 6.0, "plan_timeout_s": 2.0}
    params.update(getattr(request, "param", {}))
    s = Stack(params)
    time.sleep(0.5)   # let TF and discovery settle
    yield s
    s.close()


# --- goto -------------------------------------------------------------------

def test_goto_succeeds_and_reports_arrival(stack):
    status, payload = stack.send_goto(1.0, 0.5)
    assert status == GoalStatus.STATUS_SUCCEEDED
    assert payload["ok"] is True and payload["message"] == "arrived"
    assert stack.world.nav_goals == [(1.0, 0.5)]


def test_goto_forwards_the_pose_unchanged(stack):
    stack.send_goto(-1.25, 2.0)
    assert stack.world.nav_goals[-1] == pytest.approx((-1.25, 2.0))


def test_goto_aborted_downstream_returns_a_typed_failure_and_does_not_retry(stack):
    stack.world.nav_mode = "abort"
    status, payload = stack.send_goto(1.0, 0.0)
    assert status == GoalStatus.STATUS_ABORTED
    assert payload["ok"] is False and payload["status"] == "ABORTED"
    time.sleep(0.5)
    assert len(stack.world.nav_goals) == 1, "a failed goto must not retry-storm"


def test_goto_beyond_the_envelope_is_refused_without_reaching_nav2(stack):
    status, payload = stack.send_goto(50.0, 0.0)
    assert status == GoalStatus.STATUS_ABORTED
    assert "beyond the" in payload["message"]
    assert stack.world.nav_goals == [], "an out-of-envelope goal must never be sent"


def test_goto_refused_when_the_planner_finds_no_path(stack):
    stack.world.plannable = False
    status, payload = stack.send_goto(1.0, 0.0)
    assert status == GoalStatus.STATUS_ABORTED
    assert "no path" in payload["message"]
    assert stack.world.nav_goals == [], "precheck must run BEFORE committing the drive"


def test_goto_times_out_and_cancels_the_downstream_goal(stack):
    """Whatever we stop waiting for, we must also stop: an abandoned goal keeps
    driving, which is how a mission reports done while the rover moves (D13)."""
    stack.world.nav_mode = "hold"
    status, payload = stack.send_goto(1.0, 0.0, timeout=25.0)
    assert status == GoalStatus.STATUS_ABORTED
    assert "did not finish" in payload["message"]
    assert stack.world.nav_cancels >= 1, "the downstream goal was abandoned, not cancelled"


# --- observe ----------------------------------------------------------------

def test_observe_passes_through_success_and_message(stack):
    payload = stack.call(stack.observe)
    assert payload["ok"] is True
    assert "folded into the map" in payload["message"]
    assert stack.world.observe_calls == 1


def test_observe_passes_through_failure(stack):
    stack.world.observe_ok = False
    stack.world.observe_message = "camera unavailable"
    payload = stack.call(stack.observe)
    assert payload["ok"] is False and "camera unavailable" in payload["message"]


# --- query_semantic_map -----------------------------------------------------

def test_query_answers_from_the_published_map(stack):
    stack.world.publish_objects()
    time.sleep(0.5)
    payload = stack.call(stack.query)
    assert payload["ok"] is True and payload["count"] == 2


def test_query_filters_by_label_from_the_parameter(stack):
    stack.world.publish_objects()
    time.sleep(0.5)
    stack.task.set_parameters(
        [rclpy.parameter.Parameter("query_label", value="shoe")]
    )
    payload = stack.call(stack.query)
    assert payload["count"] == 1
    assert payload["objects"][0]["label"] == "pink shoe"


def test_query_filters_by_proximity_from_the_parameters(stack):
    stack.world.publish_objects()
    time.sleep(0.5)
    stack.task.set_parameters([
        rclpy.parameter.Parameter("query_near_x", value=0.0),
        rclpy.parameter.Parameter("query_near_y", value=0.0),
        rclpy.parameter.Parameter("query_radius_m", value=2.0),
    ])
    payload = stack.call(stack.query)
    assert [o["label"] for o in payload["objects"]] == ["pink shoe"]


def test_query_radius_beyond_the_envelope_is_refused(stack):
    stack.world.publish_objects()
    time.sleep(0.5)
    stack.task.set_parameters([
        rclpy.parameter.Parameter("query_radius_m", value=999.0),
    ])
    payload = stack.call(stack.query)
    assert payload["ok"] is False and "exceeds" in payload["message"]


def test_query_before_any_observation_answers_nothing_known(stack):
    payload = stack.call(stack.query)
    assert payload["ok"] is True and payload["count"] == 0
    assert payload["have_map"] is False


def test_query_arguments_are_cli_settable_as_plain_scalars(stack):
    """Regression guard for the reason these are typed parameters at all: `ros2
    param set` SEGFAULTS on a value starting with '{', so the obvious single
    JSON-string parameter is not CLI-callable. Every query argument must stay a
    plain scalar a shell can pass unquoted."""
    for name, value in (("query_label", "shoe"), ("query_radius_m", 1.5),
                        ("query_near_x", 0.0), ("query_min_confidence", 0.25)):
        param = stack.task.get_parameter(name)
        assert isinstance(param.value, (str, float)), f"{name} must be a scalar"
        assert not str(param.value).startswith("{")


# --- move_relative: the SELF-CALL, proven rather than reasoned ----------------
#
# `move_relative` computes a map-frame destination and drives it through THIS NODE'S
# OWN `task/goto` action. That is a shape which exists nowhere else in this codebase,
# and its risk is a deadlock: a service handler waiting on an action served by the
# same node. It is safe only because the executor is MultiThreaded with reentrant
# groups. THESE TESTS ARE WHY WE BELIEVE THAT, and they run on the Pi against the real
# executor -- a Mac approximation of executor threading would prove nothing about the
# deadlock that matters.
#
# Every one is bounded. A deadlock must present as a FAILED test, never as a hang:
# that is the corpse-trap lesson applied to a test rather than to a suite.

def _move(stack, distance, heading=0.0, timeout=15.0):
    stack.task.set_parameters([
        rclpy.parameter.Parameter("move_distance_m", value=float(distance)),
        rclpy.parameter.Parameter("move_heading_deg", value=float(heading)),
    ])
    client = stack.client.create_client(Trigger, "task/move_relative")
    return stack.call(client, timeout=timeout)


def test_move_relative_self_call_completes_and_reaches_nav2(stack):
    """CONDITION 1: the self-call returns. If the node deadlocked on its own action
    server this call would never come back, and the bounded `call` turns that into a
    failure with a name instead of a hung suite."""
    payload = _move(stack, 2.0)
    assert payload["ok"] is True, payload
    assert stack.world.nav_goals[-1] == pytest.approx((2.0, 0.0), abs=1e-6), (
        "the transformed destination did not reach nav2 -- the self-call did not "
        "traverse the goto path")


def test_move_relative_uses_the_rovers_heading_not_the_maps(stack):
    """The verb's whole reason to exist: the rover in this harness faces east, so
    'forward' is +x. A heading of +90 must be its LEFT, not map-north by accident."""
    payload = _move(stack, 1.0, heading=90.0)
    assert payload["ok"] is True, payload
    assert stack.world.nav_goals[-1] == pytest.approx((0.0, 1.0), abs=1e-6)


def test_move_relative_refuses_a_move_the_goal_tolerance_would_swallow(stack):
    """Refused in words at the contract, before any ROS call."""
    before = len(stack.world.nav_goals)
    payload = _move(stack, 0.05)
    assert payload["ok"] is False and "minimum" in payload["message"]
    assert len(stack.world.nav_goals) == before, "a refused move still reached nav2"


@pytest.mark.parametrize("stack", [{"max_goal_distance_m": 1.0}], indirect=True)
def test_move_relative_carries_the_inner_refusal_rather_than_flattening_it(stack):
    """CONDITION 3. The envelope belongs to `goto`; this verb must report WHY it
    refused, in goto's own words. A generic 'move failed' would throw away the only
    sentence that says which gate spoke."""
    payload = _move(stack, 2.0)
    assert payload["ok"] is False, payload
    assert "goto_said" in payload, (
        f"the inner reason was swallowed: {payload}")
    assert payload["goto_said"], "goto_said is empty -- the inner reason is lost"


def test_move_relative_inherits_gotos_busy_refusal(stack):
    """CONDITION 2. One drive at a time is `goto`'s rule and this verb must not become
    a second path around it."""
    stack.world.nav_mode = "hold"
    holder = threading.Thread(target=lambda: stack.send_goto(1.0, 0.0, timeout=8.0),
                              daemon=True)
    holder.start()
    time.sleep(1.0)                       # let the first drive take the lock
    payload = _move(stack, 1.0, timeout=12.0)
    assert payload["ok"] is False, "a second drive was accepted while one was running"
    assert "already running" in payload["message"] or "refused" in payload["message"]
    holder.join(timeout=12.0)


def test_move_relative_refuses_a_stale_pose_rather_than_computing_from_history(stack):
    """CONDITION 4, and the class of silent wrongness this week has been about.

    `lookup_transform(Time())` returns the LATEST transform and says nothing about its
    age -- so a rover whose TF stopped an hour ago still answers, confidently, from
    where it WAS. "Forward 2 m" from a stale pose is a destination in the wrong place.
    """
    stack.task.set_parameters(
        [rclpy.parameter.Parameter("move_pose_max_age_s", value=0.3)])
    stack.world.tf_enabled = False
    time.sleep(1.0)                       # older than the bound, by wall clock
    before = len(stack.world.nav_goals)
    payload = _move(stack, 1.0)
    assert payload["ok"] is False, "a stale pose was used to compute a destination"
    assert "stale" in payload["message"] or "old" in payload["message"], payload
    assert len(stack.world.nav_goals) == before, "the stale move still reached nav2"
    stack.world.tf_enabled = True
