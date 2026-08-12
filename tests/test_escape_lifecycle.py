"""The give-up escape's LIFECYCLE guards, on the real node.

docs/reverse_before_give_up_design.md §6(b). D34 cost this project 26 s of pushing a
weight bench because a lifecycle trigger fired mid-escape and nobody had proved what
happened when it did. That lesson applies to whatever lifecycle a batch adds, so the
two guards this one adds are asserted against the real `DecisiveControllerNode` rather
than against a description of it.

Needs rclpy, so it SKIPS on the workstation and runs on the Pi. The parts of this
batch that can be proved without ROS deliberately live elsewhere and run everywhere
(tests/test_escape_geometry.py, tests/test_escape_outcome.py,
tests/test_task_node_safety.py).
"""

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS 2 not available on this host")

from rclpy.action import GoalResponse  # noqa: E402


@pytest.fixture()
def node():
    from sphero_rvr_driver.decisive_controller_node import DecisiveControllerNode
    rclpy.init(args=["--ros-args"])
    n = DecisiveControllerNode()
    try:
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


def test_no_goal_starts_while_an_escape_is_in_flight(node):
    """REVERT-PROOF 5. A `follow_path` arriving mid-escape must be REJECTED.

    Accepting it would put two authors on cmd_vel and silently destroy the escape --
    and the escape exists precisely because planning had already failed, so the first
    thing that plans again would kill the thing that made planning possible. That is
    D34's mechanism with the roles swapped.
    """
    assert node._follow_path_goal_callback(None) == GoalResponse.ACCEPT
    node._escape_active = True
    assert node._follow_path_goal_callback(None) == GoalResponse.REJECT, (
        "a goal was accepted while a give-up escape was driving")
    node._escape_active = False
    assert node._follow_path_goal_callback(None) == GoalResponse.ACCEPT, (
        "the rejection latched — the controller would never take another goal")


def test_an_escape_is_declined_unless_the_controller_is_idle(node):
    """REVERT-PROOF 8 (controller half). The explorer only asks while it believes
    nothing is running, so accepting an escape mid-goal would mean two motions
    authored at once; the decline is what makes the disagreement VISIBLE instead of
    letting it become a collision."""
    assert node._escape_goal_callback(None) == GoalResponse.ACCEPT

    node._active_goal_handle = object()           # a goal is running
    assert node._escape_goal_callback(None) == GoalResponse.REJECT
    node._active_goal_handle = None

    node._escape_active = True                    # an escape is already running
    assert node._escape_goal_callback(None) == GoalResponse.REJECT
    node._escape_active = False

    assert node._escape_goal_callback(None) == GoalResponse.ACCEPT


def test_the_escape_server_exists_under_the_name_the_explorer_calls(node):
    """The seam's other half: a perfectly good server on the wrong name is the same
    as no server, and the explorer's fallback for "no server" is to give up."""
    names = [n for n, _ in node.get_service_names_and_types()]
    assert any("/decisive_controller/escape_in_place" in n for n in names), (
        "no escape action server on the name coverage_explorer_node calls; the "
        f"explorer would log 'escape UNAVAILABLE' and end the mission. Saw: {names}")
