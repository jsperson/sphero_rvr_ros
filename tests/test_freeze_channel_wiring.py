"""The freeze channels must actually connect. Structural, not behavioural.

Both freeze publishers were DARK at HEAD: the controller published bare relative
names, which resolve against the NAMESPACE rather than the node name, so marks went
to /freeze_marks while lean_nav2.yaml's freeze_layer read
/decisive_controller/freeze_marks. Nothing errored -- publishing to a topic with no
subscriber is legal and silent -- so freeze-as-sensor was inert in the field while
looking healthy in every unit test, because the rehearsal harness published on the
absolute names itself.

So this asserts the PAIRING rather than the string: the topic the controller resolves
must equal the topic read out of the deployed costmap config. If either side is
renamed, this fails, which is the only way the two can be kept from drifting apart
silently.

Deliberately one test per file: `importorskip` skips the whole MODULE, and this
project has already lost a safety test to being parked behind a skip alongside
unrelated cases.
"""

import os

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS 2 not available on this host")
yaml = pytest.importorskip("yaml")

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _freeze_layer_topic():
    """The topic the deployed costmap's freeze_layer actually subscribes to."""
    with open(os.path.join(REPO, "config", "lean_nav2.yaml")) as handle:
        raw = yaml.safe_load(handle)

    def walk(node):
        if isinstance(node, dict):
            if "freeze" in node and isinstance(node["freeze"], dict):
                topic = node["freeze"].get("topic")
                if topic:
                    return topic
            for value in node.values():
                found = walk(value)
                if found:
                    return found
        return None

    topic = walk(raw)
    assert topic, "freeze_layer observation source vanished from lean_nav2.yaml"
    return topic


def test_freeze_publishers_resolve_to_the_topics_their_consumers_read():
    from sphero_rvr_driver.decisive_controller_node import DecisiveControllerNode

    rclpy.init(args=["--ros-args"])
    try:
        node = DecisiveControllerNode()
        try:
            resolved = {p[0] for p in node.get_publisher_names_and_types_by_node(
                node.get_name(), node.get_namespace())}
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()

    costmap_topic = _freeze_layer_topic()
    assert costmap_topic in resolved, (
        f"freeze MARKS are dark: costmap freeze_layer reads {costmap_topic!r} but the "
        f"controller publishes {sorted(t for t in resolved if 'freeze' in t)!r}. "
        "Marks never reach the planner and nothing errors.")

    # The explorer's side of the same trap: it subscribes to this absolute name.
    event_topic = "/decisive_controller/freeze_event"
    assert event_topic in resolved, (
        f"freeze EVENTS are dark: coverage_explorer subscribes {event_topic!r} but "
        f"the controller publishes {sorted(t for t in resolved if 'freeze' in t)!r}. "
        "The mission layer can never classify an abort as discovery.")
