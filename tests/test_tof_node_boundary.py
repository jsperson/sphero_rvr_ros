"""Structural proof for the ToF node's stage-(i) boundary: it PUBLISHES ONLY."""
from pathlib import Path
NODE = Path(__file__).resolve().parents[1] / "src" / "sphero_rvr_driver" / "tof_node.py"

def _body(text):
    body = text.split('"""', 2)[-1]
    return "\n".join(l for l in body.splitlines() if not l.strip().startswith("#"))

def test_the_tof_node_has_no_motion_authority_in_stage_one():
    """The whole point of stage (i) is a sensor on the graph that nothing obeys.

    Asserted structurally because the alternative is remembering: this node must not
    publish velocity and must not touch the supervisor's topics. When stage (iii) wires
    it into the brake, that happens in collision_stop_node -- not here -- and this test
    should still pass.

    NARROWED BY THE 9.x AMENDMENT, and narrowed rather than deleted. This used to assert
    ZERO subscriptions. Rule B needs the lidar as a live background (design 9.3), so
    exactly ONE subscription is now allowed and it is named here. Widening a boundary
    test to accommodate the change that broke it is how a boundary stops being one, so
    the allowance is a WHITELIST OF ONE rather than a relaxed rule -- a second
    subscription, to anything, fails this again.
    """
    body = _body(NODE.read_text())
    assert "Twist" not in body, "the ToF node imports a velocity type"
    assert "cmd_vel" not in body, "the ToF node names a velocity topic"

    subs = body.count("create_subscription")
    assert subs <= 1, (
        f"the ToF node now has {subs} subscriptions; rule B's /scan background is the "
        "only one this stage allows, and a sensor node that consumes a second topic is "
        "a sensor node growing opinions")
    if subs:
        assert "LaserScan" in body, (
            "the single permitted subscription is not the /scan background -- whatever "
            "it is, it was not in the reviewed design")
    # Rule B is an OBSERVER of the scan, never a participant in control. These are the
    # topics that would make it one.
    for forbidden in ("goal", "/collision_stop/", "behavior_tree", "follow_path"):
        assert forbidden not in body, (
            f"the ToF node references {forbidden!r}; it is a sensor, and stage (iii) is "
            "where the supervisor consumes it, not where it consumes the supervisor")

def test_it_publishes_what_it_saw_separately_from_what_it_concluded():
    """`points` and `obstacles` must stay distinct. Merging them destroys the ability
    to re-derive a conclusion from a recording, which is exactly what rescued the
    characterisation when the first analysis statistic turned out to be wrong."""
    body = _body(NODE.read_text())
    assert '"~/points"' in body and '"~/obstacles"' in body and '"~/state"' in body


def test_rule_b_is_advertised_as_UNPINNED_on_the_state_topic():
    """The margin rule B compares against has NO recorded data behind it -- the tilt
    session captured 23,057 ToF frames and zero synchronised /scan (design 9.8).

    A number with no evidence that looks exactly like a number with evidence is how a
    provisional threshold becomes a fact nobody re-examines. So the node says so on
    every state line, and this test keeps it saying so until bench item J removes the
    reason. Delete the word and this fails, which is the point.
    """
    body = _body(NODE.read_text())
    assert "rule_b=UNPINNED" in body, (
        "the state topic no longer advertises rule B as unpinned; if bench item J has "
        "been run, cite the capture here and in design 9.8 rather than just dropping it")


def test_the_lidar_plane_height_comes_from_TF_not_a_constant():
    """N1, pre-empted a third time. Rule B's applicability test needs the height of the
    lidar's scan plane, and a node that hardcodes it is a node that keeps working after
    somebody moves the lidar -- while being wrong."""
    body = _body(NODE.read_text())
    assert "lookup_transform" in body, "the ToF node never asks TF for anything"
    assert "_lidar_plane_m = float(tf.transform.translation.z)" in body, (
        "the lidar plane height is not being read from the TF lookup; if it is coming "
        "from a parameter or a literal, moving the lidar silently breaks rule B")
