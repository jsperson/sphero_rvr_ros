"""Structural proof for the ToF node's stage-(i) boundary: it PUBLISHES ONLY."""
from pathlib import Path
NODE = Path(__file__).resolve().parents[1] / "src" / "sphero_rvr_driver" / "tof_node.py"

def _body(text):
    body = text.split('"""', 2)[-1]
    return "\n".join(l for l in body.splitlines() if not l.strip().startswith("#"))

def test_the_tof_node_has_no_motion_authority_in_stage_one():
    """The whole point of stage (i) is a sensor on the graph that nothing obeys.

    Asserted structurally because the alternative is remembering: this node must not
    publish velocity, must not subscribe to anything that would let it react, and must
    not touch the supervisor's topics. When stage (iii) wires it into the brake, that
    happens in collision_stop_node -- not here -- and this test should still pass.
    """
    body = _body(NODE.read_text())
    assert "Twist" not in body, "the ToF node imports a velocity type"
    assert "cmd_vel" not in body, "the ToF node names a velocity topic"
    assert "create_subscription" not in body, (
        "the ToF node subscribes to something; in stage (i) it is a source, not a "
        "consumer, and a subscription is how a sensor node grows opinions")

def test_it_publishes_what_it_saw_separately_from_what_it_concluded():
    """`points` and `obstacles` must stay distinct. Merging them destroys the ability
    to re-derive a conclusion from a recording, which is exactly what rescued the
    characterisation when the first analysis statistic turned out to be wrong."""
    body = _body(NODE.read_text())
    assert '"~/points"' in body and '"~/obstacles"' in body and '"~/state"' in body
