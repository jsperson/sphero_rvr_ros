"""The camera holds NO motion authority. Scott's direction, 2026-08-13.

Verbatim: "I thought we removed the camera as a guide. It isn't mounted properly so any
feedback would be terrible."

A monocular floor-projection reports WHERE the floor stops, and every distance it
derives depends on the mount's aim. The mount moved -- most likely during the ToF mount
work -- so the layer that was steering, braking and vetoing pivots was doing so on wrong
numbers for every mission since. `docs/tof_navigation_design.md` 10.2.

WHAT THESE PROOFS ARE ANCHORED ON: disabled must be INDISTINGUISHABLE from stale. Those
are the same fail-open path, already covered by revert-proofs 1-2, which is exactly why
`enable: false` was chosen over deleting the code -- it exercises a tested route rather
than creating a new one. If the two ever diverge, the disable is doing something the
stale path does not, and that something has never been reviewed.
"""
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SUPERVISOR_CFG = ROOT / "config" / "collision_stop.yaml"
SUPERVISOR = ROOT / "src" / "sphero_rvr_driver" / "collision_stop_node.py"
CONTROLLER = ROOT / "src" / "sphero_rvr_driver" / "decisive_controller_node.py"


def test_the_camera_CLOUD_does_not_feed_the_brake_or_the_veto():
    """THE PROPERTY, NOT THE MECHANISM -- and this test had to be rewritten to say so.

    Its first version asserted `low_obstacle_brake_enable: false`, which was how the
    camera lost authority on 2026-08-13. Hours later the ToF took the slot and the layer
    was re-enabled, so the test failed while the property it exists to protect was
    perfectly intact. A test pinned to the mechanism of the day fails on the next
    correct change and passes on the next wrong one.

    What must stay true regardless of mechanism: the supervisor does not consume
    /camera/low_obstacles. Whether that is achieved by disabling the layer or by
    pointing it elsewhere is an implementation detail.

    THE DEPLOYED YAML, not the dataclass default -- thirteen fields once differed
    between those two and a verdict flipped depending on which was read.
    """
    text = SUPERVISOR_CFG.read_text()
    topic = re.search(r"^\s*low_obstacle_topic:\s*(\S+)", text, re.M)
    enable = re.search(r"^\s*low_obstacle_brake_enable:\s*(\w+)", text, re.M)
    assert topic and enable, "the low-obstacle layer's config is missing"
    feeds_camera = (topic.group(1) == "/camera/low_obstacles"
                    and enable.group(1) == "true")
    assert not feeds_camera, (
        "the supervisor is consuming /camera/low_obstacles again. The mount is "
        "unmeasured -- Scott: 'any feedback would be terrible' -- so its cloud must not "
        "reach the brake or the pivot veto by any route")


def test_disabling_the_brake_also_disables_the_PIVOT_VETO():
    """One flag, both roles. The veto is a separate code path from the brake and reads
    the same cloud; a flag that stopped the brake and left the veto running would leave
    the camera silently steering turns."""
    src = SUPERVISOR.read_text()
    veto = src[src.index("def _low_obstacle_blocks_pivot"):]
    veto = veto[:veto.index("def _apply_low_obstacle_brake")]
    assert "self._lowobs_enable" in veto, (
        "the pivot veto does not consult the enable flag, so disabling the brake leaves "
        "the camera vetoing pivots")
    assert re.search(r"if not self\._lowobs_enable:\s*\n\s*return False", veto), (
        "the veto reads the enable flag but does not return a NON-BLOCKING answer when "
        "it is off -- a disabled sensor must not be able to block anything")


def test_the_steering_law_input_is_disabled_in_code_by_a_named_switch():
    """Not by pointing at a topic nobody publishes. That disables it just as well and
    reads to the next person as a configuration mistake rather than a decision."""
    src = CONTROLLER.read_text()
    assert 'declare_parameter("avoid_camera_enable", False)' in src, (
        "the steering law has no explicit enable switch defaulting to off")
    assert "if not self._avoid_camera_enable:\n                return" in src, (
        "the switch exists but nothing gates the blocker on it; a cloud that arrives "
        "anyway would still steer the rover")


def test_the_camera_NODE_still_runs():
    """Authority removed, sensor kept — and then the monocular detector retired.
    The camera itself stays (recognition, semantic map, the recordings are
    evidence). The low-obstacle NODE was deleted 2026-08-21 with bucket zero of
    the project review (the 2026-08-10 rangefinder decision made ToF the
    low-obstacle sense); this guard now pins BOTH facts: camera present,
    monocular detector gone and staying gone without a new ratification."""
    assert (ROOT / "launch" / "camera.launch.py").exists(), (
        "the camera launch was removed; Scott's direction was that its FEEDBACK must "
        "not guide the rover, not that the camera comes off the robot"
    )
    assert not (ROOT / "src" / "sphero_rvr_driver" / "low_obstacle_node.py").exists(), (
        "the monocular low-obstacle node is back — it was RETIRED by the 2026-08-21 "
        "project review on the ratified rangefinder decision; returning it needs a "
        "ratification, not a revert")

def test_the_configs_limits_are_written_down_beside_it():
    """A layer has to say what it is doing in the file a future reader opens, not only
    in a commit message they will never find. The demanded phrases track the STATE and
    have now changed twice: disabled, it had to name what still protected the rover;
    gated, it had to name what was unmeasured; LIVE, it must name the evidence that
    unlocked it and what that evidence does not cover.

    The rule is constant and the words are not, which is the point -- a fixed phrase
    list would have gone on demanding "RULE B IS GATED OFF" from a config where rule B
    is live, i.e. it would have enforced a lie."""
    cfg = SUPERVISOR_CFG.read_text()
    i = cfg.index("low_obstacle_brake_enable")
    block = cfg[max(0, i - 2500):i + 2500]
    for phrase in ("RULE B IS LIVE", "bench item J", "RULE B PINNED BY"):
        assert phrase.lower() in block.lower(), (
            f"the config does not say {phrase!r} beside the layer it governs. A reader "
            "finding this enabled must be able to learn, without leaving the file, which "
            "rules are actually live and what is still unmeasured")
