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


def test_the_deployed_config_disables_the_low_obstacle_brake():
    """THE DEPLOYED YAML, not the dataclass default. Thirteen fields once differed
    between the two and a verdict flipped depending on which was read."""
    text = SUPERVISOR_CFG.read_text()
    m = re.search(r"^\s*low_obstacle_brake_enable:\s*(\w+)", text, re.M)
    assert m, "low_obstacle_brake_enable is not set in the deployed config at all"
    assert m.group(1) == "false", (
        f"the deployed supervisor config has low_obstacle_brake_enable: {m.group(1)} -- "
        "the camera still brakes and still vetoes pivots on an unmeasured mount")


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
    """Authority removed, sensor kept. Track 2 uses it, the recording is still evidence,
    and its cloud is what will timestamp when the mount moved. Disabling a sensor and
    disabling its authority are different acts and only one of them was ordered."""
    assert (ROOT / "launch" / "camera.launch.py").exists(), (
        "the camera launch was removed; Scott's direction was that its FEEDBACK must "
        "not guide the rover, not that the camera comes off the robot")
    assert "low_obstacle_topic: /camera/low_obstacles" in SUPERVISOR_CFG.read_text(), (
        "the topic was repointed or blanked in the same change that disabled the layer; "
        "keep them separate so 'camera removed' and 'ToF given authority' stay two "
        "reviewable steps")


def test_what_still_protects_the_rover_is_written_down():
    """A batch that removes a protection has to say what remains, in the file a future
    reader opens -- not only in a commit message they will not find."""
    cfg = SUPERVISOR_CFG.read_text()
    block = cfg[cfg.index("low_obstacle_brake_enable"):]
    block = cfg[max(0, cfg.index("low_obstacle_brake_enable") - 2000):
                cfg.index("low_obstacle_brake_enable")]
    for phrase in ("NO SUB-LIDAR PROTECTION", "lidar core", "escape ladder"):
        assert phrase.lower() in block.lower(), (
            f"the config does not say {phrase!r} where the layer is disabled; a reader "
            "finding brake_enable=false has no way to learn what still protects the robot")
