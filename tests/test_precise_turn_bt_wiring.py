"""Batch (a) wiring pins: the Spin retarget is INERT until its arg flips.

Source-level, like the launch pins in test_launch_and_arm: the retargeted tree
must point at a server that exists, keep the standard tree's recovery shape, and
be unreachable from every flight command until use_precise_turn_spin flips --
that flip is the post-bench lock-flip batch's one launch-arg diff, and these
pins are what make it exactly one diff.
"""

from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
BT = (REPO / "behavior_trees" / "navigate_to_pose_stock_precise_turn.xml").read_text()
LAUNCH = " ".join((REPO / "launch" / "explore.launch.py").read_text().split())
SUPERVISOR = (REPO / "src" / "sphero_rvr_driver" / "collision_stop_node.py").read_text()
SETUP = (REPO / "setup.py").read_text()

GATEWAY = "/collision_stop/precise_turn"


def test_the_spin_child_targets_the_gateway_and_the_gateway_exists():
    """Two-sided pin (the QoS-pin pattern): the XML names the server AND the
    supervisor serves that exact name. The 2026-08-19 bench card already caught
    one server-name drift (/precise_turn vs /collision_stop/precise_turn);
    neither side may move alone."""
    assert f'<Spin spin_dist="1.57" server_name="{GATEWAY}"' in BT
    assert f'Spin, "{GATEWAY}"' in SUPERVISOR, (
        "the supervisor no longer serves the gateway at the name the BT calls")


def test_the_retargeted_tree_keeps_the_standard_recovery_shape():
    """One attribute changed, nothing else: retries, the RoundRobin order
    (clearing -> Spin -> Wait -> BackUp), and the untouched recovery children are
    the standard tree's own. A second divergence is a design change wearing a
    retarget's name."""
    import re
    tree_only = re.sub(r"<!--.*?-->", "", BT, flags=re.DOTALL)
    assert tree_only.count("server_name=") == 1, (
        "more than one node grew a server_name -- the retarget is no longer "
        "one attribute")
    assert '<RecoveryNode number_of_retries="6" name="NavigateRecovery">' in BT
    order = [BT.index(s) for s in (
        'name="ClearingActions"', "<Spin ", "<Wait wait_duration=\"5.0\"",
        '<BackUp backup_dist="0.30" backup_speed="0.15"')]
    assert order == sorted(order), "the recovery RoundRobin order changed"
    assert 'spin_dist="1.57"' in BT, "the spin distance drifted from standard"


def test_the_launch_arg_defaults_true_and_selects_the_tree():
    """The lock is FLIPPED: default true since the 2026-08-19 (viii)-(ix)
    sitting, exactly as this pin's earlier form demanded ('must arrive with the
    bench card's receipts' -- it did: (viii) PASS with 'precise turn SETTLED:
    target 150.4 deg, heading 151.7 deg, err -1.3 deg, 1.0 s' on the stall
    scrub, (ix) satisfied by live event, PM-reviewed; see the card's RESULTS).
    A revert to false is the watch-item rollback and must cite its wrong turn,
    or it is a silent revert. The selection expression reaches both surviving
    trees -- the retarget on the arg, standard as fallthrough. (The decisive
    tree and its first-priority slot died with the bespoke controller,
    2026-08-21 project review.)"""
    assert '"use_precise_turn_spin", default_value="true"' in LAUNCH, (
        "the routing arg's default reverted -- if that was the watch-item "
        "rollback (one wrong turn), flip this pin with it and cite the turn; "
        "if not, it is a silent revert")
    assert "navigate_to_pose_stock_precise_turn.xml" in LAUNCH
    # Precedence is read from the SELECTION EXPRESSION, not the file head (the
    # Path definitions up top appear in a different order).
    expr = LAUNCH[LAUNCH.index("nav_to_pose_bt_xml = ParameterValue"):]
    expr = expr[:expr.index("value_type=str")]
    order = [expr.index(s) for s in (
        "str(precise_turn_nav_to_pose_bt)",
        "use_precise_turn_spin",
        "str(standard_nav_to_pose_bt)")]
    assert order == sorted(order), (
        "the BT selection precedence changed: the retarget must gate on its "
        "arg, standard must be the fallthrough")


def test_the_flight_commands_do_not_touch_the_arg():
    """launch_and_arm's exact commands stay byte-identical: no stack passes
    use_precise_turn_spin, so every flight rides the launch default (false).
    When the lock-flip batch lands, it flips the DEFAULT -- not the commands."""
    script = (REPO / "scripts" / "launch_and_arm.py").read_text()
    assert "use_precise_turn_spin" not in script


def test_the_tree_is_installed():
    assert "behavior_trees/navigate_to_pose_stock_precise_turn.xml" in SETUP, (
        "the retargeted tree is not in setup.py data_files -- real, reviewed, "
        "and unreachable by get_package_share_directory (the D-era install "
        "manifest lesson)")
