"""The mission runner's bars are RATIFIED constants, not tunables.

The PM round of 2026-08-18 ratified B1-B5 with exact numbers and per-arm
predictions BEFORE scripts/rig_mission.py existed. These tests pin the table to
that ratification: an edit that nudges a bar to make an arm pass goes red here,
which is the point — bar changes return to the consensus round in daylight.

Source-level (AST), because the runner imports rclpy and the Mac has none.
"""

import ast
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "rig_mission.py"
SRC = SCRIPT.read_text()
TREE = ast.parse(SRC)


def _assigned_literal(name):
    for node in ast.walk(TREE):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if getattr(target, "id", None) == name:
                    return ast.literal_eval(node.value)
    raise AssertionError(f"{name} is not assigned a literal in rig_mission.py")


def test_the_ratified_numbers_are_the_ratified_numbers():
    assert _assigned_literal("ARM_WINDOW_S") == {
        "f1": 300.0, "f2": 600.0, "cert": 1800.0}
    assert _assigned_literal("B1_ARM_S") == 30.0
    assert _assigned_literal("B3_PATH_M") == 15.0
    assert _assigned_literal("B3_GOALS") == 3


def test_the_per_arm_predictions_are_the_ratified_table():
    """f1 fails everything but the quiet-marks bar; f2 must END HONESTLY (B4
    passes while B2 fails); cert predicts a clean sweep."""
    assert _assigned_literal("PREDICTED") == {
        "f1":   {"B1": False, "B2": False, "B3": False, "B4": False, "B5": True},
        "f2":   {"B1": True,  "B2": False, "B3": False, "B4": True,  "B5": True},
        "cert": {"B1": True,  "B2": True,  "B3": True,  "B4": True,  "B5": True},
    }


def test_the_exit_trichotomy_is_all_three_values():
    """0 package-pass / 1 package-fail / 2 rig-inconclusive — INCONCLUSIVE must
    stay distinct: a rig that cannot ask the question must never report an
    answer to it."""
    assert "return finish(2)" in SRC, "the inconclusive exit lost its own code"
    assert "finish(0 if package_pass else 1)" in SRC
    assert "neither a pass nor a failure" in SRC


def test_the_watch_window_cannot_hang():
    """f2's pre-registration says an arm that hangs is a package FAIL — which is
    only observable if the runner itself always returns. The watch loop must be
    bounded by the window, never by the mission."""
    assert "while time.monotonic() - t_start < window_s:" in SRC, (
        "the watch loop is no longer bounded by the ratified window")


def test_b5_counts_nonempty_marks_only_and_all_promotions():
    """Rider 1 as implemented: contact_marker's empty heartbeat clouds are its
    designed idle state; a cloud WITH points in a rig with no contact physics is
    a phantom marker firing. Promotions count literally."""
    assert "if msg.width > 0:" in SRC
    assert "nonempty_marks == 0 and watch.promotes == 0" in SRC
