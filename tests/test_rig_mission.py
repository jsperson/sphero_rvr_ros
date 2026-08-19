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
        "f1": 300.0, "f2a": 600.0, "f2": 600.0, "cert": 1800.0}
    assert _assigned_literal("B1_ARM_S") == 30.0
    assert _assigned_literal("B3_PATH_M") == 15.0
    assert _assigned_literal("B3_GOALS") == 3


def test_the_per_arm_predictions_are_the_ratified_table():
    """The table as AMENDED by the PM ruling of 2026-08-18 (F2a/F2b): f1 fails
    everything but the quiet-marks bar; f2a runs against the UN-fixed product
    and predicts B4 FAIL -- the repaired bar must be seen to catch the D38
    silence before it is trusted; f2 (F2b) keeps the original ratified
    predictions against the FIXED product -- the explorer itself names the
    failure; cert predicts a clean sweep."""
    assert _assigned_literal("PREDICTED") == {
        "f1":   {"B1": False, "B2": False, "B3": False, "B4": False, "B5": True},
        "f2a":  {"B1": True,  "B2": False, "B3": False, "B4": False, "B5": True},
        "f2":   {"B1": True,  "B2": False, "B3": False, "B4": True,  "B5": True},
        "cert": {"B1": True,  "B2": True,  "B3": True,  "B4": True,  "B5": True},
    }


def test_b4_excludes_the_runners_own_stop_report():
    """Ratified fix (a): F2's first run scored B4 on the STOPPED_BY_OPERATOR
    report the runner's own cleanup caused -- fabricated input. A report that
    arrives at or after the runner's stop call is not the explorer speaking."""
    assert "stop_called_at" in SRC
    assert "watch.report_t is None or watch.report_t >= stop_called_at" in SRC
    assert "excluding self-caused report" in SRC


def test_b2_means_completed_not_merely_ended():
    """Sharpened with the D38 batch (flagged to the PM in daylight): the fix
    makes honest INCOMPLETE ends set done=true too, so the literal 'done within
    budget' would score F2b's honest end as a pass and contradict the ruling's
    own F2b prediction. The intent was always 'the mission COMPLETES'."""
    assert 'b2 = done_seen and bool((report or {}).get("complete"))' in SRC


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
