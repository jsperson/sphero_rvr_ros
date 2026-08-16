"""D50: a mission that ends must leave a report ON DISK, whichever way it ended.

2026-08-16 mission 2 ran, ended, published its report to a latched topic, logged it -- and
left no report file. The outcome numbers survived only because the status topic happened
to be sampled by hand before teardown. **A mission whose evidence depends on somebody
remembering to catch a topic has optional evidence.**

Two distinct gaps, both closed here:

  1. `_finish` published and logged but never wrote a file.
  2. `mission/stop` never called `_finish` at all -- an operator-ended mission produced no
     report by ANY route. It disarmed, cancelled the goal, and returned success.

These are source-level guards so they run everywhere, including the Mac where rclpy is
absent. The behavioural harness (`tests/test_coverage_explorer_mission.py`) runs the real
node and is the right place for outcomes; this file guards the SHAPE that made the gap
possible.
"""

import ast
from pathlib import Path

import pytest

from sphero_rvr_core import mission_report

NODE = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "sphero_rvr_driver"
    / "coverage_explorer_node.py"
)
SOURCE = NODE.read_text()
TREE = ast.parse(SOURCE)


def _function(name: str) -> ast.FunctionDef:
    for node in ast.walk(TREE):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{name}() is missing from coverage_explorer_node.py")


def _calls_in(node) -> set:
    return {
        getattr(call.func, "attr", None) or getattr(call.func, "id", None)
        for call in ast.walk(node)
        if isinstance(call, ast.Call)
    }


def test_the_operator_stop_outcome_exists_and_is_a_recognised_outcome():
    assert mission_report.OUTCOME_STOPPED_BY_OPERATOR == "STOPPED_BY_OPERATOR"
    assert mission_report.OUTCOME_STOPPED_BY_OPERATOR in mission_report.ALL_OUTCOMES


def test_finish_writes_the_report_to_disk_not_only_to_a_topic():
    calls = _calls_in(_function("_finish"))
    assert "_write_report_file" in calls, (
        "_finish publishes and logs; without a file write the mission's evidence lives "
        "only in a latched topic and a log line"
    )


def test_mission_stop_ends_the_mission_through_the_same_funnel():
    calls = _calls_in(_function("_on_mission_stop"))
    assert "_finish" in calls, (
        "mission/stop disarms and cancels but must also FINISH, or an operator-ended "
        "mission leaves no report by any route -- exactly what happened on 2026-08-16"
    )


#: Outcomes that are deliberately NOT terminal, with the reason. Being wedged does not end
#: a mission -- freeing the rover resumes it -- so START_BLOCKED publishes a status report
#: without consuming the terminal latch. Adding to this list is a deliberate act a reviewer
#: sees; the first version of the guard below had no list and failed on this legitimate
#: case, which is the right way round for a guard to be wrong.
#:
#: Worth knowing when reading old runs: gauntlet mission 1 is recorded as ending
#: `INCOMPLETE_START_BLOCKED`, but that was a NON-terminal status report that happened to
#: be the last one latched when the run was stopped. With mission/stop now routed through
#: _finish, a run halted while wedged produces a real terminal report instead of leaving a
#: wedge notice as the de facto final word.
NON_TERMINAL_OUTCOMES = {"OUTCOME_START_BLOCKED"}


def test_every_mission_ending_outcome_is_routed_through_finish():
    # If a new terminal outcome is ever introduced, it must reach _finish.
    finish_args = set()
    for node in ast.walk(TREE):
        if not isinstance(node, ast.Call):
            continue
        if (getattr(node.func, "attr", None) or getattr(node.func, "id", None)) != "_finish":
            continue
        if node.args and isinstance(node.args[0], ast.Name):
            finish_args.add(node.args[0].id)

    used_outcomes = {
        n.id
        for n in ast.walk(TREE)
        if isinstance(n, ast.Name) and n.id.startswith("OUTCOME_")
    }
    unrouted = used_outcomes - finish_args - NON_TERMINAL_OUTCOMES
    assert not unrouted, (
        f"{sorted(unrouted)} appear in the node but are never passed to _finish -- a "
        "terminal outcome that does not go through the funnel produces no artifact. "
        "If one of these is deliberately non-terminal, add it to NON_TERMINAL_OUTCOMES "
        "with the reason, so the exemption is visible rather than implicit."
    )
    # And the exemption list must not rot: an outcome listed here must still be used.
    assert NON_TERMINAL_OUTCOMES <= used_outcomes, (
        f"{sorted(NON_TERMINAL_OUTCOMES - used_outcomes)} is exempted but no longer "
        "appears in the node"
    )


def test_the_report_writer_fails_soft():
    """A report that cannot be written must not also destroy the mission it describes --
    the same rule `_save_map` already follows."""
    writer = _function("_write_report_file")
    handlers = [n for n in ast.walk(writer) if isinstance(n, ast.ExceptHandler)]
    assert handlers, "_write_report_file must catch its own failures"
    for handler in handlers:
        raises = [n for n in ast.walk(handler) if isinstance(n, ast.Raise)]
        assert not raises, "a failed report write must be logged, never raised"


def test_the_report_directory_is_a_declared_parameter():
    assert '"mission_report_dir"' in SOURCE, (
        "the report location must be configurable, not hard-coded"
    )
    # And expanded, or a literal ~ from YAML creates a directory named '~' in the cwd.
    writer_source = ast.get_source_segment(SOURCE, _function("_write_report_file"))
    assert "expanduser" in writer_source


@pytest.mark.parametrize(
    "outcome",
    sorted(mission_report.ALL_OUTCOMES),
)
def test_build_report_accepts_every_declared_outcome(outcome):
    report = mission_report.build_report(
        outcome,
        covered_cells=1,
        resolution=0.05,
        duration_s=1.0,
        goals_sent=1,
        goals_succeeded=0,
        goals_aborted=1,
        goals_aborted_after_recovery=1,
        goals_aborted_without_recovery=0,
        planner_rejections=0,
        remaining_candidates=0,
        map_files=[],
    )
    assert report["outcome"] == outcome
