"""D38's fix: a mission that never finds a target must END, honestly, by name.

THE MUST-FLIP WITNESS. Before 2026-08-18 the completion latch read
`self._ever_had_target and self._consecutive_empty >= N` -- which makes a mission
that never had a single target UN-ENDABLE BY CONSTRUCTION. The F2 falsifier arm
reproduced it in the sim rig: 687 status messages of armed=true goals_sent=0
across a 600 s window, done never, report never (the only report in the bag was
the one the test runner's own cleanup stop caused). On that code,
test_a_mission_that_never_had_a_target_ends fails -- first because the pure
helper does not exist, and behaviorally because the decision it encodes was
"None forever". That is the flip this file certifies.

The decision is PURE (sphero_rvr_core.mission_report.empty_cycle_outcome), one
debounce for every ending, no new tunable; the node consumes it. An AST guard
below holds the node to the helper so the fix cannot silently un-happen.
"""

import ast
from pathlib import Path

import pytest

from sphero_rvr_core.mission_report import (
    ALL_OUTCOMES,
    OUTCOME_COMPLETE,
    OUTCOME_NO_PLANNABLE_TARGETS,
    OUTCOME_NO_TARGETS_FROM_START,
    empty_cycle_outcome,
)

NODE = (Path(__file__).resolve().parents[1] / "src" / "sphero_rvr_driver"
        / "coverage_explorer_node.py")
NODE_SRC = NODE.read_text()

DEBOUNCE = 20  # any value works; the tests pass the same number both sides


def test_a_mission_that_never_had_a_target_ends():
    """THE D38 FLIP: never-had-target + nothing wanted + debounce satisfied must
    produce a terminal outcome, not None. On the old code this decision was
    'None forever' and the rover sat armed and silent."""
    outcome = empty_cycle_outcome(
        ever_had_target=False, consecutive_empty=DEBOUNCE,
        complete_after_empty=DEBOUNCE, remaining_candidates=0)
    assert outcome == OUTCOME_NO_TARGETS_FROM_START


def test_the_ending_has_its_own_name_and_the_report_accepts_it():
    """Never an overload of an existing outcome (D51's SUCCEEDED lesson):
    START_BLOCKED means own-cell-at-inscribed, which measurably does NOT cover
    an enclosure whose walls sit past the inscribed radius."""
    assert OUTCOME_NO_TARGETS_FROM_START == "INCOMPLETE_NO_TARGETS_FROM_START"
    assert OUTCOME_NO_TARGETS_FROM_START in ALL_OUTCOMES
    assert OUTCOME_NO_TARGETS_FROM_START != "INCOMPLETE_START_BLOCKED"


@pytest.mark.parametrize("empty_count", range(0, DEBOUNCE))
def test_the_debounce_still_debounces_every_ending(empty_count):
    """Inside the debounce nothing ends, whatever the history -- the same
    N-consecutive-empties rule the node always had, no new tunable."""
    for ever, remaining in ((True, 0), (True, 5), (False, 0), (False, 5)):
        assert empty_cycle_outcome(ever, empty_count, DEBOUNCE, remaining) is None


def test_the_pre_existing_endings_are_byte_for_byte_semantics():
    """The fix adds a row to the lattice; the two old rows must not move."""
    assert empty_cycle_outcome(True, DEBOUNCE, DEBOUNCE, 0) == OUTCOME_COMPLETE
    assert empty_cycle_outcome(True, DEBOUNCE, DEBOUNCE, 7) == (
        OUTCOME_NO_PLANNABLE_TARGETS)


def test_never_had_target_with_wanted_candidates_stays_alive():
    """The unstick lane owns motion problems: candidates that exist but resist
    planning must not be ended by this function (the residual documented on
    D38's register row -- ruled out of this batch's scope, not forgotten)."""
    assert empty_cycle_outcome(False, DEBOUNCE, DEBOUNCE, 3) is None


def test_the_node_consumes_the_pure_decision():
    """AST guard: the goal_cell-None branch must call empty_cycle_outcome, and
    the old un-endable-by-construction gate must be gone from the latch."""
    tree = ast.parse(NODE_SRC)
    called = {getattr(c.func, "id", None) or getattr(c.func, "attr", None)
              for c in ast.walk(tree) if isinstance(c, ast.Call)}
    assert "empty_cycle_outcome" in called, (
        "coverage_explorer_node no longer consults the pure end decision")
    assert ("self._ever_had_target and self._consecutive_empty" not in NODE_SRC), (
        "the old latch gate is back -- the exact construction that made a "
        "never-had-target mission silent forever")
    assert "OUTCOME_NO_TARGETS_FROM_START, res, 0" in NODE_SRC, (
        "the D38 ending no longer reaches _finish with its own name")
