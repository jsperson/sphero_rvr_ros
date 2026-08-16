"""The START_BLOCKED report must carry what happened, and must not invent a zero.

WHAT THIS PINS (gauntlet mission 1, 2026-08-16). The mission ended
`INCOMPLETE_START_BLOCKED`. Its latched report said:

    "freeze_events": [], "freeze_positions": [], "remaining_candidates": 0

while the status line counted **5 freezes** and the launch log named four positions.
Both were artifacts of one omission: the START_BLOCKED branch called `build_report`
without the forensic arguments, so `freeze_events` came back empty and
`remaining_candidates` fell to a default of **0**.

`build_report`'s own docstring forbids exactly that zero -- *"a fabricated 0 reads as
'nothing left worth going to', which is the most reassuring possible claim and was
untrue on both 2026-08-10 runs (D24)"* -- but the forbidding was aimed at callers, and
the lie came from the DEFAULT. The run analysis then quoted `remaining_candidates: 0`
as if the explorer had counted.

Two tests, because there are two ways to reintroduce it: the default, and the call site.
"""

import ast
import inspect
from pathlib import Path

import pytest

from sphero_rvr_core.mission_report import (
    OUTCOME_COMPLETE, OUTCOME_START_BLOCKED, build_report,
)

NODE_SRC = (Path(__file__).resolve().parents[1] / "src" / "sphero_rvr_driver"
            / "coverage_explorer_node.py")


def test_omitting_remaining_candidates_reports_UNKNOWN_not_zero():
    """THE DEFAULT. Before 2026-08-16 this returned 0 and the report claimed, in the
    project's most-read field, that nothing was left to explore."""
    r = build_report(OUTCOME_START_BLOCKED, covered_cells=100, resolution=0.05,
                     duration_s=12.0)
    assert r["remaining_candidates"] is None, (
        "an uncounted remaining_candidates must serialize to null (unknown), never 0")


def test_an_explicit_zero_still_means_zero():
    """The fix must not make a real zero unsayable -- COMPLETE legitimately reports 0."""
    r = build_report(OUTCOME_COMPLETE, covered_cells=100, resolution=0.05,
                     duration_s=12.0, remaining_candidates=0)
    assert r["remaining_candidates"] == 0


def test_freeze_events_survive_into_the_report():
    r = build_report(OUTCOME_START_BLOCKED, covered_cells=100, resolution=0.05,
                     duration_s=12.0,
                     freeze_events=[{"x": -0.98, "y": -1.76}, {"x": 0.11, "y": -0.03}])
    assert len(r["freeze_events"]) == 2
    assert r["freeze_mark_counts"]["events"] == 2


def test_the_start_blocked_call_site_passes_the_forensic_fields():
    """THE CALL SITE. A source-level tripwire, because the branch that produced
    tonight's report only runs on a robot that has wedged itself -- there is no cheap
    unit path to it, and "we will remember to pass them" is what failed.

    Checked by AST rather than by grep so a mention in a comment cannot satisfy it.
    """
    tree = ast.parse(NODE_SRC.read_text())
    blocked_calls = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        name = getattr(func, "id", None) or getattr(func, "attr", None)
        if name != "build_report":
            continue
        first = node.args[0] if node.args else None
        if isinstance(first, ast.Name) and first.id == "OUTCOME_START_BLOCKED":
            blocked_calls.append(node)

    assert blocked_calls, "no build_report(OUTCOME_START_BLOCKED, ...) call site found"
    for call in blocked_calls:
        passed = {kw.arg for kw in call.keywords}
        for required in ("remaining_candidates", "freeze_events"):
            assert required in passed, (
                f"the START_BLOCKED report omits {required!r}; gauntlet mission 1 "
                f"published an empty freeze list and a fabricated 0 that way")
