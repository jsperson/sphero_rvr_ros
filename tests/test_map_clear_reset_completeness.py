"""The map-clear reset registry cannot silently fall behind the state it clears.

Source-level (ast), so it runs without rclpy anywhere. The drift this pins is
exactly how D61 happened once: a map-tied latch (`_mission_done`) that nothing
reset. Every zero-initialised `self._x = <empty/zero/None/False>` in the
explorer's __init__ must be either a MAP_CLEAR_RESETS row or a REASONED entry
in the allowlist below — a new counter lands red here, not as stale belief
surviving a map clear in the field.
"""

import ast
from pathlib import Path

NODE = (Path(__file__).resolve().parents[1]
        / "src" / "sphero_rvr_driver" / "coverage_explorer_node.py")

#: NOT reset on map clear, each for a stated reason. An entry here without a
#: real reason is the same drift wearing a costume — reviewers, read these.
ALLOWED_UNRESET = {
    # Cursors into OTHER nodes' monotonic counters (stall counter, ladder
    # seam). Those counters do NOT reset when our map does; zeroing our cursor
    # would misread the next delta as a giant burst — counters-not-levels.
    "_last_stall_count",
    "_ladder_active_at",
    # Monotonic goal-generation id published to consumers; resetting it aliases
    # deltas for anything reading generations across the clear.
    "_active_goal_generation",
    # Owned by _cancel_active, which _on_map_clear calls before the registry
    # runs — cancel needs the live handle, so these cannot be table rows.
    "_active_goal_handle",
    "_active_goal_cell",
    # D75's per-goal ending flag. It describes the GOAL, not the map: every
    # `_send_goal` re-initialises it to False, and `_cancel_active` -- which
    # `_on_map_clear` calls -- reads it to decide whether the goal it is about to
    # cancel already has a named ending. A registry row would zero it AFTER that
    # read and before the next send, changing nothing; putting it in the table
    # would imply a map-tied belief it does not hold.
    "_active_goal_ended",
}


def _zeroish(value):
    if isinstance(value, ast.Constant) and value.value in (0, 0.0, False, None):
        return True
    if isinstance(value, (ast.List, ast.Set)) and not value.elts:
        return True
    if isinstance(value, ast.Dict) and not value.keys:
        return True
    if isinstance(value, ast.Call) and not value.args:
        return getattr(value.func, "id", "") in ("set", "list", "dict")
    return False


def _parse():
    tree = ast.parse(NODE.read_text())
    init_state, registry = set(), set()
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name)
                and node.targets[0].id == "MAP_CLEAR_RESETS"):
            registry = {key.value for key in node.value.keys}
        if isinstance(node, ast.ClassDef) and node.name == "CoverageExplorerNode":
            for fn in node.body:
                if isinstance(fn, ast.FunctionDef) and fn.name == "__init__":
                    for stmt in ast.walk(fn):
                        if (isinstance(stmt, ast.Assign)
                                and len(stmt.targets) == 1
                                and isinstance(stmt.targets[0], ast.Attribute)
                                and isinstance(stmt.targets[0].value, ast.Name)
                                and stmt.targets[0].value.id == "self"
                                and _zeroish(stmt.value)):
                            init_state.add(stmt.targets[0].attr)
    assert registry, "MAP_CLEAR_RESETS vanished from coverage_explorer_node"
    assert init_state, "found no zero-initialised state — the extractor broke"
    return init_state, registry


def test_every_map_tied_field_is_reset_or_reasoned():
    init_state, registry = _parse()
    unaccounted = init_state - registry - ALLOWED_UNRESET
    assert not unaccounted, (
        f"zero-initialised state with no MAP_CLEAR_RESETS row and no reasoned "
        f"allowlist entry: {sorted(unaccounted)} — stale belief would survive "
        f"a map clear")


def test_the_registry_names_no_ghosts():
    init_state, registry = _parse()
    ghosts = registry - init_state
    assert not ghosts, (
        f"MAP_CLEAR_RESETS rows for state __init__ no longer creates: "
        f"{sorted(ghosts)} — a renamed field kept its old reset row")
    overlap = registry & ALLOWED_UNRESET
    assert not overlap, f"in BOTH the registry and the allowlist: {sorted(overlap)}"


def test_the_clear_applies_the_registry_not_a_hand_list():
    src = NODE.read_text()
    body = src[src.index("def _on_map_clear"):]
    body = body[:body.index("\n    def ")]
    assert "MAP_CLEAR_RESETS.items()" in body, (
        "_on_map_clear no longer iterates the registry — a hand list is the "
        "drift shape this file exists to prevent")
