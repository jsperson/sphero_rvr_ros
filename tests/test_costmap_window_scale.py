"""The inscribed threshold has ONE author, and it is on the OccupancyGrid scale.

WHAT THIS PINS, and what it cost to learn (2026-08-16, gauntlet mission 1):
`costmap_window` kept its own copy of `INSCRIBED_COST` set to **253** -- the raw
`costmap_2d` scale -- while the explorer's gate used **99**, the value Nav2 actually
publishes on a `nav_msgs/OccupancyGrid`. The gate was right and refused correctly. The
D43 auto-dump, built specifically to convict that refusal, compared 99 >= 253, printed
`centre_blocked=False`, and drew a robot buried in inscribed cost as open-ish `+`.

The instrument reported the exact inverse of the truth, on its first flight, and was
believed for four minutes.

Equality alone is not enough here -- both copies could drift to 253 together and agree
perfectly while being wrong about the robot. So the SCALE is asserted as a physical
fact about the message type, not just the agreement.
"""

import pytest

from sphero_rvr_core import costmap_window
from sphero_rvr_core.coverage_exploration import INSCRIBED_COST as GATE_THRESHOLD
from sphero_rvr_core.costmap_window import (
    INSCRIBED_COST as DUMP_THRESHOLD, extract_window, format_window,
)

#: `nav_msgs/OccupancyGrid.data` is `int8[]` documented 0..100 with -1 unknown. Nav2's
#: costmap→grid conversion maps lethal 254 -> 100 and inscribed 253 -> 99.
OCCUPANCY_GRID_MAX = 100


def test_one_author_for_the_threshold():
    """Not merely equal -- the SAME object. A second assignment anywhere re-opens the
    defect, and equality would still pass the day someone 'helpfully' redefines it."""
    assert DUMP_THRESHOLD is GATE_THRESHOLD


def test_the_threshold_is_on_the_occupancy_grid_scale():
    """The invariant that catches BOTH copies drifting together. 253 is unreachable on
    the message this code consumes, so a threshold above 100 can never fire -- it is not
    a strict gate, it is a disabled one."""
    assert DUMP_THRESHOLD <= OCCUPANCY_GRID_MAX
    assert DUMP_THRESHOLD == 99


def test_costmap_window_module_defines_no_second_copy():
    """No raw-costmap-scale literal may appear in this module's CODE.

    Checked by AST rather than by string search, and that distinction matters: the
    module's docstring now explains the 253 defect at length, so a grep-based guard
    fails on its own explanation. A guard that forces you to delete the account of the
    bug in order to stay green is a guard that will be deleted instead.
    """
    import ast
    import inspect
    tree = ast.parse(inspect.getsource(costmap_window))
    literals = [n.value for n in ast.walk(tree)
                if isinstance(n, ast.Constant) and isinstance(n.value, int)]
    assert 253 not in literals, "a raw-costmap-scale threshold is back in costmap_window"
    assigned = [t.id for n in ast.walk(tree) if isinstance(n, ast.Assign)
                for t in n.targets if isinstance(t, ast.Name)]
    assert "INSCRIBED_COST" not in assigned, (
        "INSCRIBED_COST is assigned here again; it must be imported, one author")


# --- the behaviour the wrong constant broke -----------------------------------------

def _grid(value, width=5, height=5):
    return [value] * (width * height)


def test_an_inscribed_cell_reads_as_blocked():
    """The gauntlet-mission case, reduced: a robot standing on 99. Against the old 253
    this returned False and the mission's central diagnostic was inverted."""
    window = extract_window(_grid(99), 5, 5, 0.0, 0.0, 0.05, 0.125, 0.125, 0.10)
    assert window is not None
    assert window.centre_value == 99
    assert window.centre_is_blocked is True


def test_an_inscribed_cell_renders_as_hash_not_plus():
    """The picture is the evidence. Tonight's dump drew a buried robot as a field of
    `+` (occupied-but-passable) when every one of those cells was `#`."""
    window = extract_window(_grid(99), 5, 5, 0.0, 0.0, 0.05, 0.125, 0.125, 0.10)
    text = format_window(window, 0.125, 0.125)
    assert "inscribed=99" in text
    assert "centre_blocked=True" in text
    body = [ln for ln in text.splitlines() if set(ln) <= set("#+.? R")]
    assert any("#" in line for line in body), "inscribed cells must render as #"
    assert not any("+" in line for line in body), "99 is inscribed, not merely occupied"


def test_a_below_inscribed_cell_still_reads_as_passable():
    """The other direction, so the fix is not just 'call everything blocked'."""
    window = extract_window(_grid(50), 5, 5, 0.0, 0.0, 0.05, 0.125, 0.125, 0.10)
    assert window.centre_is_blocked is False
    assert "+" in format_window(window, 0.125, 0.125)


def test_lethal_is_also_blocked():
    window = extract_window(_grid(100), 5, 5, 0.0, 0.0, 0.05, 0.125, 0.125, 0.10)
    assert window.centre_is_blocked is True


def test_unknown_is_not_blocked_and_not_free():
    """-1 must stay its own answer. Reading unknown as free is how a dump invents open
    floor around a robot that is actually against the map edge."""
    window = extract_window(_grid(-1), 5, 5, 0.0, 0.0, 0.05, 0.125, 0.125, 0.10)
    assert window.centre_value is None
    assert window.centre_is_blocked is None
    assert "?" in format_window(window, 0.125, 0.125)
