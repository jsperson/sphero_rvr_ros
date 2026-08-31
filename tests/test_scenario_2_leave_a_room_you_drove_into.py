"""SCENARIO 2 of Scott's nine, and D60's falsifier -- the instrument that row has
never had.

  REQUIREMENT, his words: "It should be able to leave a room it drove into."

  BAR, PRE-REGISTERED: a path of cells < 253 exists from the STUCK pose to the
  STANDING pose on the OPEN frame, and does NOT exist on the CLOSED frame -- while
  the lidar's own returns over the closed period support none of the newly-blocked
  cells.

  PREDICTION FILED BEFORE EXECUTION: FAIL (D60 open).

WHY CONNECTIVITY AND NOT "IS THE DOOR BLOCKED". The corridor did not close because a
doorway filled in. The 447 newly-blocked cells fall in FOURTEEN connected components
spread over about 2.6 x 3.4 m -- 23 seeds each inflated x19, not a doorway. What was
lost is CONNECTIVITY, which is what the archive's round 2 measured directly. A
falsifier asserting "the door is blocked" would test a geometric claim the artifact
never made. (An earlier note of mine, "that is not a door", read the artifact's
shorthand as a geometric claim and was withdrawn: it reports nearest-cell distances
and belief-without-physics, never a contiguous doorway.)

WHY RECORDED FRAMES AND NOT A SYNTHETIC ROOM. These grids are the room as the rover
actually believed it, immediately before and after the closure -- extracted by
`scripts/extract_d60_frames.py`, which refuses to write unless it reproduces the
archive's 447/23. A pocket we authored would certify a shape we chose.

WHAT THIS IS AND IS NOT. It is a RECORDED-STATE falsifier: given this belief state,
can the rover plan its way out? It does NOT reproduce the painting process -- the 23
seeds come from `/contact_marks` (1824 of 7825 mark points land on them), not from
the three `/contact_marks/promote` messages, so replaying the promotions would not
regenerate them. Any future exit-plannability clause or escape primitive gets
certified against THIS, not against a story about it.

Archive cross-checks, all re-run 2026-08-31 against the bag and reproducing exactly:
447 newly >=253 of which 23 newly 254 (`basin_geometry4.py`); 0 of 447 door cells
supported by any of 23,027 live returns over 35 scans in the closed period
(`basin_geometry3.py`).
"""

from __future__ import annotations

import os

import pytest

ART = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "artifacts", "d60_falsifier")
STUCK, STANDING = (0.209, 0.121), (0.866, -0.009)
BLOCKED = 253


def _read_pgm(path):
    values, dims = [], None
    for line in open(path):
        if line.startswith("#"):
            continue
        parts = line.split()
        if dims is None:
            if parts == ["P2"]:
                continue
            if len(parts) == 2:
                dims = (int(parts[1]), int(parts[0]))   # (height, width)
                continue
            continue
        if parts == ["255"] and not values:
            continue
        values.extend(int(v) for v in parts)
    height, width = dims
    return [values[r * width:(r + 1) * width] for r in range(height)]


def _geometry():
    out = {}
    for line in open(os.path.join(ART, "geometry.txt")):
        key, value = line.split()
        out[key] = float(value)
    return out


def _cell(pose, geom):
    return (int((pose[1] - geom["origin_y"]) / geom["resolution"]),
            int((pose[0] - geom["origin_x"]) / geom["resolution"]))


def _connected(grid, start, goal):
    height, width = len(grid), len(grid[0])
    free = lambda y, x: grid[y][x] < BLOCKED
    if not (free(*start) and free(*goal)):
        return False
    seen = {start}
    stack = [start]
    while stack:
        y, x = stack.pop()
        if (y, x) == goal:
            return True
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                ny, nx = y + dy, x + dx
                if (0 <= ny < height and 0 <= nx < width
                        and (ny, nx) not in seen and free(ny, nx)):
                    seen.add((ny, nx))
                    stack.append((ny, nx))
    return False


def test_the_open_frame_still_lets_the_rover_out():
    """THE CONTROL. Without it, a falsifier that always reported 'no path' -- a
    misread grid, a transposed cell -- would score D60 as reproduced for free."""
    geom = _geometry()
    grid = _read_pgm(os.path.join(ART, "d60_open.pgm"))
    assert _connected(grid, _cell(STUCK, geom), _cell(STANDING, geom)), (
        "no path on the OPEN frame -- the falsifier cannot show the failure because "
        "it cannot show the working case either")


@pytest.mark.xfail(strict=True, reason=(
    "SCENARIO 2 / D60: on the CLOSED frame the rover cannot plan from where it stood "
    "to where it had been standing 29 s earlier, through a corridor no lidar return "
    "ever contradicted. Strict xfail so the bar stays at Scott's requirement rather "
    "than at current behaviour -- the day an exit-plannability fix lands, this test "
    "FAILS loudly and forces the row to be re-scored."))
def test_scenario_2_the_rover_can_leave_the_room_it_drove_into():
    geom = _geometry()
    grid = _read_pgm(os.path.join(ART, "d60_closed.pgm"))
    assert _connected(grid, _cell(STUCK, geom), _cell(STANDING, geom)), (
        "D60 reproduced: no path of cells <253 from the stuck pose to the standing "
        "pose on the closed frame")


def test_neither_endpoint_is_itself_blocked():
    """The closure is the CORRIDOR, not the poses. If an endpoint were lethal the row
    would be a different defect -- and a reader would be entitled to think the rover
    had simply parked itself inside an obstacle."""
    geom = _geometry()
    for name in ("d60_open.pgm", "d60_closed.pgm"):
        grid = _read_pgm(os.path.join(ART, name))
        for pose, label in ((STUCK, "stuck"), (STANDING, "standing")):
            y, x = _cell(pose, geom)
            assert grid[y][x] < BLOCKED, f"{label} pose is blocked in {name}: {grid[y][x]}"


def test_the_frames_still_carry_the_archive_s_numbers():
    """Drift pin. If someone regenerates the frames from a different window, the
    counts move and every claim above is about a different pair of grids."""
    geom = _geometry()
    assert geom["newly_blocked"] == 447
    assert geom["newly_lethal"] == 23
    a = _read_pgm(os.path.join(ART, "d60_open.pgm"))
    b = _read_pgm(os.path.join(ART, "d60_closed.pgm"))
    newly = sum(1 for r in range(len(a)) for c in range(len(a[0]))
                if b[r][c] >= BLOCKED > a[r][c])
    assert newly == 447, f"the committed frames differ by {newly} cells, not 447"
