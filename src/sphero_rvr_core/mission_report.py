"""What a mission produced, as data rather than as log lines to be read back.

Two pieces, both pure so they can be tested without ROS or hardware:

  * :func:`build_report` — the machine-readable summary a run ends with. The outcome
    is an explicit enum, not prose, because the distinction that matters most here is
    between "covered everything" and "ran out of reachable targets", and that
    distinction was previously buried in log text and inferred from a blacklist count.
  * :func:`occupancy_grid_to_pgm` / :func:`map_yaml_text` — the saved map, written in
    the standard map_server PGM+YAML pair.

The map is written directly from the OccupancyGrid rather than by calling
nav2_map_server's ``/map_saver/save_map``, so saving does not depend on another node
being up and cannot fail at the one moment it is needed. Losing a map is expensive
here: a 34-minute SLAM map was destroyed by a stack restart on 2026-08-08 and took
both arms of an A/B with it.
"""

from sphero_rvr_core.decisive_control import merge_positions

# The radius at which two freeze events are one place. CHANGE BOTH OR NEITHER: this
# must equal decisive_controller_node's `freeze_mark_merge_radius_m` default, because
# the report's "distinct positions" claim is only true if it merges the way the
# controller that produced the events merged. tests/test_freeze_mark_reporting.py
# fails in CI if the two drift.
#
# The honest better answer, deferred: the controller should PUBLISH its merge radius
# on the freeze event, so the explorer merges at the radius the controller actually
# used instead of at a matching literal. That is a message-contract change and waits
# for the ToF frame fix to land and the Pi to be current --
# docs/reverse_before_give_up_design.md carries it in the later-batch list.
DEFAULT_FREEZE_MARK_MERGE_RADIUS_M = 0.15

OUTCOME_COMPLETE = "COMPLETE"
OUTCOME_NO_PLANNABLE_TARGETS = "INCOMPLETE_NO_PLANNABLE_TARGETS"
OUTCOME_START_BLOCKED = "INCOMPLETE_START_BLOCKED"
# Goals are being accepted and then failing, over and over, wherever they are sent.
# That is a broken stack, not a hard room: on 2026-08-09 the controller sat in a
# permanent "boxed in" state and killed 82 of 93 goals in ~70 ms each while the rover
# thrashed. A mission must give up and say so rather than grind through the whole map.
OUTCOME_GOALS_KEEP_FAILING = "ABORTED_GOALS_KEEP_FAILING"
# The room is cluttered with things no sensor can see, not the stack broken. A freeze
# is positive evidence (the supervisor permitted motion and the rover did not move),
# so repeated freezes deserve their own honest ending rather than being reported as
# "goals keep failing" -- which blames the software for a fact about the furniture.
OUTCOME_BLOCKED_BY_UNSEEN_OBSTACLES = "INCOMPLETE_BLOCKED_BY_UNSEEN_OBSTACLES"

ALL_OUTCOMES = (
    OUTCOME_COMPLETE,
    OUTCOME_NO_PLANNABLE_TARGETS,
    OUTCOME_START_BLOCKED,
    OUTCOME_GOALS_KEEP_FAILING,
    OUTCOME_BLOCKED_BY_UNSEEN_OBSTACLES,
)

# map_server's PGM convention.
PGM_FREE = 254
PGM_OCCUPIED = 0
PGM_UNKNOWN = 205


def build_report(
    outcome,
    covered_cells,
    resolution,
    duration_s,
    goals_sent=0,
    goals_succeeded=0,
    goals_aborted=0,
    goals_aborted_after_recovery=None,
    goals_aborted_without_recovery=None,
    planner_rejections=0,
    remaining_candidates=0,
    map_files=None,
    freeze_events=None,
    freeze_mark_merge_radius_m=DEFAULT_FREEZE_MARK_MERGE_RADIUS_M,
    escape_events=None,
    escape_poses=None,
):
    """The end-of-mission summary, as a plain dict ready to serialize.

    ``outcome`` must be one of the OUTCOME_* constants. The caller is expected to have
    already decided which applies; encoding it as a field rather than a sentence is the
    point of this module -- a consumer should never have to parse prose to learn
    whether the room was actually covered.

    ``remaining_candidates`` is what makes an incomplete run diagnosable: it says how
    much ground the explorer still wanted when it stopped. **None means UNKNOWN** and
    serializes to JSON null -- reserved for the case where the count genuinely cannot
    be taken (no map, no pose). It must never be used as a stand-in for zero: a
    fabricated 0 reads as "nothing left worth going to", which is the most
    reassuring possible claim and was untrue on both 2026-08-10 runs (D24).

    ``freeze_events`` is one entry per freeze the controller reported, in order. The
    report publishes it under that name, plus the merged ``freeze_positions`` and a
    ``freeze_mark_counts`` block carrying both counts and the radius they were merged
    at. It was called ``freeze_marks`` and held only the events, which is D35.
    """
    if outcome not in ALL_OUTCOMES:
        raise ValueError(f"unknown outcome {outcome!r}")
    _marks = [{"x": round(float(m["x"]), 3), "y": round(float(m["y"]), 3)}
              for m in (freeze_events or [])]
    _places = merge_positions(((m["x"], m["y"]) for m in _marks),
                              freeze_mark_merge_radius_m)
    return {
        "outcome": outcome,
        "complete": outcome == OUTCOME_COMPLETE,
        "covered_cells": int(covered_cells),
        "covered_area_m2": round(int(covered_cells) * resolution * resolution, 3),
        "duration_s": round(float(duration_s), 1),
        "goals": {
            "sent": int(goals_sent),
            "succeeded": int(goals_succeeded),
            "aborted": int(goals_aborted),
            # THE ABORT SPLIT. `aborted` alone cannot support the conclusion that
            # ABORTED_GOALS_KEEP_FAILING implies -- that the rover tried and the room
            # refused. Only the aborts where a RECOVERY ran carry that evidence; an
            # abort with no recovery (the 2026-08-13 follow_path acknowledgement
            # timeout is the recorded example) says something about the stack's
            # concurrency and nothing about the room.
            #
            # NAMED FOR THE SIGNAL, NOT THE QUESTION. `without_recovery` is not
            # "never tried": a goal the rover drove at normally and failed lands here
            # too. What the pair supports is "how much of this ending is backed by a
            # recovery attempt", which is exactly what the gauntlet's no-count rule
            # needs and is all the controller's `ladder_active` signal can say.
            #
            # None means UNKNOWN and serializes to null -- for callers that predate
            # the split. Never defaulted to 0: a fabricated zero in
            # `without_recovery` reads as "every abort was earned", the most
            # reassuring possible claim, which is the D24 mistake in a new field.
            "aborted_after_recovery": (
                None if goals_aborted_after_recovery is None
                else int(goals_aborted_after_recovery)
            ),
            "aborted_without_recovery": (
                None if goals_aborted_without_recovery is None
                else int(goals_aborted_without_recovery)
            ),
            "planner_rejections": int(planner_rejections),
        },
        "remaining_candidates": (
            None if remaining_candidates is None else int(remaining_candidates)
        ),
        # Places the robot PROVED it could not pass. In the report, never in the
        # saved map: the map is the room as SLAM measured it, these are the robot's
        # own belief about where it could not go.
        #
        # EVENTS AND PLACES ARE DIFFERENT NUMBERS AND BOTH ARE NAMED (D35). The old
        # field was called `freeze_marks` and held one entry per event, so run
        # 112721's nine entries for six positions read as nine obstacles -- to its own
        # author, an hour after the run. The field is RENAMED rather than merely
        # supplemented: leaving `freeze_marks` in place would leave every
        # `len(report["freeze_marks"])` in the world still quietly wrong, whereas a
        # rename makes that reader fail loudly and get fixed. `freeze_events` is a
        # list of events and says so; `freeze_positions` is the merged places.
        #
        # SCOPE: a mark lasts the MISSION by design. The costmap layer that carries
        # them runs clearing:false (it must -- the lidar sees straight through these
        # obstacles and would otherwise erase them), and an ObstacleLayer never
        # un-marks a cell. Measured 2026-08-10: a marked cell stayed lethal after
        # publication stopped. Revoking a mark mid-mission would need a real
        # mechanism; the publisher's TTL bounds what is published, not the grid.
        "freeze_events": _marks,
        "freeze_positions": [{"x": round(x, 3), "y": round(y, 3)} for x, y in _places],
        # The radius travels WITH the count, because "6 distinct positions" is not a
        # fact about the room until you know at what separation two freezes were
        # called one place. A reader of the archived report should never have to go
        # find out which config produced it.
        "freeze_mark_counts": {
            "events": len(_marks),
            "distinct_positions": len(_places),
            "merge_radius_m": round(float(freeze_mark_merge_radius_m), 3),
        },
        # THE GIVE-UP ESCAPES: what was attempted when nothing would plan, and how
        # each one turned out. EVENTS AND DISTINCT PLACES, separately and on purpose.
        # Mission 1 on 2026-08-12 reported `freeze_marks: 9` for six distinct
        # positions and its own author read that as nine obstacles an hour later --
        # a count of events reads as a count of places unless the report says which
        # it is (D35). `outcomes` is a histogram over the escape_outcome vocabulary,
        # so a run that never escaped and a run whose escapes were all declined are
        # not the same empty-looking line.
        "give_up_escapes": {
            "attempted": len(list(escape_events or [])),
            "outcomes": _histogram(escape_events or []),
            "distinct_poses": len({tuple(p) for p in (escape_poses or [])}),
        },
        "map_files": list(map_files or []),
    }


def _histogram(items):
    out = {}
    for item in items:
        out[item] = out.get(item, 0) + 1
    return out


def occupancy_grid_to_pgm(data, width, height, occupied_at=65, free_at=25):
    """Serialize an OccupancyGrid's cells as binary P5 PGM bytes.

    Rows are flipped: an OccupancyGrid's row 0 is the BOTTOM (it grows upward from
    ``origin``), while PGM's first row is the top. Getting this backwards produces a
    map that is silently mirrored -- readable, plausible, and wrong.
    """
    header = f"P5\n{int(width)} {int(height)}\n255\n".encode("ascii")
    rows = []
    for row in range(height - 1, -1, -1):
        base = row * width
        rows.append(bytes(
            PGM_UNKNOWN if v < 0 else
            PGM_OCCUPIED if v >= occupied_at else
            PGM_FREE if v <= free_at else
            PGM_UNKNOWN
            for v in data[base:base + width]
        ))
    return header + b"".join(rows)


def map_yaml_text(image_name, resolution, origin_x, origin_y, origin_yaw=0.0):
    """The map_server sidecar YAML naming the PGM and locating it in the world."""
    return (
        f"image: {image_name}\n"
        f"mode: trinary\n"
        f"resolution: {resolution}\n"
        f"origin: [{origin_x}, {origin_y}, {origin_yaw}]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.25\n"
    )
