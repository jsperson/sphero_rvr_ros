"""Freeze marks: the shared data model of "a place the robot proved it could not pass".

MOVED VERBATIM from decisive_control.py (2026-08-21, the project-review split —
the keystone that unlocks the bespoke family's retirement): these four symbols
are the touch port's and the mission report's LIVE dependency, and they were
living inside the bespoke controller's module only for historical reasons. The
class docstrings below, including the measured never-un-marks behaviour and the
D35 counting lesson, are the originals — nothing here changed but the address.

Live consumers: contact_marker_node (FreezeMarkSet), contact_marking
(freeze_mark_pose), mission_report (merge_positions).
"""

from dataclasses import dataclass
import math



@dataclass(frozen=True)
class FreezeMark:
    """A place the robot proved it could not pass, with an expiry."""
    x: float
    y: float
    stamp: float
    expires_at: float


class FreezeMarkSet:
    """The live set of freeze marks, with TTL owned HERE rather than in the costmap.

    WHAT THE TTL ACTUALLY BOUNDS -- measured, because the intuitive reading is wrong:
    it governs what this set PUBLISHES and what reaches the mission report. It does
    NOT un-mark the costmap. The layer these feed runs with clearing:false (it must:
    the lidar sees straight through these obstacles and would otherwise erase them),
    and an ObstacleLayer never un-marks a cell it has marked. Verified on the bench
    2026-08-10: a free cell went cost 0 -> 100 on a mark and stayed 100 after
    publication stopped. In practice a mark therefore lasts the mission, which is the
    useful scope; revoking one mid-mission would need a different mechanism.

    Why they cannot simply be another source of the existing obstacle layer: that
    layer raytrace-clears from `scan`, and the entire premise of a freeze is that the
    lidar sees straight THROUGH the obstacle. The marks would be erased within a scan
    or two by the one sensor that is blind to them. They need a separate,
    non-clearing layer.

    Geometry note: a mark records the pose where motion stopped, and the consumer
    inflates it by roughly the robot radius. We know the robot could not pass HERE;
    we know nothing about the obstacle's true extent, so we mark the footprint we
    proved was blocked rather than guessing at the object.
    """

    def __init__(self, ttl_s: float = 300.0, merge_radius_m: float = 0.15):
        self._ttl_s = float(ttl_s)
        self._merge_radius_m = float(merge_radius_m)
        self._marks: list = []


    def add(self, x: float, y: float, now: float) -> FreezeMark:
        """Record a freeze. Re-freezing within `merge_radius_m` refreshes the existing
        mark rather than stacking duplicates -- a rover that retries the same spot
        three times has learned one fact, not three. (Deduplication is real and
        useful; expiry bounds publication and the report only -- see the class
        docstring.)"""
        self.prune(now)
        for i, m in enumerate(self._marks):
            if math.hypot(m.x - x, m.y - y) <= self._merge_radius_m:
                refreshed = FreezeMark(m.x, m.y, m.stamp, now + self._ttl_s)
                self._marks[i] = refreshed
                return refreshed
        mark = FreezeMark(float(x), float(y), float(now), now + self._ttl_s)
        self._marks.append(mark)
        return mark

    def prune(self, now: float) -> None:
        self._marks = [m for m in self._marks if m.expires_at > now]

    def live(self, now: float) -> list:
        self.prune(now)
        return list(self._marks)

    def as_report_list(self, now: float) -> list:
        """Plain dicts for the mission report. These belong in the REPORT, never in
        the saved map: the map is the room as SLAM measured it, while these are the
        robot's own belief about places it could not go."""
        return [{"x": round(m.x, 3), "y": round(m.y, 3), "stamp": round(m.stamp, 1)}
                for m in self.live(now)]

    def __len__(self) -> int:
        return len(self._marks)


def merge_positions(points, merge_radius_m: float) -> list:
    """The distinct PLACES behind a sequence of freeze events.

    Exists because the mission report needs the same answer the controller already
    computes, and a second implementation of "within merge_radius_m" is a second thing
    to keep in sync. So this does not re-implement the rule -- it runs the events
    through :class:`FreezeMarkSet` itself, with expiry disabled so that every event
    counts regardless of when it happened. Whatever the controller's merge does, this
    does, by construction rather than by review.

    Expiry is switched off ON PURPOSE. The set's TTL bounds what is PUBLISHED to the
    costmap layer (see the class docstring); a report is a summary of the whole
    mission, so an obstacle discovered in the first minute is still one of the places
    the rover could not go at minute nine.

    D35: the explorer appended one dict per freeze event and the report field was named
    as though it held places. Run 112721 filed nine entries for six positions --
    ``(-0.847,-1.094)`` four times -- and the run's own author read it as nine
    obstacles within the hour. Counting is the fix; naming is the other half of it.
    """
    marks = FreezeMarkSet(ttl_s=float("inf"), merge_radius_m=merge_radius_m)
    for x, y in points:
        marks.add(float(x), float(y), 0.0)
    return [(m.x, m.y) for m in marks.live(0.0)]

def freeze_mark_pose(x, y, yaw, front_m, rear_m, reversing=False):
    """Where a freeze mark goes: the edge of the footprint that was DRIVING INTO it.

    The leading edge, not the centre: a mark at the centre sits `footprint_front_m`
    behind the obstacle it marks along the approach heading, so the costmap gets a
    point where the robot was standing rather than where the thing it hit was, and an
    approach from a slightly different angle reaches the same object without ever
    crossing a mark (the contact-by-contact face-walking of run 20260811_093818).

    AND THE EDGE DEPENDS ON THE DIRECTION OF TRAVEL. `reversing` is the commanded
    motion's sign, and it must be the COMMAND rather than anything inferred: a freeze
    while backing out means the obstacle is BEHIND, and marking the front edge then
    plants a lethal disc on clear floor ahead while leaving the real obstacle unmarked
    -- poisoning the costmap in the one situation where the rover most needs the floor
    ahead of it to stay plannable. Nothing in the shipped ladder can reach that case
    (freezes are classified only when a rung is NOT running, where the command is
    never negative), which is exactly why it survived: it becomes reachable the moment
    an escape reverses on its own.

    `rear_m` rather than `front_m` for the trailing edge, because this footprint is
    not symmetric: 0.11 m front, 0.16 m rear as deployed.
    """
    reach = -float(rear_m) if reversing else float(front_m)
    return (x + reach * math.cos(yaw), y + reach * math.sin(yaw))
