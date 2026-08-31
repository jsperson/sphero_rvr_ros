"""Body-frame motion expressed as a map-frame destination.

WHY THIS EXISTS. Scott asked for it twice -- "move forward 2 metres" and "pivot 180
and drive forward 1 m" -- and scenario 8 (2026-08-31) scored it as one of three
instruction classes the robot has no verb for. `goto` takes ABSOLUTE map-frame x/y,
so "forward 2 m" was not a command the rover had: it was trigonometry the language
agent would have to do for itself, from `where_am_i`, unguarded and untested. An
unverified composition is not a verb.

WHAT THIS IS AND IS NOT. It is a COORDINATE TRANSFORM, not a manoeuvre and not a new
motion authority. It answers "where does the rover end up" and says nothing about how
it gets there -- the planner and the controller decide that, through every gate they
already enforce. In particular it does NOT mean "drive straight": a destination behind
the rover is a destination, and the stack may reach it by any route it considers safe.

THE COMPOSE CASE USES THE EXISTING VERB. "Pivot 180 then forward 1 m" is `turn(180)`
followed by `move_relative(1.0)`, not a new two-phase primitive -- the pivot already
has a verb with its own gateway and its own admission rules, and duplicating that
inside a motion verb would put a second author on the manoeuvre this project has
already paid to get right once.

Pure core: no ROS, no clients, no motion. Metres and radians in, metres out.
"""

from __future__ import annotations

import math


class RelativeMotionError(ValueError):
    """The request cannot be turned into a destination. Raised rather than clamped:
    a silently-adjusted distance is a different instruction from the one given."""


#: Bound on a single relative move, in metres. NOT a safety limit -- the envelope and
#: the supervisor are that, downstream and unchanged. This is a CONTRACT sanity bound
#: in the same spirit as `turn`'s +-180: a number outside it is a mistake in the
#: request rather than an intention, and the boundary should say so before any ROS
#: call happens. 5 m is well beyond any room this rover works in and well inside the
#: goal envelope, so it can only ever refuse nonsense.
MAX_DISTANCE_M = 5.0

#: A move shorter than this is a no-op the goal tolerance would swallow whole:
#: `xy_goal_tolerance` is 0.12 m in the deployed config, so a 0.05 m request would
#: report success without the rover having moved. Refuse it in words instead.
MIN_DISTANCE_M = 0.15


def relative_goal(pose_xy_yaw, distance_m, heading_deg=0.0):
    """Map-frame (x, y) for a body-frame move.

    `pose_xy_yaw` is the rover's current (x, y, yaw) in the map frame; `heading_deg`
    is the direction of travel in the BODY frame -- 0 straight ahead, +90 to the
    rover's left, 180 behind. Distance is along that direction.

    The yaw comes from TF at call time and is the whole reason this belongs in one
    place: every caller that did this arithmetic for itself would be a new opportunity
    to add the heading to the wrong angle, which is the mistake the ~179 deg laser
    mount already taught this project once.
    """
    if pose_xy_yaw is None:
        raise RelativeMotionError(
            "no pose: a relative move is measured from where the rover IS, and "
            "nothing knows that right now")
    distance = float(distance_m)
    if not math.isfinite(distance):
        raise RelativeMotionError(f"distance must be a finite number, got {distance_m!r}")
    if distance < MIN_DISTANCE_M:
        raise RelativeMotionError(
            f"{distance:.2f} m is under the {MIN_DISTANCE_M} m minimum: the goal "
            f"tolerance would report success without the rover moving")
    if distance > MAX_DISTANCE_M:
        raise RelativeMotionError(
            f"{distance:.2f} m exceeds the {MAX_DISTANCE_M} m single-move bound")
    heading = float(heading_deg)
    if not math.isfinite(heading) or not -180.0 <= heading <= 180.0:
        raise RelativeMotionError(
            f"heading must be within [-180, 180] degrees, got {heading_deg!r}")

    x, y, yaw = pose_xy_yaw
    course = yaw + math.radians(heading)
    return (x + distance * math.cos(course), y + distance * math.sin(course))


def describe(distance_m, heading_deg=0.0):
    """Plain words for the request, for the tool's own answer. A verb that cannot say
    back what it understood is a verb whose misreadings are invisible."""
    heading = float(heading_deg)
    if abs(heading) < 1e-9:
        return f"forward {float(distance_m):.2f} m"
    if abs(abs(heading) - 180.0) < 1e-9:
        return f"backward {float(distance_m):.2f} m"
    side = "left" if heading > 0 else "right"
    return f"{float(distance_m):.2f} m at {abs(heading):.0f} degrees to the {side}"
