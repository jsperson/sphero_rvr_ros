"""THE FALSIFIER, run before anything in Part Two is built.

Gauntlet 1 (run 20260811_211237) ended `ABORTED_GOALS_KEEP_FAILING` at pose
(0.428, -0.482, yaw -135.1). Scott, standing over it: *"A 45 degree pivot to the
right and the rover had the entire north side of the room to explore."* The rover's
own lidar agreed — measured at that pose, through the controller's real gap search,
before anything was moved:

    OPEN BEARING: -71.4 deg      (base frame, 0 = nose, + = left)
      -60 deg: 2.07 m   the way out
      -45 deg: 1.88 m
      -30 deg: 1.89 m
       +0 deg: 0.90 m
      +30 deg: 0.28 m   <-- the global minimum over ALL bearings
      +90 deg: 0.37 m
     +135 deg: 0.29 m

Part Two's whole design says the fix is ESCAPE ORDERING: pivot toward the open
bearing first when the stall cause is lidar-visible. That design is only worth
building if the supervisor would actually have GRANTED that pivot. If it would have
refused it too, the rover was genuinely boxed at that pose, ordering is not the fix,
and the finding moves to prevention (never park there) — a different batch.

So this test asks the real `CollisionStopSupervisor`, loaded from the DEPLOYED YAML
(never dataclass defaults — 13 fields differ and several decide this verdict), what
it would have done with the pivot the ladder never reached.

The scan is RECONSTRUCTED from the measured 15-degree profile rather than replayed
raw: the live scan was read through the gap search and not saved, which is an
artifact-capture miss worth naming. It is faithful for this verdict, because the
pivot gate consults `nearest["any"]` — the minimum over all bearings — and the
minimum of the bucket minima IS the global minimum over the valid rays.
"""

import math
import os

import pytest

from sphero_rvr_driver.collision_stop import (
    CollisionState,
    CollisionStopConfig,
    CollisionStopSupervisor,
    ScanInput,
    Transform2D,
    TwistCommand,
)

# Measured at the death pose, base frame, degrees -> metres.
DEATH_POSE_PROFILE = {
    -180: 0.44, -165: 0.57, -150: 1.50, -135: 1.31, -120: 1.54, -105: 1.56,
    -90: 1.34, -75: 1.50, -60: 2.07, -45: 1.88, -30: 1.89, -15: 1.15,
    0: 0.90, 15: 1.08, 30: 0.28, 45: 0.61, 60: 0.74, 75: 0.47,
    90: 0.37, 105: 0.31, 120: 0.30, 135: 0.29, 150: 0.30, 165: 0.33,
}
OPEN_BEARING_DEG = -71.4          # what the controller's own gap search returned
COUNT = 720                       # 0.5 deg resolution
NOW = 5.0


def deployed_config():
    """The config the ROBOT runs, from config/collision_stop.yaml.

    NOT `CollisionStopConfig()`: the dataclass defaults differ from the deployed YAML
    in 13 fields, and `footprint_*` plus `payload_margin_m` decide the pivot radius
    outright. A falsifier run against defaults would answer confidently about a robot
    that does not exist.
    """
    yaml = pytest.importorskip("yaml")
    here = os.path.dirname(__file__)
    raw = yaml.safe_load(open(os.path.join(here, "..", "config", "collision_stop.yaml")))

    def find(d):
        for _k, v in (d or {}).items():
            if isinstance(v, dict):
                if "ros__parameters" in v:
                    return v["ros__parameters"]
                found = find(v)
                if found:
                    return found
        return None

    params = find(raw) or {}
    fields = CollisionStopConfig.__dataclass_fields__
    return CollisionStopConfig(**{k: v for k, v in params.items() if k in fields})


def death_pose_scan(stamp=NOW):
    """Every ray set to its 15-degree bucket's measured minimum: the conservative
    reconstruction, since a bucket minimum is a floor for every ray inside it."""
    inc = 2.0 * math.pi / COUNT
    amin = -math.pi
    ranges = []
    for i in range(COUNT):
        bearing = math.degrees(amin + i * inc)
        bucket = int(round(bearing / 15.0)) * 15
        if bucket == 180:
            bucket = -180
        ranges.append(DEATH_POSE_PROFILE[bucket])
    return ScanInput(
        ranges=tuple(ranges), angle_min=amin, angle_increment=inc,
        range_min=0.05, range_max=8.0, stamp=stamp, received_at=stamp,
        frame_id="laser", transform_to_base=Transform2D(),
    )


def _pivot_radius(cfg):
    """The circumscribed corner circle a pivot sweeps (D18's geometry)."""
    return math.hypot(max(cfg.footprint_front_m, cfg.footprint_rear_m),
                      max(cfg.footprint_left_m, cfg.footprint_right_m)) + cfg.payload_margin_m


def test_the_supervisor_would_have_granted_the_pivot_the_ladder_never_reached():
    """THE FALSIFIER. If this fails, Part Two's ordering fix is not the fix.

    A pure pivot toward the open bearing, at the pose where the mission gave up,
    against the deployed config and the real supervisor.
    """
    cfg = deployed_config()
    sup = CollisionStopSupervisor(cfg, now=NOW)
    sup.update_scan(death_pose_scan(), now=NOW)

    # Turning right (toward -71.4 deg) is negative angular in REP-103.
    pivot = TwistCommand(0.0, math.copysign(cfg.max_angular_rad_s, OPEN_BEARING_DEG))
    decision = sup.apply_command(pivot, now=NOW)

    nearest_any = decision.nearest.get("any")
    radius = _pivot_radius(cfg)
    assert nearest_any == pytest.approx(0.28, abs=0.01), (
        "the reconstruction should preserve the measured global minimum")
    assert nearest_any > radius, (
        f"the pivot sweeps a {radius:.4f} m corner circle and the nearest return at "
        f"the death pose is {nearest_any:.3f} m — the rover was GENUINELY BOXED and "
        "Part Two's ordering fix is answering the wrong question")
    assert abs(decision.output.angular_z) > 0.0, (
        f"the supervisor REFUSED the pivot ({decision.state.value}/{decision.reason}) "
        "— ordering cannot fix this pose and the finding moves to prevention")
    assert decision.output.linear_x == 0.0, "a pivot must not acquire forward motion"


def test_the_margin_is_stated_not_assumed():
    """How much room the verdict had, so nobody has to re-derive it — and so a future
    footprint or margin change that eats it shows up here as a failure rather than as
    a surprise on carpet."""
    cfg = deployed_config()
    radius = _pivot_radius(cfg)
    nearest = min(DEATH_POSE_PROFILE.values())
    assert radius == pytest.approx(0.2087, abs=0.001), (
        f"pivot corner radius moved to {radius:.4f} — re-run the falsifier")
    assert nearest - radius > 0.05, (
        f"margin is only {nearest - radius:.3f} m; at that point the verdict is inside "
        "the reconstruction's own error and the raw scan must be re-captured")


def test_driving_out_along_the_open_bearing_is_also_granted():
    """Rung 4 follows rung 3, so the escape is only real if the drive-out is granted
    too. Checked at the ladder's own forward speed, not cruise."""
    cfg = deployed_config()
    sup = CollisionStopSupervisor(cfg, now=NOW)
    sup.update_scan(death_pose_scan(), now=NOW)
    # Facing the open bearing after the pivot: the profile rotated so -71 deg is ahead.
    rotated = {}
    for deg, r in DEATH_POSE_PROFILE.items():
        moved = int(round(((deg - OPEN_BEARING_DEG + 180) % 360 - 180) / 15.0)) * 15
        if moved == 180:
            moved = -180
        rotated[moved] = min(rotated.get(moved, 99.0), r)
    for deg in DEATH_POSE_PROFILE:
        rotated.setdefault(deg, 6.0)

    inc = 2.0 * math.pi / COUNT
    ranges = []
    for i in range(COUNT):
        bearing = math.degrees(-math.pi + i * inc)
        bucket = int(round(bearing / 15.0)) * 15
        if bucket == 180:
            bucket = -180
        ranges.append(rotated[bucket])
    sup2 = CollisionStopSupervisor(cfg, now=NOW)
    sup2.update_scan(ScanInput(
        ranges=tuple(ranges), angle_min=-math.pi, angle_increment=inc,
        range_min=0.05, range_max=8.0, stamp=NOW, received_at=NOW,
        frame_id="laser", transform_to_base=Transform2D()), now=NOW)
    decision = sup2.apply_command(TwistCommand(0.10, 0.0), now=NOW)
    assert decision.output.linear_x > 0.0, (
        f"the drive-out along the open bearing was refused "
        f"({decision.state.value}/{decision.reason}) — pivoting there would strand it")
