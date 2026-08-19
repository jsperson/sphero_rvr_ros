"""Option D's brain, held to the day's three specimens and the consensus discipline.

The signature constants were derived from run 3c before this module existed; these
tests replay those numbers (from the goal logs -- the action feedback topic was not
in the bags, so the fixtures are synthetic-from-measured, stated honestly) and pin
the two consensus amendments: firing discipline and the wall cap.
"""

import math
from pathlib import Path

import pytest
import yaml

from sphero_rvr_core.refusal_promotion import (
    COOLDOWN_S,
    INSCRIBED,
    LETHAL,
    MAX_DISCS_PER_FIRING,
    STALL_DISPLACEMENT_M,
    WINDOW_S,
    FiringDiscipline,
    Grid,
    LivelockWindow,
    blindness_delta_cells,
    cluster_and_cap,
)


# --- the signature against the specimens ----------------------------------------------

def feed_uniform(window, duration_s, speed_mps, recovery_every_s, goal="g1"):
    """A goal moving at constant speed with periodic recoveries, 5 Hz feedback."""
    fired_at = None
    r = 0
    steps = int(duration_s * 5)
    for i in range(steps):
        t = i / 5.0
        if recovery_every_s and t > 0 and (t % recovery_every_s) < 0.2:
            r += 1
        if window.feed(t, speed_mps * t, 0.0, r, goal) and fired_at is None:
            fired_at = t
    return fired_at


def test_goal_4s_livelock_fires_around_t14():
    """The field specimen: ~0.2 m of total shuffling and 10 recoveries in 19 s.
    Modeled as pinned (0.01 m/s creep) with a recovery every ~2 s: the signature
    must fire once the stall has SUSTAINED a full window -- near t+14, and never
    before the window has even elapsed."""
    fired = feed_uniform(LivelockWindow(), duration_s=19.0, speed_mps=0.01,
                         recovery_every_s=2.0)
    assert fired is not None
    assert WINDOW_S <= fired <= 16.0


def test_goal_2s_healthy_chatter_is_structurally_unfireable():
    """1.46 m in 22 s WITH 8 recoveries: real progress. The displacement gate --
    THE discriminator -- keeps it silent no matter how many recoveries land."""
    fired = feed_uniform(LivelockWindow(), duration_s=22.0, speed_mps=0.066,
                         recovery_every_s=2.75)
    assert fired is None


def test_goal_1s_short_life_dies_before_the_window():
    """10 s from accept to abort: WINDOW_S never elapses, nothing can fire."""
    fired = feed_uniform(LivelockWindow(), duration_s=10.0, speed_mps=0.005,
                         recovery_every_s=2.0)
    assert fired is None


def feed_goal_churn(window, n_goals, goal_life_s, speed_mps,
                    recoveries_per_goal=2, hz=5.0):
    """The 2026-08-19 flight's shape: consecutive short goals over one rover
    pose, each goal's feedback recovery counter STARTING OVER (that is how Nav2
    hands it across goals). Returns the first firing time or None."""
    fired_at = None
    t = 0.0
    for n in range(n_goals):
        steps = int(goal_life_s * hz)
        for i in range(steps):
            # recoveries mount within the goal's own life, then reset with it
            r = min(recoveries_per_goal, int(i / (steps / (recoveries_per_goal + 1))))
            if window.feed(t, speed_mps * t, 0.0, r, f"goal_{n}") and fired_at is None:
                fired_at = t
            t += 1.0 / hz
    return fired_at


def test_d55_stationary_livelock_survives_goal_churn():
    """THE FLIGHT REPLAY, and D55's must-flip. Five ~6.9 s goals over one pose
    (displacement < 0.03 m across all of them), recoveries running and resetting
    per goal -- the exact signature the watcher exists to catch, which the old
    goal-keyed window could not fire on BY CONSTRUCTION (12 s of history could
    never accumulate inside a 6.9 s goal). The spatial key must fire once the
    stall has sustained a full window, and never before WINDOW_S has elapsed."""
    fired = feed_goal_churn(LivelockWindow(), n_goals=5, goal_life_s=6.9,
                            speed_mps=0.001)
    assert fired is not None, (
        "the window still cannot fire under goal churn -- D55's arithmetic "
        "impossibility is back")
    assert WINDOW_S <= fired <= WINDOW_S + 2 * 6.9


def test_d55_the_deployed_watchdog_cannot_starve_the_window():
    """The composed-stack inequality pin, satisfied by construction: goal
    lifetimes equal to the DEPLOYED explorer watchdog (read from the yaml the
    robot actually flies, not a dataclass default) must still let the signature
    fire. If someone tunes goal_progress_timeout_s, this test re-runs the
    composition question automatically -- the next tune cannot silently recreate
    the un-fireable watcher."""
    cfg = yaml.safe_load(
        (Path(__file__).resolve().parents[1] / "config" /
         "coverage_explorer.yaml").read_text())
    timeout_s = float(
        cfg["coverage_explorer"]["ros__parameters"]["goal_progress_timeout_s"])
    fired = feed_goal_churn(LivelockWindow(), n_goals=6,
                            goal_life_s=timeout_s + 0.9, speed_mps=0.001)
    assert fired is not None, (
        f"goals living goal_progress_timeout_s ({timeout_s:.1f}s, deployed "
        "value) starve the livelock window -- the D55 composition defect")


def test_a_goal_change_at_a_new_place_is_a_new_question():
    """What survives of the old goal-reset pin: 'a new goal is a new question'
    is now answered by the ROVER, not the goal id. A full stationary window
    under goal_a, then the rover MOVES and goal_b feeds from the new pose: the
    displacement gate must hold it silent -- history from the old place cannot
    vouch for a stall at a new one."""
    w = LivelockWindow()
    for i in range(int(WINDOW_S * 5) + 10):
        w.feed(i / 5.0, 0.0, 0.0, i // 10, "goal_a")
    t = (int(WINDOW_S * 5) + 10) / 5.0
    assert w.feed(t, 1.0, 0.0, 99, "goal_b") is False


def test_a_straggling_feedback_cannot_inflate_the_recovery_count():
    """Action feedback can straggle: one late message from a replaced goal
    arrives after its successor started. Per-goal counting must keep its counts
    under its own key -- a stationary-but-quiet rover (no recoveries anywhere)
    must not fire because two goals' counters were conflated."""
    w = LivelockWindow()
    t = 0.0
    for i in range(int(WINDOW_S * 5) + 20):
        goal = "goal_b" if i % 7 == 3 else "goal_a"   # stragglers interleaved
        assert w.feed(t, 0.0, 0.0, 1, goal) is False, (
            "recoveries never exceeded 1 on any goal, yet the window fired -- "
            "goal interleaving is inflating the count")
        t += 0.2


def test_no_goal_means_no_signature():
    w = LivelockWindow()
    for i in range(200):
        assert w.feed(i / 5.0, 0.0, 0.0, i, None) is False


def test_recoveries_are_required_not_just_stillness():
    """A robot PAUSED (waiting, planning, held by the supervisor) is not
    livelocked. Stillness without recoveries must not fire -- promotion is for a
    controller fighting its map, not a robot standing quietly."""
    fired = feed_uniform(LivelockWindow(), duration_s=30.0, speed_mps=0.0,
                         recovery_every_s=0)
    assert fired is None


# --- the blindness delta ---------------------------------------------------------------

def grid_with(width=40, height=40, res=0.05, fill=0, cells=()):
    g = Grid(data=[fill] * (width * height), origin_x=0.0, origin_y=0.0,
             resolution=res, width=width, height=height)
    for x, y, v in cells:
        mx, my = int(x / res), int(y / res)
        g.data[my * g.width + mx] = v
    return g


PLAN = [(0.1 * i, 1.0) for i in range(11)]      # straight line y=1.0, x 0..1.0
ROBOT = (0.3, 1.0)


def test_the_boot_shape_is_promoted():
    """Lethal in local, free in global, on the corridor: the exact delta the
    planner was blind to on 2026-08-18."""
    local = grid_with(cells=[(0.6, 1.0, LETHAL)])
    global_ = grid_with()
    cells = blindness_delta_cells(local, global_, PLAN, ROBOT)
    assert cells, "the boot's cell was not promoted"
    assert all(math.hypot(x - 0.6, y - 1.0) < 0.06 for x, y in cells)


def test_a_person_is_excluded_by_the_delta_itself():
    """Lidar-visible legs are lethal in BOTH maps: no delta, never promotable --
    the 5b ambulatory worry, answered structurally."""
    local = grid_with(cells=[(0.6, 1.0, LETHAL)])
    global_ = grid_with(cells=[(0.6, 1.0, LETHAL)])
    assert blindness_delta_cells(local, global_, PLAN, ROBOT) == []


def test_global_inscribed_also_blocks_promotion():
    """A cell the global map already prices at inscribed is not blindness --
    the planner can see it fine; promoting it would double-book."""
    local = grid_with(cells=[(0.6, 1.0, LETHAL)])
    global_ = grid_with(cells=[(0.6, 1.0, INSCRIBED)])
    assert blindness_delta_cells(local, global_, PLAN, ROBOT) == []


def test_off_corridor_lethality_is_not_promoted():
    """A sub-lidar obstacle BESIDE the path did not cause the refusal; promoting
    it would sterilise floor the mission never asked about."""
    local = grid_with(cells=[(0.6, 1.6, LETHAL)])    # 0.6 m off the corridor
    global_ = grid_with()
    assert blindness_delta_cells(local, global_, PLAN, ROBOT) == []


def test_beyond_lookahead_is_not_promoted():
    local = grid_with(cells=[(1.8, 1.0, LETHAL)])    # plan doesn't reach; far away
    global_ = grid_with()
    plan = [(0.2 * i, 1.0) for i in range(11)]        # x 0..2.0
    assert blindness_delta_cells(local, global_, plan, (0.0, 1.0)) == []


# --- the cap and the discipline ---------------------------------------------------------

def test_a_wall_refuses_itself():
    """More clusters than the firing cap is a wall, not an obstacle -- one mark
    closed a 0.51 m corridor once; a promoted wall would do it deliberately."""
    points = [(0.1 * i, 0.0) for i in range(0, 20, 4)]    # 5 spread clusters
    firing = cluster_and_cap(points)
    assert firing.centroids == []
    assert "wall" in firing.suppressed_reason


def test_a_blob_clusters_to_one_disc():
    points = [(0.60, 1.00), (0.62, 1.02), (0.58, 0.99), (0.61, 1.01)]
    firing = cluster_and_cap(points)
    assert len(firing.centroids) == 1
    cx, cy = firing.centroids[0]
    assert math.hypot(cx - 0.60, cy - 1.005) < 0.03


def test_one_firing_per_goal_per_region_until_a_replan_completes():
    d = FiringDiscipline()
    c = [(0.6, 1.0)]
    ok, why = d.allow(0.0, "g", c)
    assert ok and "first firing" in why
    d.record(0.0, "g", c)
    ok, why = d.allow(COOLDOWN_S + 1, "g", c)
    assert not ok and "no replan" in why
    d.note_plan()
    ok, why = d.allow(COOLDOWN_S + 2, "g", c)
    assert ok and "re-fire" in why


def test_the_cooldown_gates_even_new_regions():
    """A broken delta computation must not machine-gun marks across the map --
    the cooldown is global across regions by design."""
    d = FiringDiscipline()
    d.record(0.0, "g", [(0.6, 1.0)])
    ok, why = d.allow(1.0, "g", [(5.0, 5.0)])
    assert not ok and "cooldown" in why
    ok, _ = d.allow(COOLDOWN_S + 0.1, "g", [(5.0, 5.0)])
    assert ok


def test_suppressions_always_carry_a_reason():
    d = FiringDiscipline()
    for args in ((0.0, "g", []),):
        ok, why = d.allow(*args)
        assert not ok and why


# --- the grid tracker's update path ------------------------------------------------------

def test_apply_update_patches_the_window_not_the_world():
    g = grid_with(width=10, height=10)
    g.apply_update(2, 3, 2, 2, [LETHAL, LETHAL, LETHAL, LETHAL])
    # Cell CENTRES, not boundaries: 0.10/0.05 is 1.999... in floats and truncates
    # into the neighbouring cell -- real callers pass arbitrary world coordinates,
    # never exact edges.
    assert g.at(0.125, 0.175) == LETHAL        # centre of patched cell (2,3)
    assert g.at(0.125, 0.325) == 0             # outside the patch, untouched


# --- the evidence freshness gate (transient-short's finding) ---------------------------

from sphero_rvr_core.refusal_promotion import (  # noqa: E402
    EVIDENCE_FRESHNESS_S,
    RecentReturns,
    freshness_verdict,
)


def test_stale_paint_cannot_be_promoted():
    """The rig's discovery, pinned: an obstacle that leaves in silence strands its
    paint; the delta survives but the RETURNS stop. A centroid with no return
    inside the freshness horizon is stale, whatever the costmap still says."""
    rr = RecentReturns()
    rr.add(0.0, 0.625, -0.025)                 # the obstacle, before it left
    now = EVIDENCE_FRESHNESS_S + 3.0
    fresh, stale = freshness_verdict(rr, now, [(0.625, -0.025)])
    assert fresh == [] and stale == [(0.625, -0.025)]


def test_a_live_obstacle_keeps_its_evidence_fresh():
    rr = RecentReturns()
    for i in range(40):
        rr.add(i * 0.15, 0.62 + 0.01 * (i % 3), -0.02)
    fresh, stale = freshness_verdict(rr, 6.0, [(0.625, -0.025)])
    assert stale == [] and fresh


def test_one_live_cluster_cannot_launder_a_stale_one():
    """Per-centroid gating: a firing with one fresh and one stale cluster promotes
    ONLY the fresh one -- a live obstacle nearby must not resurrect a departed
    one's paint."""
    rr = RecentReturns()
    rr.add(9.5, 0.6, 0.0)                       # fresh cluster A
    rr.add(0.0, 2.0, 2.0)                       # cluster B's returns long gone
    fresh, stale = freshness_verdict(rr, 10.0, [(0.6, 0.0), (2.0, 2.0)])
    assert fresh == [(0.6, 0.0)]
    assert stale == [(2.0, 2.0)]


def test_freshness_radius_is_the_merge_radius():
    """A return 20 cm away vouches for nothing at this centroid."""
    rr = RecentReturns()
    rr.add(9.9, 0.85, 0.0)
    fresh, stale = freshness_verdict(rr, 10.0, [(0.625, 0.0)])
    assert fresh == []


def test_the_freshness_horizon_re_derives_from_the_sensor():
    """F admits the flickeriest MEASURED real obstacle (the 15% thin-target class
    at the rate-band floor: one return per ~1.03 s) with ~3x margin, and expires a
    departed obstacle in seconds. Change the sensor constants and this re-derives
    or fails here, not in a field mission."""
    from sphero_rvr_core.refusal_promotion import (
        EVIDENCE_FRESHNESS_S,
        FLICKER_DETECTION_FRACTION,
        TOF_RATE_FLOOR_HZ,
    )

    flicker_period_s = 1.0 / (TOF_RATE_FLOOR_HZ * FLICKER_DETECTION_FRACTION)
    assert EVIDENCE_FRESHNESS_S >= 2.5 * flicker_period_s, (
        "F no longer covers the flickeriest measured real obstacle"
    )
    assert EVIDENCE_FRESHNESS_S <= 6.0, (
        "F this long keeps departed obstacles promotable for longer than any "
        "measured evidence supports"
    )
