# Coverage + frontier explorer

An exploration driver that finishes a room only when the rover has both **seen**
every reachable spot *and* **driven within a coverage radius** of it — not just
glanced at it from across the room.

## Why (vs explore_lite)

`explore_lite` is **frontier** exploration: it drives to the boundary of the
unknown until everything reachable has been *seen* by the lidar, then stops. A
cell 4 m away with a clear sightline is "done" the instant a ray touches it — the
rover may never go near it. For **semantic inspection with a camera**, that isn't
enough: the camera needs to be *close* (≈0.75 m) to read a scene.

This node runs a **coverage + frontier** mission: keep going until no reachable
free cell is either unseen (a frontier) **or** farther than `coverage_radius_m`
from where the rover has driven.

## How it works

- Subscribes to `/map` (SLAM) and tracks the robot pose via TF.
- **Stamps coverage** along the actual path — every cycle it marks all free cells
  within `coverage_radius_m` of the live pose as covered (world-grid keyed, so a
  shifting SLAM origin doesn't invalidate it).
- Each cycle picks the **nearest reachable target** — a free cell that is uncovered
  OR (if `include_frontiers`) a frontier — via a flood over free space from the
  robot (walled-off cells are never chosen; noise clusters below
  `min_cluster_cells` are skipped) and sends it as a `NavigateToPose` goal.
- If a goal is unreachable / aborts, it **blacklists** a small disk around it and
  moves on. If the goal's cell gets covered *en route*, it cancels and reselects.
- **Mission complete** when no reachable target remains.

It reuses the whole nav stack (planner + decisive controller + collision brake) —
it just replaces explore_lite's frontier-picking with coverage+frontier-picking.

## Pieces

- `sphero_rvr_core/coverage_exploration.py` — pure, ROS-free core:
  `stamp_coverage`, `is_frontier`, `select_next_goal`. Unit-tested
  (`tests/test_coverage_exploration.py`).
- `sphero_rvr_driver/coverage_explorer_node.py` — the ROS node (NavigateToPose
  client, coverage/ blacklist state). Not lifecycle-managed.

## Enable it (opt-in; explore_lite is the default)

```bash
ros2 launch sphero_rvr_driver explore.launch.py start_motion_stack:=true \
    start_explore:=true enable_imu_fusion:=true use_decisive_controller:=true \
    use_coverage_explorer:=true
```

With `use_coverage_explorer:=true` the launch runs `coverage_explorer` instead of
`explore_lite` (and drops the `/explore/resume` kick — coverage never quits on an
empty frontier search).

## Tunables (`config/coverage_explorer.yaml`)

`coverage_radius_m` 0.75, `min_cluster_cells` 5, `include_frontiers` true,
`free_threshold` 0 (strict trinary map), `cycle_period_s` 1.0,
`blacklist_radius_m` 0.3.

## Status

Built, unit-tested (core), build-validated. **UNTESTED on hardware.** Watch that it
covers the room and stops cleanly, and that near-wall targets it can't approach get
blacklisted rather than looping.
