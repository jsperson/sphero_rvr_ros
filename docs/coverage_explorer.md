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
- Each cycle **proposes targets nearest-first** — free cells that are uncovered OR
  (if `include_frontiers`) frontiers — via a flood over free space from the robot,
  one per cluster, skipping noise clusters below `min_cluster_cells`.
- It then **asks the planner** (`ComputePathToPose`) about each proposal in turn and
  sends the first that plans as a `NavigateToPose` goal. If the goal's cell gets
  covered *en route*, it cancels and reselects.
- **Mission complete** only when there are no targets left at all. If targets remain
  but the planner refuses every one, it says so — *"targets wanted but NONE
  plannable"* — which is a different outcome and must not be read as success.

It reuses the whole nav stack (planner + RPP controller + collision brake).
(2026-08-21: explore_lite and the decisive controller are retired; this IS the
explorer, on the stock middle.)

## Reachability belongs to the planner

The core proposes; it does not decide what is drivable. It used to: it eroded the
map by an inscribed radius, flooded only over the eroded result, and treated that as
"reachable". That estimate disagreed with the real costmap in both directions, and
the blacklist (plus its radius, plus its TTL, plus a watchdog) existed to contain the
disagreement — a permanent, wide blacklist once reached 2390 cells, 73% of all free
space, ending a mission with 108 frontier cells still unexplored in an area that was
reachable the whole time.

Asking `ComputePathToPose` costs one query per rejected candidate, against the same
planner the goal would be handed to anyway, and it is right by construction. The only
failure memory left is a short TTL on a goal that *planned* and then would not drive
(the watchdog case) — planner refusals and goal aborts are simply re-asked next cycle.

## Pieces

- `sphero_rvr_core/coverage_exploration.py` — pure, ROS-free core:
  `stamp_coverage`, `is_frontier`, `candidate_goals`, `robot_start_blocked`.
  Unit-tested (`tests/test_coverage_exploration.py`).
- `sphero_rvr_driver/coverage_explorer_node.py` — the ROS node (NavigateToPose +
  ComputePathToPose clients, coverage state). Not lifecycle-managed.
- `diagnostics/plannability_check.py` — prints the whole candidate list with the
  planner's verdict on each. No motion.

## Enable it (opt-in)

```bash
ros2 launch sphero_rvr_driver explore.launch.py start_motion_stack:=true \
    start_explore:=true enable_imu_fusion:=true use_coverage_explorer:=true
```

With `use_coverage_explorer:=true` the launch runs `coverage_explorer` instead of
`explore_lite` (and drops the `/explore/resume` kick — coverage never quits on an
empty frontier search).

## Tunables (`config/coverage_explorer.yaml`)

`coverage_radius_m` 0.75, `min_cluster_cells` 5, `include_frontiers` true,
`free_threshold` 0 (strict trinary map), `cycle_period_s` 1.0,
`max_candidates` 12 (planner queries per selection), `plan_timeout_s` 2.0,
`stall_suppress_radius_m` 0.2 / `stall_suppress_ttl_s` 45.

## Status

Core unit-tested; planner gating **not yet validated on hardware**. Watch for: the
first goal each cycle being reached rather than churned, `planner-rejected=` in the
goal log staying small, and a completion that distinguishes "covered everything"
from "nothing plannable".
