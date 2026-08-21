# Belief-basin geometry — flight 4, 2026-08-20 (bag_20260820_161932, on the Pi)

Four rounds of analysis (scripts here, run on the Mac against a copy of the
bag via mcap + mcap-ros2-support; frame math: map->odom ∘ odom->base_link ∘
base_link->laser from the bag's own /tf, laser yaw 179.0° confirmed).
Transcript anchors: rover stood at (0.866, −0.009) at 16:42:21 (tool 7 look);
failed 4× to return east from (0.209, 0.121), 16:44:27–16:49:21.

## Round 1 — the poses are not lethal
Cost at the former standing cell: 0 (pre) → 207 (mid-window). Stuck pose: 202.
Both are inflation gradient, not lethal. The basin is a corridor phenomenon.

## Round 2 — the corridor timeline
Connectivity stuck→standing through cells <253, per global costmap_raw grid:
- pre-window: 0/48 grids blocked
- window: OPEN t+0.3..221.3 s, CLOSED t+222.8..293.8 s (71 s)
The ABORTED gotos fall in the open phase (controller refusing a high-cost
corridor); the "planner found no path" endings in the closed phase.
The ≥253 ring within 1 m of the standing pose holds real furniture: 24.5% of
94,930 window returns land there. The ROOM is cluttered; see round 3 for the door.

## Round 3 — the door is imaginary
Diff of last-open vs closed grid: 447 cells newly ≥253 (1.12 m²), nearest cell
0.12 m from the stuck rover, 0.56 m from the standing pose.
Lidar truth over the closed period: **0/447 door cells supported by any of
23,027 live returns, at ±1-cell tolerance. 100% belief, 0% physics.**

## Round 4 — the painter, named
Door composition: 23 new LETHAL(254) seeds + 424 INSCRIBED(253) inflation
(0.30 m radius = ×19 amplification). Seed coverage: contact_marks 23/23,
ToF 0/23. /contact_marks/promote events at t+37.4, t+216.9, and **t+222.2 s —
the corridor closes at t+222.8 s.** A refusal promotion painted the door shut;
mark mission-permanence (the feature) keeps it shut against all sensing.

Feeds: docs/design_supervisor_escape_primitive_2026-08-20.md (the D52/D60
daylight round). Bag stays on the Pi (285 MB); this dir holds the method and
the numbers.
