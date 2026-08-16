# Replay exhibit: what the stock costmap held at the moments the bespoke stack froze

**Mission 2's recorded `/scan` and `/tf` replayed into a live Nav2 local costmap on the
Pi, chassis off. 6,638 samples over 445 s of mission time.**
Probe: `diagnostics/replay_stock_costmap_probe.py`. Data:
`03_validation/gauntlet_2026-08-16_mission2_diagnostics/replay_stock_costmap_probe.csv`.

## Claim boundary — read first

**PROVEN:** what a stock local costmap *contains*, given the mission's own recorded sensor
inputs, at the instants the bespoke stack declared a freeze. That is what stock's
collision-checked recoveries would have had to reason about.

**NOT PROVEN, and not claimed anywhere below:** what the mission would have *done*. A
replay is open-loop. The instant stock chose differently the robot would have moved
differently and seen different scans, so nothing here forecasts an outcome. **Anyone
reading these numbers as "stock would have escaped" has over-read them.**

**Coverage gap, stated rather than buried:** the probe's first sample is at mission time
`1786918659.3`, so **freezes 1 and 2 (17:15:47 and 17:16:10) are NOT covered** — the
costmap had no TF until the bag supplied it, and the probe only records once both exist.
Five of seven freezes are covered. The count of covered specimens is five, not seven.

## The specimens

| freeze @ (x,y) | robot's own cell | blocked? | lethal | inscribed | free |
|---|---|---|---|---|---|
| (−0.85, −0.73) | **253 (inscribed)** | **YES** | 344 | 1656 | 1153 |
| (−0.85, −0.71) | **253 (inscribed)** | **YES** | 354 | 1670 | 1111 |
| (−1.14, −0.36) | **253 (inscribed)** | **YES** | 370 | 1690 | 1063 |
| (−1.52, 0.62) | **0 (free)** | no | 184 | 916 | 2202 |
| (−1.25, 0.75) | **0 (free)** | no | 196 | 967 | 2152 |

Window: 3.0 × 3.0 m, 60 × 60 cells at 0.05 m. Across the whole replay, the robot's own
cell was at or above inscribed in **310 of 6,638 samples (4.7 %)** — the condition is
real but *local*, concentrated in one corner rather than general.

## What this says, in two halves

**1. Three of five freezes were in genuinely tight space, and OUR MARKS WERE NOT THE ONLY
REASON.** At the chair-leg corner the stock costmap — built from lidar and inflation
alone, with none of our freeze marks — **also** put the robot's own cell at inscribed
cost. D43's `START POSE BLOCKED` fired truthfully in mission 1, and this says the same
geometry would have confronted a stock costmap. The reckoning's framing that our
permanent marks *created* the prison needs qualifying: at these three positions the robot
was wedged, and a decaying-memory costmap would have seen it too.

The difference that remains real is **persistence**: our marks never decayed, so the
prison outlived the obstacle. A stock obstacle layer clears on raytrace. This replay
cannot show that difference — it would need the robot to move away and come back, which
open-loop replay cannot produce.

**2. Two of five freezes happened where the stock costmap saw OPEN FLOOR.** At
(−1.52, 0.62) and (−1.25, 0.75) the robot's own cell was free, with ~2,200 free cells in
the window and only ~190 lethal. The bespoke stack declared *"an obstacle no sensor on
this robot can see"* at positions where a stock local costmap held free space in every
direction. **Those are the freezes where a collision-checked recovery would have had
somewhere to go** — and where, in the real mission, Nav2's Spin/BackUp refused in 2 ms
because no local costmap existed to check against.

## What it does not settle

* Whether stock's recoveries would have **succeeded** — outcome, not decision.
* Whether the inflation is right. **1,656 of 3,600 cells (46 %) inscribed** in the
  cluttered corner is a lot of window to lose; that is `robot_radius` 0.145 plus the
  inflation layer doing what it is configured to do, and whether that configuration suits
  a 0.25 m-wide robot in a chair-legged room is its own question, unasked here.
* Anything about the touch layer. It had no marks to hold during replay, so this exhibit
  exercises `scan_layer` and `inflation_layer` only.

## Method notes worth keeping

* Alignment needed no signature matching: the bag supplies `/clock`, the probe runs on
  `use_sim_time`, so its stamps *are* mission time and line up with the launch log's
  epochs directly.
* The static `odom→base_link` MUST be off in replay (`static_odom:=false`). Publishing it
  on top of the bag's own transform is two answers to "where is the robot".
* The lidar must be off (`start_lidar:=false`), or a live scanner publishes `/scan` over
  the recording and the costmap silently mixes a recording with the room it is sitting in.
