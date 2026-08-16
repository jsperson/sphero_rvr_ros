# RUN CARD — §3a: does the stock middle drive this robot?

**One hour of Scott's time. Everything else is already done.** This is the experiment the
whole architecture argument has been waiting on since `docs/navigation_reckoning.md` was
written, and Option A was ruled on 2026-08-16.

**Status: NOT FLOWN. Needs Scott, a staged rover, and a chassis.**

---

## What is already true, so the hour is spent on wheels and nothing else

| already proven | where |
|---|---|
| stock middle comes up on the Pi: controller_server, local_costmap, planner, behaviors, all **active** | `docs/stock_middle_stationary_bringup_2026-08-16.md` |
| local costmap **populates from real floor-level lidar** (579 lethal cells in one update) | same |
| behavior_server **subscribed to a costmap publisher that exists** — D36's refusal cause structurally dead | same |
| planner produces a path over a real SLAM map in **1 ms** | same |
| every angular constant **derived from the measured curve**, not guessed | `config/lean_nav2_stock.yaml`, `src/sphero_rvr_core/pivot_curve.py` |
| the driver honours a commanded pivot rate instead of discarding it | `b273d51`, `94bc22c` |

**Nothing in the stock middle is unverified except its behaviour against a moving robot.**

## The binary and the branch

Branch `prototype/stock-middle`. Confirm the Pi's SHA before anything arms — the SHA, not
the narration.

## PRE-FLIGHT EXAMINATIONS — do these before the chassis goes on

### P1. The 46 % inscribed window (MUST, and it is a real question)

The replay exhibit measured **1,656 of 3,600 local-costmap cells (46 %) at inscribed cost**
in the chair-leg corner. That is `robot_radius` 0.145 plus the inflation layer doing
exactly what it is configured to do — but half a window lost to inflation in a
chair-legged room could close gaps the robot can physically drive.

**The check, and it is concrete:** replay mission 2 into the stock costmap
(`bringup_stationary_test.launch.py start_lidar:=false static_odom:=false
use_sim_time:=true` + `ros2 bag play … --clock`) and, **at the door-gap transit** — the
known-good specimen, Scott watched the rover drive through it — ask the planner for a path
through that gap with `ComputePathToPose`.

* **Path exists** → 46 % is simply what a chair-legged corner looks like. Note it, move on.
* **No path** → stock's inflation closes a gap the robot **demonstrably drove**.
  `inflation_radius` and `cost_scaling_factor` get examined **before wheels**, because
  flying an over-inflated config would reproduce "stock cannot navigate this room" for the
  same reason as last time: a config value, not an architecture.

### P2. `slam_toolbox` lifecycle (MUST)

**`slam_toolbox` does not self-activate.** Launched without autostart it sits
`unconfigured`, publishing nothing and logging nothing after its banner — and it appears
in `ros2 node list` looking exactly like a healthy node. Either launch it with autostart
or gate on `ros2 lifecycle get /slam_toolbox` returning `active` before arming. A node
that looks healthy while doing nothing belongs to the tools-that-lie family.

### P3. Install manifest (MUST)

`setup.py` lists launch and config files explicitly. `config/lean_nav2_stock.yaml` sat on
this branch for a day **without ever being installed** — real, reviewed, and unreachable
by `get_package_share_directory`. There is now a guard test
(`tests/test_install_manifest_is_complete.py`); run the suite before flying and trust it
rather than assuming a file in git is a file on the robot.

## The experiment

1. Stage the rover on open floor, ≥ 0.5 m clear, chassis on, battery ≥ 25 %.
2. Bring up the stock middle with the driver and the reflex supervisor beneath it
   (the floor we keep — it has never failed).
3. Send **one** `navigate_to_pose` goal to open floor 1–2 m ahead.
4. Watch it drive.

## What each outcome means

* **It drives cleanly** → the architecture question is largely answered, Phase 3 is mostly
  deletion, and three weeks of bespoke middleware are explained by a config value in the
  impossible gap.
* **It grinds or fails to rotate** → the curve-derived constants are wrong or something
  below the driver is refusing. **Instrument first, do not tune**: `/diagnostics` is now
  recorded, so `motor_transport_write_count` and `motor_stall` answer "did the driver
  write, did the firmware stall" without inference.
* **It drives but wanders / overshoots** → RPP tuning, which is ordinary work on a
  component thousands of robots use, not an architecture problem.

## KILL CRITERIA — physical, not aesthetic

1. **Any sustained in-place angular command that the drivetrain cannot execute is an
   immediate stop.** This is now *testable rather than judged*: the curve says the
   producible band is **3.55–5.83 rad/s** and that duties below `pivot_min_duty` are the
   measured dead zone and bimodal walk band. Assert from the bag: no sustained
   `/cmd_vel_motor` with zero linear and `|angular|` below 3.55 that reaches the wheels as
   a sub-floor duty.
2. **Contact is a data point, not an abort** — reverse out, mark, continue. But **repeated
   re-entry into the same obstacle is an abort**: 2026-08-16 mission 2 showed the escape
   ladder driving forward through a mark it had planted 11 s earlier (D49).
3. **Any smell of hot motor, or grinding that does not resolve into motion within a
   burst** — power switch. Scott's hand, as always.

## Teardown

`/stop_motor` **before** killing the lidar node — killing the node does not stop the
motor. Then SIGINT by explicit PID. **Never `pkill -f` with a pattern that appears in your
own command line**; it matches the SSH session and kills the shell mid-sequence. That cost
a retry on 2026-08-16.

## After

Archive per vault protocol with `/diagnostics` in the bag. The comparison against the
bespoke stack is mission 2 (`03_validation/gauntlet_2026-08-16_mission2_diagnostics/`):
13 goals, 5 succeeded, 6 aborted after recovery, **0 without recovery**, 7 freezes,
3,686 covered cells.

**And the A/B is only fair if this is the honest comparison it looks like.** Stock's old
`rotate_to_heading_angular_vel` of 0.4 was never executable by this drivetrain — the
grinding that motivated the fork is what an unproducible rate looks like. If stock now
drives well, the finding is that **stock was under-tested, not unsuitable**.
