# RUN CARD — §3a: does the stock middle drive this robot?

**One hour of Scott's time. Everything else is already done.** This is the experiment the
whole architecture argument has been waiting on since `docs/navigation_reckoning.md` was
written, and Option A was ruled on 2026-08-16.

**Status: READY TO FLY as of 2026-08-18 (`3334be5`). Needs Scott, a staged rover, and a
chassis.** Everything below the wheels is measured; the four-clause acceptance is green in
the closed-loop rig; the Pi is on the branch and built. What is NOT proven is anything
that needs a real floor — see the 08-18 section below for what changed and what to watch.

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

**RESULT (answered 2026-08-22 — six days late, found unanswered by the project
review's field day): PATH EXISTS.** Instrument: the rig (map_server on mission 2's
own saved map + the deployed stock yaml) after the bag-replay form of this check
failed its own control (TF tolerance broke at 2× playback; a NO with no
demonstrated YES certifies nothing — the control caught it). Calibrated probe:
control pair PLAN EXISTS (0.40 m); then ComputePathToPose THREADS both driven
pinches — the 0.78 m corridor at (0.52, −0.11) both directions (14 poses,
0.83 m, straight through) and the 0.75 m corridor at (−1.40, 0.61) (15 poses).
Verdict per this card's own branches: the 46 % window is what a chair-legged
corner looks like; deployed inflation does NOT close the doorways this robot
drives. (Deployed-inflation geometry, same sitting: hard blocking floor is
2×inscribed ≈ 0.31 m at ANY inflation_radius; corridors 0.31–0.60 m are
costed-but-plannable with no legal goal cell INSIDE; detours win only where a
cheaper one exists — the 2026-08-22 field route-around and these threaded plans
are one cost model.)

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
   **THE GOAL MUST BE VERIFIED IN MAPPED FREE SPACE BEFORE SEND** — flown 2026-08-18:
   "1.5 m dead ahead" placed goal 1 physically inside the couch, in SLAM-unknown
   space, and the planner's correct refusal read as a navigation failure until the
   operator looked at the room. The operator's mental map and the furniture are the
   same thing the planner is asked about, so the map is consulted, not the mind's
   eye. `scripts/fly_stock_goal.py` enforces it (mapped free + cost 0 + dry-run
   plans, plus a lifecycle liveness precheck) — send goals through it, not by hand.
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

## WHAT CHANGED SINCE THIS CARD WAS WRITTEN (2026-08-18) — READ BEFORE FLYING

Three things that alter what you will see, all measured and all in the branch at
`3334be5`.

**1. The planner can see.** `lean_nav2_stock.yaml` had no `global_costmap` section, so
planner_server ran on nav2 defaults with an obstacle layer subscribed to nothing. In this
project's entire history the planner has planned against the SLAM map alone, blind to
every change since the map was drawn. It now has live `/scan` and `/contact_marks`. **This
changes planning everywhere, not just near marks** — it is the largest behavioural change
in this flight and the most likely source of surprise.

**2. Inflation went 0.16 → 0.30, and the rover will look different for it.** At 0.16 the
cost gradient was 8 mm wide (the deployed circumscribed radius is 0.1591), so the planner
had nothing to prefer clearance with. **WATCH:** RPP's regulated velocity scaling slows
near obstacles, so expect a slower rover near furniture, and in a tight corridor nearly
every cell is now costly, which shifts route preference. Expected, watched, not a mystery.
It cannot close a gap the rover could previously plan through — the blocking region comes
from the footprint, not from this value.

**3. Contact marking is live.** The rover now learns where it has been hit and the planner
routes around it. Rig-measured on an open map: mark planted, 0.321 m detour, goal reached,
0.075 m of clearance, no re-contact.

### The mat, and why "it routes around" is not the whole truth

Scott's mat is 9.5 mm against his ~19 mm crossable spec, and the rover met its edge on
2026-08-18. A stall there is a *true* contact — the detector requires stall plus
packets-written plus no-motion — but "stalled here" is not "impassable", and until
try-harder and revocation land the mark is permanent.

**In a 0.4–0.6 m corridor there is no elsewhere: one mark ends the route.** Measured in
the recorded room — the free span containing the rover's own path is 0.40–0.60 m through
that stretch and 1.05 m at its widest anywhere on it, against ~0.51 m sterilised by a
single mark. The planner does not route around; it reports `no valid path found` from the
start pose and the goal fails. It fails *honestly and immediately*, and the rover never
approaches the mark (closest approach 0.456 m, versus driving to 0.151 m from it before
this change) — which is the improvement. It is still a stalled mission.

**Expected is not acceptable: this ending is the field receipt that pulls try-harder
forward, per Scott's own rule.** Pre-naming an outcome must not anaesthetise us to it. If
the flight ends at the mat edge, that is the evidence, not a shrug.

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
