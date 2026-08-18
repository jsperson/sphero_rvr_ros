# Chassis Run Protocol — the recorded run that settles D1/D2/D4/D6 (+D5)

Chassis runs are the scarce resource: 2026-08-09 spent ~5 of them and closed
nothing, because each failure was unrecorded or confounded. This protocol exists so
the NEXT run is diagnosable no matter how it ends. Every step is checkable before
the chassis powers on; the close criteria are written BEFORE the run so nothing get
closed by vibes afterward.

## What this run settles (and the observation that settles it)

| Defect | Closes on this observation | Not closed if |
|---|---|---|
| D1 (boxed-in latch) | ≥5 consecutive goals complete with zero "boxed in" aborts in the launch log | run ends earlier for any reason |
| D2 (contact) | mission drives among obstacles the lidar sees and the rover stops short of everything — zero contact, confirmed by watching + recorder CSV showing STOPPED/SLOW before every close approach | any contact, even glancing |
| D4 (retry forever) | a goal fails and the NEXT goal goes somewhere else (log shows suppression, different cell) | no goal happens to fail (vacuous — leave open) |
| D6 (no completed mission ever) | mission ends `COMPLETE` with report artifact + saved map in `~/.ros/missions/` | any other outcome — but an HONEST incomplete outcome (`INCOMPLETE_NO_PLANNABLE_TARGETS` etc.) with artifacts intact is still a diagnosable run, not a wasted one |
| D5 (narrow gap, rides along) | after the mission (or during it) the rover crosses the table/couch gap on a directed `NavigateToPose` goal, both directions | gap not attempted |

D22 gets a free data point: the recorder CSV now carries `cam_cloud_age` via
`/collision_stop/state` — check post-run that it stayed under `camera_max_age`
(0.6 s) for the whole drive.

## Preconditions (all checkable cold)

1. **D16 bench probe has run.** The nav2 `behavior_server` Spin gate reads a local
   costmap that decisive mode removes; until measured, a refused recovery cannot be
   attributed. If the probe shows Spin refused in decisive mode, expect unstick
   turns to fall back to straight BackUp and do not read that as a regression.
2. **Pi at repo HEAD and rebuilt** (`git -C ~/ros2_ws/src/sphero_rvr_ros log
   --oneline -1` matches origin/main; `colcon build` run since).
3. **Rover on the FLOOR** (a bench run scans at waist height and proves nothing),
   in a **static room** (no door that opens mid-run — that confound wasted the one
   prior clean attempt), with **≥0.5 m clearance all round the start pose**
   (<0.30 m puts the robot's own cell at inscribed cost: nothing plans, no recovery
   works).
4. **Battery charged** and the plan is to tear down PROMPTLY on a stall — an idle
   stack (lidar spinning, nodes up) drained the battery flat on 2026-08-03.
5. **Scott attended, within reach of the power switch** for the entire run.
6. **Clock sync.** `ssh sphero-pi-2 timedatectl status` must report *System clock
   synchronized: yes* BEFORE the stack goes up. A freshly booted Pi has been seen 72
   minutes behind the workstation until NTP caught up (2026-08-10), and every
   attribution in this protocol depends on pairing Scott's wall-clock notes against
   log timestamps. Starting inside that window silently invalidates the whole
   exercise while every artifact still looks plausible.
7. `/collision_stop/state` fields sanity: after bringup, one `ros2 topic echo
   --once` must show `pivot_veto=`, `cam_cloud_age=`, `output_angular_published=`
   (proves the deployed supervisor is ≥ 935f95d and the recorder will capture the
   D21/D22 instrumentation).
8. **`ros2 node list` is EMPTY before bringup.** Added 2026-08-11 after finding
   seven `/fake_world` nodes and a `/coverage_explorer` still alive on the Pi hours
   after a test run. The mission harness hosts FAKE `navigate_to_pose`,
   `compute_path_to_pose`, `backup` and `spin` action servers, and the Pi's pytest
   process routinely outlives its own suite by many minutes (67 s of tests, process
   alive 26 min later, zero CPU — an asyncio teardown hang, `Event loop is closed`).
   A real bt_navigator coming up alongside a fake action server of the same name is
   a bizarre failure that would cost a chassis run to diagnose. Kill leftovers **by
   explicit PID** — see the teardown section on why `pkill -f` is not safe here —
   then `ros2 daemon stop` so the listing is not a stale cache.

   Two corollaries, both bought on 2026-08-18 by an agent that hit every hazard in this
   paragraph without having read it:

   * **Gate on the SUMMARY LINE, never on process exit.** `python3 -m pytest` printed
     `10 failed, 1298 passed, 2 skipped ... in 79.71s` and was still resident twenty
     minutes later. (2026-08-18: first MAC sighting of the same hang — a 12 s suite,
     nothing through the pipe, SIGTERM'd at 10 min. `scripts/run_pytest_bounded.py`
     is the norm on BOTH machines now.) A waiter watching the process reports a 13-minute hang for an
     80-second suite, and three stacked waiters then ran three concurrent suites against
     one ROS graph. Wait for the summary text; kill the pid afterwards.
   * **Match teardown patterns on the EXECUTABLE, not the language.** A sweep over
     `python3` processes misses `tf2_ros static_transform_publisher`, which is C++ — two
     of them were left publishing `map->odom` after a teardown that reported itself
     clean. A stray global-frame TF publisher is two answers to "where is the robot",
     which is the seam class this project keeps paying for.

## Bringup order (separate terminals / tmux panes on the Pi)

```bash
# Pane 1 — DO NOT START THE CAMERA. Retired from bringup 2026-08-16 by Scott's
# charter: the camera is an INTELLIGENCE sensor (objects, scenes, faces), invoked
# on demand by Track 2's observe path — never part of default bringup, never in the
# safety stack, never in direct navigation.
#
# It is not a preference. On gauntlet mission 1 the camera plus the monocular
# detector burned ~66% of a CPU on a Pi that reached load 10.7 and starved the ToF
# to 5.4 Hz — below the rate its own staleness bound is derived from — to feed a
# topic the collision brake stopped reading. Shedding both took load to 3.1 and the
# ToF to 6.9 Hz, and only then was the run flyable.
#
# The monocular `low_obstacle` node no longer exists in the launch or as an entry
# point. `camera.launch.py` still exists and is started only when something needs
# eyes.

# Pane 2 — recorder BEFORE anything motor-capable; NOT installed by setup.py,
# run it from the source tree. 1800 s ceiling, explicit outfile:
cd ~/ros2_ws/src/sphero_rvr_ros/diagnostics
python3 run_recorder.py 1800 ~/run_$(date +%Y%m%d_%H%M%S).csv

# Pane 3 — console log captured to disk, then the stack. ALL FIVE args are
# required; the guide's bare command yields an inert stack that never moves.
# The stack now comes up DISARMED (mission_autostart defaults false, D29), so
# this command no longer commits the robot to moving:
ros2 launch sphero_rvr_driver explore.launch.py \
    start_motion_stack:=true \
    start_explore:=true \
    use_coverage_explorer:=true \
    use_decisive_controller:=true \
    start_low_obstacle:=true \
    2>&1 | tee ~/launch_$(date +%Y%m%d_%H%M%S).log

# Pane 3, AFTER the gates below pass — THIS is liftoff, not the launch above:
ros2 service call /coverage_explorer/mission/start std_srvs/srv/Trigger
# and to end the mission early (this is NOT an e-stop; the supervisor owns that):
ros2 service call /coverage_explorer/mission/stop std_srvs/srv/Trigger
```

**Gates, THEN go — and as of D29 that is real rather than aspirational.** Until
2026-08-10 the mission began the instant the explorer node started, so run 185048's
entire 53 s mission ran and died *during* the bringup gate checks: the first gate
was read after the mission report had already latched, and the operator spent over a
minute watching a stopped rover with no idea a mission had happened at all. Verify
every gate below against the live stack, and only then call `mission/start`.

**Beats go out on a TIMER, not on events.** One line every 60 s from liftoff to
teardown — position, cardinal heading, supervisor state and `reason`, whether output
is moving — *including* "driving, all nominal". Twice on 2026-08-10 an event-driven
policy went silent exactly when the operator most needed a line, because a silent
stall produces no event. That is the failure mode of reporting only what seems
noteworthy.

Open decision (flag at run time): `enable_imu_fusion:=true` is hardware-validated
and visibly straightened driving-with-turns; include it unless the run is meant to
reproduce a wheel-odom-only baseline.

## Direct sun — a SCHEDULING rule, and it now matters more than it did

**Missions avoid hard direct sun until the sun capture happens.** This is not a code
gate and there is nothing in the stack that enforces it; it rides on whoever schedules
the run.

It was already the rule. What changed on 2026-08-14 is the stake: **rule B now holds
brake authority**, pinned against an indoor wall under indoor light. A ToF's failure
mode in strong ambient IR is dropped or shortened returns, and rule B reads a shortened
return as an obstacle nearer than the lidar sees — which is the phantom direction. The
sensor has never been measured in sun, so the size of that effect is unknown rather than
small.

Until Scott's sun capture exists, treat hard direct sun as an unstaged condition: run in
it only deliberately, with the run labelled, and do not read a brake event in sun as
evidence about the room.

## One check that must happen on the FIRST Pi bringup after the frame batch

**Chassis off; the ToF needs only I2C.** Bring `tof_node` up alone and watch one cycle
of `/tof/state`. The line must contain `obstacle_consumers=<n>`.

Two reasons, and the second is the one that costs something if skipped:

1. That token replaced `consumers=none_stage_i`, a literal that had been false since the
   supervisor first subscribed. It is now a MEASURED count
   (`get_subscription_count()`), so it also serves as a pairing check: on a flight
   bringup it must be `>= 1`, and a `0` means the supervisor is not actually subscribed
   whatever its own config claims.
2. **It is the one line in that batch that no test on the dev machine can execute** —
   there is no `rclpy` on the Mac, so the call was reviewed but never run. It lives in
   the 1 Hz state timer, where an `AttributeError` would take the node down. A state
   line that prints the token proves the call is real; a node that dies a second after
   bringup is the other answer, and it is far better to get it here than mid-mission.

## During the run

- Watch; do not touch a healthy run. Note wall-clock times of anything odd — the
  recorder rows are timestamped from its start.
- **Stall >2 min with no goal progress and no recovery motion**: treat as ended.
  Retrieve artifacts (below), THEN tear down. Do not wait for the battery lesson
  to repeat.
- **Any contact**: cut chassis power, note the time, leave every process running —
  the recording around the contact instant is the evidence; retrieve artifacts
  before any teardown.

## Artifact retrieval — BEFORE any Ctrl-C of the launch

The mission report is a latched topic and the map is written by the explorer node
itself: **both die with the launch process.** Ctrl-C first = run written off.

```bash
# 1. Report (works even mid-mission for the wedged/blocked interim state):
ros2 topic echo /coverage_explorer/report --once

# 2. Map: written to ~/.ros/missions/ ONLY on mission self-termination.
#    If the mission did NOT self-terminate, save SLAM's map by hand FIRST:
ros2 run nav2_map_server map_saver_cli -f ~/manual_map_$(date +%H%M%S)

# 3. Recorder: Ctrl-C pane 2 (safe — it flushes on the way out).
# 4. Copy everything OFF the Pi before teardown:
scp sphero-pi-2:"~/run_*.csv ~/launch_*.log ~/.ros/missions/*" <local>
```

**AND ANY CHARACTERISATION DATA, not just mission runs.** Bench and sensor sessions
produce evidence that lives nowhere else — `~/tof_smoke/` (2026-08-13's rangefinder
characterisation) is the current example. It goes to the vault at teardown exactly
like a mission artifact, with its README, because the vault plus iCloud is the ONLY
backed-up home for evidence data: the provisioning repo deliberately excludes it
(`sphero-pi-provisioning`, RESTORE.md §4), and the Pi's SD card is not a backup.
Un-copied artifacts from an earlier run count too — check `ls ~/run_*.csv` against
what the vault already holds before killing anything.

**An artifact is ARCHIVED when it is in the vault with its hashes recorded — not when
it has been copied off the Pi.** A session scratchpad is a staging area, and staging
areas are swept. The 2026-08-12 gauntlet-reset runs (mission 1 = `112721`, D35's
evidence; mission 2 = `125305`, D36's close-criterion source) were pulled off the Pi
correctly and then sat in `/private/tmp/claude-501/.../scratchpad/` for two days while
the vault held nothing from that date — long enough that a later session searching
`03_validation/` concluded the runs had never been captured at all. They were recovered
on 2026-08-13 and are now at `03_validation/run_2026-08-12_gauntlet-reset_mission{1,2}/`
with per-file sha256 manifests. So: the "check what the vault already holds" sweep above
applies to **scratchpads too**, the copy is not done until `shasum -a 256` matches at
both ends, and the manifest goes in the set's README where a future session will find it
without having to trust a narration.

Only then: Ctrl-C the launch. Remember it ignores SIGINT and children survive.
**Kill by explicit PID — list them with `ps -eo pid,cmd`, then `kill <pid>` — and
EXCLUDE YOUR OWN SESSION from that list.** Filtering the listing is not optional:
`ps | grep <pattern>` matches the SSH wrapper carrying your command too, so feeding
that straight into `kill` drops your own connection mid-teardown. Add
`| grep -v tailscaled | grep -v "bin/bash"` (or check each PID) before killing. Do
not reach for `pkill -f <pattern>`: a bracketed pattern does NOT protect you here.
The bracket trick only defeats *grep's* self-match; `pkill -f` treats `[d]emo` as a
plain regex matching `demo`, and your own SSH command line (and the tailscaled
wrapper carrying it) contains the target string — including when the name appears
in an unrelated part of the same command, such as an `rm` in the cleanup half.
This killed the operator's session three times on 2026-08-10 alone, twice mid-teardown
with nodes still running — and once more on 2026-08-11, in a session that had read
this paragraph the same hour: `pkill -f "pytest tests/test_coverage"` matched the SSH
command line carrying it. Knowing about the trap is not protection; only filtering the
PID list is. Then `ros2 service call /stop_motor` for the lidar, and
verify `/scan` silent and `/dev/ttyUSB0` free.

## Directed gap test (D5, after the mission)

With the stack still up and the mission over (or explorer stopped), send one
`NavigateToPose` goal through the table/couch gap, then one back. Watch
`/collision_stop/state` in a spare pane: the 2026-08-03 result was that a clean
crossing stays CLEAR throughout — SLOW/STOPPED chatter in the gap is itself a
finding.

## Boxed-in attribution — pairing the human eye with the recording

The 2026-08-10 run ended with the rover stuck and the costmap insisting it was
boxed in, while Scott stood there looking at open floor. That disagreement could
not be settled afterwards, and the reason is worth stating precisely: **the rover
rotated between the observation and the measurement.** By the time the costmap was
dumped, "east" in Scott's cardinal sense and "the direction the robot is facing"
were no longer the same bearing, so neither reading could confirm or refute the
other. Nothing was wrong with either observation. They simply could not be
compared, because nothing recorded *when* and *facing where*.

Two habits fix that, and they need no tooling — the recorder already carries
`odom_yaw_deg`, `cam_cloud_age` and `pivot_veto` as of `4c397c7`.

**At mission start — Scott, once:** note which compass direction the rover's NOSE
points. Without it the recorded `odom_yaw_deg` cannot be converted to a cardinal
bearing at all: yaw is measured from the startup heading, so "yaw 22 deg" means
nothing until that heading has a compass value. One sentence, once, and every stall
in the run becomes convertible.

**During the run — Scott, on each audible or visible stall:** note the **wall-clock
time** (to the nearest ~5 s is plenty) and **one compass-style sentence** about what
you see, phrased as direction plus distance: *"11:04:30 — east about 2 m clear,
backpack close on the southwest, chair legs northwest."* Cardinal directions rather
than robot-relative ones ("its left") — the robot's left changes when it turns, and
that ambiguity is exactly what cost the last attribution. Note it even when the
stall resolves itself; a stall that clears is still evidence about the costmap.

**After the run — pair each note against the recording.** For each boxed-in event in
the launch log, take its timestamp, find the matching rows in the recorder CSV, and
read `odom_yaw_deg` there. That yaw converts Scott's cardinal observation into the
robot's frame, so his "east is clear" and the costmap's "blocked ahead" finally
describe the same physical direction and can agree or disagree meaningfully. Then
one of three things is true, and the run can say which:

- **They agree** (something really was where the costmap said): the boxed-in verdict
  was correct, and the question becomes why recovery could not escape it.
- **They disagree** (open floor marked lethal): that is a costmap-fidelity defect
  with a concrete bearing and timestamp attached — the strongest form of that
  finding this project has ever had, and far better than the standing suspicion.
- **The yaw shows the robot had turned** between the note and the reading: the
  comparison is void, exactly as on 2026-08-10 — but now it is *known* to be void
  rather than mistaken for a result.

Also read `cam_cloud_age` at those same rows. Both camera gates fail open on a stale
cloud, so a boxed-in event with a stale or empty `cam_cloud_age` was decided by lidar
and costmap alone, and the camera should not be credited or blamed for it.

## Post-run, same sitting

- Grep the launch log: `"boxed in"` count (D1), abort/suppression lines (D4),
  mission outcome line (D6).
- Recorder CSV: every close approach shows SLOW/STOPPED before minimum range
  (D2); `cam_cloud_age` max over the drive (D22); `pivot_veto` events if any
  pivot was refused near a low obstacle.
- Update the register the same night — closed rows cite the artifact filenames.

---

## Sensor GEOMETRY sessions — the constraint that must be in every capture

Applies to any bench sitting that fits a sensor's mount geometry (height, pitch,
zone/beam angles). Chassis off; this is not a run.

**ALWAYS capture a flat wall at TWO distances, in addition to whatever the session is
actually about.** Not optional and not a nice-to-have — it is the constraint that makes
the fit identifiable at all.

Why, from 2026-08-13 (docs/tof_navigation_design.md §9.10): mount height, pitch and
vertical zone pitch are strongly correlated in a fit against floor rows alone. Separate
fits over the *same* eight recorded medians produced heights from 0.109 m to 0.185 m at
comparable RMS — all of them "good fits", none of them identifiable. A flat wall breaks
the degeneracy because **a plane's row-to-row gradient depends on pitch and zone pitch
but not on mount height at all**. Solving the wall distance from one row and predicting
the rest landed within 6 mm.

Two distances, not one, for the same reason at a different level: at one distance,
competing models of what the sensor REPORTS can coincide. The level-mount session
answered the ToF's reporting convention correctly by luck, because a horizontal
boresight makes two of the three candidate models the same line.

Practical points, all learned the hard way:

* **Check monotonicity before trusting a wall segment.** The 2.13 m wall was discarded
  because its row medians ran 1.905 / 1.857 / 1.967 — not monotonic, therefore not one
  plane. Side clutter. A contaminated constraint is worse than no constraint.
* **Pre-register the competing models and what each predicts**, before capturing. That
  is what turned "it reports z" from a lucky guess into a measurement.
* **Watch the capture's line count GROW**, not merely exist — a capture died silently
  after 500 frames on 2026-08-13 while Scott held a pose in front of it.
* **Stop captures with the script's own `--stop`**, never a bare `kill` on a shell
  wrapper: `python3 diagnostics/tof_capture.py --stop <csv>`. Killing the wrapper leaves
  the Python child recording, which happened three times in one session.
