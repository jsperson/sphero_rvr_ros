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
6. `/collision_stop/state` fields sanity: after bringup, one `ros2 topic echo
   --once` must show `pivot_veto=`, `cam_cloud_age=`, `output_angular_published=`
   (proves the deployed supervisor is ≥ 935f95d and the recorder will capture the
   D21/D22 instrumentation).

## Bringup order (separate terminals / tmux panes on the Pi)

```bash
# Pane 1 — camera first; separate launch, SURVIVES motion-stack restarts:
ros2 launch sphero_rvr_driver camera.launch.py

# Pane 2 — recorder BEFORE anything motor-capable; NOT installed by setup.py,
# run it from the source tree. 1800 s ceiling, explicit outfile:
cd ~/ros2_ws/src/sphero_rvr_ros/diagnostics
python3 run_recorder.py 1800 ~/run_$(date +%Y%m%d_%H%M%S).csv

# Pane 3 — console log captured to disk, then the stack. ALL FIVE args are
# required; the guide's bare command yields an inert stack that never moves:
ros2 launch sphero_rvr_driver explore.launch.py \
    start_motion_stack:=true \
    start_explore:=true \
    use_coverage_explorer:=true \
    use_decisive_controller:=true \
    start_low_obstacle:=true \
    2>&1 | tee ~/launch_$(date +%Y%m%d_%H%M%S).log
```

Open decision (flag at run time): `enable_imu_fusion:=true` is hardware-validated
and visibly straightened driving-with-turns; include it unless the run is meant to
reproduce a wheel-odom-only baseline.

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

Only then: Ctrl-C the launch. Remember it ignores SIGINT and children survive —
kill children by explicit PID (`pgrep` with a bracketed pattern, never a bare
`pgrep -f` that matches your own SSH command line), then `ros2 service call
/stop_motor` for the lidar, then verify `/scan` silent and `/dev/ttyUSB0` free.

## Directed gap test (D5, after the mission)

With the stack still up and the mission over (or explorer stopped), send one
`NavigateToPose` goal through the table/couch gap, then one back. Watch
`/collision_stop/state` in a spare pane: the 2026-08-03 result was that a clean
crossing stays CLEAR throughout — SLOW/STOPPED chatter in the gap is itself a
finding.

## Post-run, same sitting

- Grep the launch log: `"boxed in"` count (D1), abort/suppression lines (D4),
  mission outcome line (D6).
- Recorder CSV: every close approach shows SLOW/STOPPED before minimum range
  (D2); `cam_cloud_age` max over the drive (D22); `pivot_veto` events if any
  pivot was refused near a low obstacle.
- Update the register the same night — closed rows cite the artifact filenames.
