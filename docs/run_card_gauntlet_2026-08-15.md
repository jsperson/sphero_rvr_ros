# RUN CARD — gauntlet restart on the contact batch (hold-on-vanish + measured footprint)

**One page to work from during the run.** The full protocol is
`docs/chassis_run_protocol.md`; this card is what changed, what to watch, and what to do
when it goes wrong. If this card and the protocol disagree, the protocol wins on
procedure and this card wins on what is new.

**Binary:** `640e440`. **Gauntlet counter: 0 of 3.** This is the restart the contact
batch earned.

---

## 0. What is different about this run, in four lines

1. **The brake no longer reads silence as clearance (D39).** When a tracked obstacle's
   returns vanish inside the sensor's structurally invisible band, the brake HOLDS its
   last belief instead of releasing. This is the fix for the 2026-08-15 rangefinder
   contact, where the brake worked, the evidence disappeared, and the rover drove
   90 mm back into a table leg at full commanded speed.
2. **The footprint is a measurement for the first time.** Scott's tape, referenced to
   base_link: front **0.0965** (to the rangefinder's leading edge), rear **0.1145**,
   left **0.098**, right **0.106**. Was 0.11 / 0.16 / 0.10 / 0.10. **The rover is
   smaller than the software thought, especially behind.**
3. **Every gate derived from the footprint moved with it.** The 08-14b wedge that
   defined D40 had 28 lidar returns inside the *declared* footprint; with measured
   extents it has **zero**. Refusals that were correct-by-construction there should
   not recur in that form.
4. **2c (the execute stage) is NOT aboard.** It is built and proven on the bench but
   deliberately unwired — see §5. The rover's stuck behaviour is unchanged from the
   last flight.

---

## 1. Preflight — one command now

```bash
# on the Pi
cd ~/ros2_ws/src/sphero_rvr_ros && git pull --ff-only && git rev-parse --short HEAD
#   must print 640e440 -- TRUST THE SHA, NOT THE PULL'S OUTPUT
cd ~/ros2_ws && colcon build --packages-select sphero_rvr_driver     # ~3 s
#   there is only ONE colcon package; sphero_rvr_core ships inside it and colcon has
#   never heard of it. Naming it prints "ignoring unknown package" and builds fine.

python3 ~/ros2_ws/src/sphero_rvr_ros/scripts/preflight_pi.py   # ~25 s, does the rest
```

The preflight covers the installed-tree verify, clock sync, an empty ROS graph and the
chassis. It diagnosed a dead chassis in 25 s where the manual sequence took five
minutes. Read its remedies rather than improvising.

**Two things it will say tonight, both expected:**
* `installed_tree_matches` now reports `... 2 package dir(s) are SYMLINKS to source
  (cannot be stale, not evidence)`. That wording is deliberate — `--symlink-install`
  makes the deployed files the same inodes as the source, so they cannot be stale and
  the check cannot fail for them. It is a PASS that is honest about what it did not
  prove. Before tonight this gate returned **NOT CLEARED on a perfectly good build**.
* `chassis_alive` FAILs until Scott powers the chassis. That is the gate working.

**Running the test suite on the Pi? Expect 10 red that are not real.** With
`install/setup.bash` sourced, ROS's environment breaks pytest's `caplog` capture, so
every test asserting on captured log text fails — in `test_dispatcher.py`,
`test_driver_safety.py`, `test_avoidance_wiring.py`. Verified pre-existing: the same 10
fail identically at the pre-batch SHA `85ab0be`. With `PYTHONPATH=src` instead, the Pi
runs **927 passed, 8 skipped** — identical to the Mac. Do not debug these before a run.

**One extra check this run, because the whole batch turns on it:**

```bash
ros2 param get /collision_stop low_obstacle_hold_on_vanish_enable   # must be true
ros2 param get /collision_stop footprint_rear_m                     # must be 0.1145
```

A config file is a claim; the robot's state line is the robot.

**Sun check** (rule B holds brake authority on a sensor never measured in direct sun —
wait it out or label the run sun-contaminated *before* starting).
**Battery ≥ 25%, Scott within reach of the power switch, floor not a bench.**

---

## 2. Bringup and liftoff

Per the protocol's bringup order. The stack comes up **disarmed** (D29); liftoff is the
explicit service call, and the gates get read *before* it.

New line to read on `/collision_stop/state` before arming:

```bash
ros2 topic echo /collision_stop/state --once --full-length | tr ' ' '\n' | grep cam_hold
#   want: cam_hold_active=false  cam_hold_reason=clear
```

`cam_hold_reason=held_no_pose` before the rover has moved means TF is not yet flowing —
that is the hold failing SAFE (it holds when it cannot place a belief), but it must
clear once odom is up. Arming with it stuck is arming into a permanent forward clamp.

---

## 3. Watch list — what this run is for

Beats **every 60 s on a timer**, including "driving, all nominal". A silent stall
produces no event, and event-driven reporting has gone quiet exactly when it was most
needed, twice.

| watch | why it matters this run | what to note |
|---|---|---|
| **`cam_hold_active=true` episodes** | the fix's first flight | wall-clock time, what was in front, and **how long the hold lasted** |
| **A hold outliving its retirement by >10 s** | the over-caution tripwire | this is the agreed escalation signal — note it and report it; the rover is safe but stuck-ish |
| **`cam_hold_reason=held_no_pose`** | TF gap while driving | should be rare; a run full of these means the hold is running blind on pose |
| **Contact of any kind** | **Scott's standing bar** | contact with anything the rangefinder detected *or could have detected* is a DEFECT. Cut chassis power, note the time, **leave every process running**, retrieve artifacts before teardown |
| **Reverse behaviour** | rear extent shrank 0.16 → 0.1145 | does it now reverse where it used to refuse? That is the intended direction |
| **Phantom brakes** | rule B still young | any brake with nothing visibly there — note the surface, especially glass, gloss, dark or shiny floors |
| **Wedged with open floor** | see §5 | note the pose and the open bearings; this is expected-possible this run |
| **Abort split** | in the end report | `aborted_after_recovery` vs `aborted_without_recovery` |

**Recording:** run the recorder before anything motor-capable, per protocol. The CSV
now carries `cam_hold_active` and `cam_hold_reason` columns — **an episode without them
cannot be reconstructed**, so confirm the header before liftoff. Also
`ros2 bag record /scan /tof/points /tof/obstacles /tof/state /tf /tf_static /odom` if
cheap; `/odom` is what makes a hold episode replayable.

---

## 4. If the brake holds and will not let go

The hold retires two ways, and **both are motions the rover can be given**: reverse far
enough that the belief becomes visible again and a look clears it, or **rotate** so the
belief leaves the commanded corridor. The second is the one the arbiter is most willing
to grant.

So if the rover sits with `cam_hold_active=true` and refuses to advance: **that is the
fix working**, not a hang. Note it, let the recovery run, and if it persists past ~10 s
past where it should have retired, that is the tripwire above. **Do not disable the hold
mid-run to "unstick" it** — a run with the fix switched off halfway is a run that
measures nothing.

---

## 5. Known contingency — a wedged-with-open-floor ending is possible, and it is not a surprise

**2c (execute stage) is not wired, and D42's mark inflation is unfixed (batch B).** So a
run can still end wedged on measurably open floor, mission-2 style. **If Scott's field
ruling voids such a run, that is his call and his precedent — and it is not a failed
evening: it is the promotion evidence that makes 2c + batch B the next morning's work,
and the gauntlet re-flies on them.** Decided in advance so nobody relitigates it at 9 pm.

---

## 6. After

Artifacts **before** any Ctrl-C, per protocol: mission report (latched), map, recorder
CSV, launch log, bag if recorded. Then teardown: **lidar motor by service**
(`/stop_motor`, confirm `/scan` silent), processes by explicit PID — `pkill -f` matches
this session's own SSH command line and has been demonstrated to do so — then
`ros2 node list` empty and `ros2 daemon stop`.

Vault: `03_validation/gauntlet_2026-08-15_mission1/` with a README naming the binary
(`640e440`), what was new (hold-on-vanish live, measured footprint) and the outcome.

**Scoring:** mission 1 of 3. Read `stall_survival_ladder.md` §7.1's no-count rule before
recording the count.

**One comparison is owed and has a trap in it:** `cam_nearest` keeps its name and its
meaning ("what the brake acted on"), but that value can now be a HELD BELIEF rather than
a live sighting. Any longitudinal comparison of `cam_nearest` against earlier runs must
filter on `cam_hold_active=false` to compare like with like.
