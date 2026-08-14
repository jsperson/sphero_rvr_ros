# RUN CARD — gauntlet on the frame-fixed binary, + the English-instruction demo

**One page to work from during the run.** The full protocol is
`docs/chassis_run_protocol.md`; this card is what changed, what to watch, and the
script for the demo segment. If this card and the protocol disagree, the protocol wins
on procedure and this card wins on what is new.

**Binary:** `a667de2`. Frame fix live, rule A **and rule B** live, Track 2 aboard.
**Gauntlet counter: 0 of 3.** This is the restart the frame fix earned.

---

## 0. What is different about this run, in four lines

1. **Every ToF range is correct for the first time.** Points were 0.10 m short in every
   flight before this one.
2. **Rule B has brake authority** — pinned by bench J this morning. First flight ever
   with the lidar-background rule live.
3. **The slow band is 0.60 m, not 0.30 m.** The rover will start easing off *twice as
   far out* as any previous run. **This will look different and it is not a fault.**
4. **Track 2 is aboard but touches nothing.** `task_node` publishes no velocity and is
   not in the motion path (asserted by `tests/test_task_node_safety.py`). It is inert
   until someone calls a service.

---

## 1. Preflight — cheap now, do it anyway

```bash
# on the Pi
cd ~/ros2_ws/src/sphero_rvr_ros && git pull --ff-only && git rev-parse --short HEAD
#   must print a667de2 -- TRUST THE SHA, NOT THE PULL'S OUTPUT
cd ~/ros2_ws && colcon build --packages-select sphero_rvr_driver     # ~3 s

# installed-tree verify: cheap, and it caught a real one this morning
for f in tof_node.py task_node.py coverage_explorer_node.py; do
  I=$(find ~/ros2_ws/install -name $f | head -1)
  S=$(find ~/ros2_ws/src/sphero_rvr_ros/src -name $f | head -1)
  echo "$f  $(sha256sum $I | cut -c1-16)  vs  $(sha256sum $S | cut -c1-16)"
done
#   the pairs MUST match. Before today the install had NO tof_node in it at all
#   while the source tree looked perfect.

timedatectl status | grep synchronized     # must be yes -- a fresh boot has been 72 min out
ros2 node list                             # must be EMPTY; if not, kill by PID, then `ros2 daemon stop`
```

**Sun check, and it is a scheduling gate not a code gate:** rule B now holds brake
authority on a sensor that has never been measured in direct sun. If the room has hard
sun on the floor, either wait for it to move or label the run as sun-contaminated
before starting. Do not discover this in the recording.

**Battery ≥ 25%, Scott within reach of the power switch, floor not a bench.**

---

## 2. Bringup and liftoff

Per the protocol's bringup order. The stack comes up **disarmed** (D29); liftoff is the
explicit service call, and the gates get read *before* it:

```bash
ros2 service call /coverage_explorer/mission/start std_srvs/srv/Trigger
```

**Gates to read before that call** — all four, against the live stack:

```bash
ros2 topic echo /tof/state --once --full-length
#   want: OK, rate_hz 6.5-7.6, i2c_errors 0, rules=rule_a+b, background=ok,
#         rule_b=pinned, margin_m=0.06, rule_a_rows=5|6|7, obstacle_consumers>=1
#   obstacle_consumers=0 means the supervisor is NOT subscribed whatever its config says

ros2 topic echo /collision_stop/state --once
#   want: pivot_veto=, cam_cloud_age=, output_angular_published= present

ros2 topic echo /coverage_explorer/status --once
#   NEW this run. want: armed=false, done=false -- i.e. disarmed and ready

ros2 topic hz /scan            # ~10 Hz
```

---

## 3. Watch list — what this run is for

Beats **every 60 s on a timer**, including "driving, all nominal". A silent stall
produces no event, and event-driven reporting has gone quiet exactly when it was most
needed, twice.

| watch | why it matters this run | what to note |
|---|---|---|
| **ToF brake engagements** | rule B has never flown | wall-clock time, what was in front, whether the stop looked early or late |
| **The 0.60 m slow band** | doubles the ease-off distance | does the rover crawl for uncomfortably long stretches? That is a tuning input, not a fault |
| **Phantom brakes** | J measured 0 in 5112 samples on ONE wall | any brake with nothing visibly there — note the surface, especially glass, gloss, dark or shiny floors, which J's sample did not contain |
| **The give-up escape branch** | field-unobserved; cannot be hand-staged | if it fires, note the time — the recording is the whole prize |
| **Abort split** | new fields, first flight | in the end report: `aborted_after_recovery` vs `aborted_without_recovery` |
| **`INCOMPLETE_NO_PLANNABLE_TARGETS` dominated by `without_recovery`** | the no-count rule | if it happens, this run does NOT count toward the three (`stall_survival_ladder.md` §7.1) — confirm from the launch log before deciding |
| **Contact** | always | cut chassis power, note the time, **leave every process running**, retrieve artifacts before teardown |

**Recording:** run the recorder before anything motor-capable, per protocol. If it is
cheap, also `ros2 bag record /scan /tof/points /tof/obstacles /tof/state /tf /tf_static`
— every mission bag is more bare-wall background for rule B's distribution, which is the
free way to widen a one-wall sample.

---

## 4. The demo segment — Scott types English at the robot

**With the stack still up**, between or after missions. Twenty minutes. This is the
thing the project has been building toward, and it doubles as the field proof of Track 2.

```bash
# Pane 4, stack already running:
ros2 run sphero_rvr_driver task_node
```

First, **prove it without the model** — Stage D in one command:

```bash
ros2 service call /task/status std_srvs/srv/Trigger
```

Then the REPL, **and Scott should be the one typing**:

```bash
ros2 run sphero_rvr_driver task_client
```

Script, in order, with what to expect:

| type this | expect |
|---|---|
| `what are you doing?` | a `status` call, then a plain-English answer naming idle/exploring |
| `explore the room` | an `explore` call, then a reply that says the mission has STARTED — **not** that the room has been explored |
| `are you done yet?` | a `status` call reporting it still running |
| `stop` | a `stop` call, and a reply that says it is **not** an emergency stop |
| `what do you know about shoes?` | a `query_semantic_map` call (only if `semantic_map` is up) |

**Three specific failures worth more than a success**, so write them down if they happen:

1. The model reports the room explored immediately after `explore` returns. The prompt
   warns against exactly this; if it happens anyway the prompt is not strong enough and
   that is a real finding.
2. The model treats `stop` as an emergency stop in its wording to Scott.
3. The model invents a tool or an argument. It should be refused at the boundary — the
   refusal reaching Scott as a sensible sentence is the thing to check.

**Safety during the demo:** every tool goes through Nav2 and then the collision
supervisor. `task_node` cannot publish a velocity. `stop` is a mission stop; the power
switch remains the real emergency stop and Scott stays within reach of it.

---

## 5. After

Artifacts **before** any Ctrl-C, per protocol: mission report (latched), map, recorder
CSV, launch log, bag if recorded. Then teardown: **lidar motor by service**
(`/stop_motor`, confirm `/scan` silent), processes by explicit PID — `pkill -f` matches
this session's own SSH command line and has been demonstrated to do so — then
`ros2 node list` empty and `ros2 daemon stop`.

Vault: `03_validation/gauntlet_2026-08-14_mission1/` with a README naming the binary
(`a667de2`), what was new (rule B live, 0.60 slow band, Track 2 aboard) and the outcome.

**Scoring:** mission 1 of 3. Read §7.1's no-count rule before recording the count — a
run whose ending is dominated by `aborted_without_recovery` needs the launch-log
classification first.
