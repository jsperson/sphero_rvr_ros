# Design note — reverse before giving up

**Status: BUILT — `5fe3b24` and its follow-up, with §4/§6(a)/§6(d) amended at
implementation (F-A/F-B/F-C, marked inline).** Written 2026-08-12 after gauntlet mission 2
(`run_20260812_125305`), which ended `INCOMPLETE_NO_PLANNABLE_TARGETS` with 0.78 m of
clear floor behind it. Scott, standing over it: *"This one is reasonably
understandable - except that in it's current spot it could have just backed up. New
rule: before giving up the rover should just try reversing."*

Every number below is cited to that run's artifacts or to a line of code.

---

## 0. THE PREMISE IS WRONG, AND THAT CHANGES THE WHOLE DESIGN

The obvious reading of Scott's rule is "add a reverse to the give-up path". **The
give-up path already has one, it already fired, and it failed in two milliseconds.**

From the mission-2 log — **one attempt, both nodes, interleaved by their own
timestamps** (the first draft of this note quoted an explorer block from attempt 2 next
to a behaviour-server block from an earlier trigger four seconds away, which is a
citation splice and was caught in review; the nodes agree to the millisecond, there is
no clock skew, and the claim is unchanged once properly anchored):

```
1786557349.343  coverage_explorer   4 target(s) left but none plannable from here
                                    (0 CLEARANCE, 4 PLANNER) — unsticking (attempt 1/4)
1786557349.349  behavior_server     Running backup
1786557349.352  behavior_server     Collision Ahead - Exiting DriveOnHeading   <-- 3 ms
1786557349.352  behavior_server     backup failed
1786557349.368  coverage_explorer   unstick: back up did not finish, trying the next
1786557349.371  behavior_server     Running spin
1786557349.571  behavior_server     Collision Ahead - Exiting Spin
1786557349.571  behavior_server     spin failed
1786557349.592  coverage_explorer   unstick: turn toward the target did not finish
1786557349.594  coverage_explorer   unstick: nothing worked from this pose
```

Backup refused **3 ms** after it was asked; the whole attempt — both behaviours —
finished in **251 ms**. Five such episodes appear in the run (one before the give-up
sequence, then attempts 1-4), every one identical, and the `backup failed` interval is
2-4 ms in all five.

At that moment the robot's own lidar reported **rear 0.781 m** — 3.1x the supervisor's
`reverse_stop_distance_m` (0.25) — and it never dropped below 0.203 m across the 2465
recorded cycles the rover then sat there. **The sensor said "go", the costmap said
"collision", and the costmap won.**

Why the costmap said collision is the part worth keeping: the rover had planted five
freeze marks around itself (distinct positions, D35 discipline: 5 events / 5 places),
each a 0.14 m lethal disc, each inflated by `inflation_radius` 0.16. Standing among
them, its own footprint sits in its own inflation, and `nav2_behaviors`' collision
checker refuses **every** direction — backward, forward, rotational — because the cost
under the robot is already lethal. The same field is what refused all 96 planner
requests.

So the mechanism is a closed loop of the robot's own making:

```
  touch something invisible  ->  plant a freeze mark  ->  mark inflates over the robot
        ^                                                            |
        |                                                            v
   nothing plans  <-  planner refuses 96/96  <-  every escape behaviour refuses (3 ms)
```

**The lesson is not "add a reverse". It is that the escape is asking the wrong
oracle.** This project already learned this once, and wrote it into the ladder:

> Each is an ordinary drive command through the existing supervisor -- deliberately
> NOT a Nav2 behaviour, because decisive mode removes the local costmap that
> behavior_server's Spin gate reads (D16).
> — `stall_ladder.py:27`

The stall ladder escapes through the supervisor, which reads the **live lidar**. The
explorer's unstick escapes through `behavior_server`, which reads the **costmap**. The
ladder works in the field (mission 1: 9 pivot-first invocations, every escape it ran
cleared). The unstick does not, and mission 2 is the first run where that difference
decided the mission.

**D36 registered:** *the explorer's unstick is invoked correctly and executes nothing,
because Nav2's BackUp/Spin collision-check the costmap the rover's own freeze marks
have made lethal — refusing in 3 ms with 0.78 m of measured clear floor behind.*

---

## 1. What this design does, in one paragraph

The explorer keeps the trigger it already has and changes **who performs the escape**:
instead of `behavior_server`, it asks the **decisive controller** — the node that owns
motion, already escapes through the supervisor, and already knows how to judge whether
an escape changed anything. The explorer publishes a request, the controller performs
one bounded escape and **publishes the outcome as a fact**, and the explorer consumes
the fact rather than a timer or a guess. If the escape moves the rover clear, candidate
planning re-runs from the new pose; if it does not, the mission ends with an honest
report that now records *that an escape was attempted and what it achieved*.

Nothing about the stall ladder changes. It is not the component that failed.

---

## 2. Boundaries this design does not cross

| Rule | How it is honoured |
|---|---|
| The explorer does not publish velocity | It publishes a **request**; the controller is the only node that ever writes `cmd_vel`. Verifiable by diffstat: no publisher of `Twist` is added to `coverage_explorer_node.py`. |
| Assert, don't infer, at seams | The controller publishes the escape's **outcome** (`cleared`/`refused`/`frozen`/`declined`, plus the distance achieved). The explorer never times a behaviour and never infers success from a pose delta it measured itself. |
| Safety path untouched | Zero lines of `collision_stop.py`, the camera brake, or the low-obstacle detector. The escape is an ordinary command the supervisor gates like any other. Empty diffstat, verified. |
| No room-specific constants | The escape distance is derived from the costmap geometry that traps the robot (§4). |
| No new recovery layer | This REPLACES the body of `_unstick`; it does not add a parallel mechanism. `behavior_server` stops being asked to move this robot. |

---

## 3. The trigger, scoped precisely

**Fires when:** the explorer is about to give up with `NO_PLANNABLE_TARGETS` — i.e.
candidates remain, none is plannable from the live pose, and the existing
`_unstick_attempts < _max_unstick` bound has not been spent. This is exactly the branch
that already exists (`coverage_explorer_node.py:588`); its body changes, not its
condition.

**Does the goals-keep-failing path deserve the same courtesy?** Recommendation: **no,
and deliberately.** That path ends after five goals each of which ran the full stall
ladder at its own pose — the rover has already reversed, arced, pivoted and driven out,
up to four rungs per goal, with the supervisor ruling on each. An additional reverse
there is the fifth attempt at something that has failed twenty times, and the honest
answer at that point is that the mission is over. The two endings differ in exactly the
way that matters: `GOALS_KEEP_FAILING` means *we tried motion and it did not help*;
`NO_PLANNABLE_TARGETS` means *we never tried motion at all*. Only the second is a hole.

**Bound:** one escape per give-up decision, at most `_max_unstick` (4, unchanged)
across the mission, and the counter is NOT reset by a successful escape — a rover that
escapes, replans, and gets stuck again four times has told us something about the room.
No reverse-replan loop is possible: the counter is monotonic per mission, exactly like
the ladder's `max_total_traversals_per_goal`.

---

## 4. The escape itself, and where its distance comes from

**Command:** straight reverse at `ladder_reverse_speed_mps` (0.10), through the
supervisor, exactly as ladder rung 1 is issued. If the supervisor refuses it (`rear_hold`),
the controller falls back to a **reverse arc** — the same rung-2 reasoning, and the same
geometry that motivated it: `rear_hold` refuses the linear half and passes the angular
through.

**Distance — derived, not chosen.** The escape exists to get the robot's footprint out
of the inflation its own freeze marks project over it. Those are the three numbers that
build the trap, all from deployed config:

```
freeze_mark_radius_m   0.14   (decisive_controller_node.py, = robot_radius)
inflation_radius       0.16   (lean_nav2.yaml, global costmap)
robot_radius           0.14   (lean_nav2.yaml)
```

Stated precisely, because the first draft blurred two different radii and review caught
it. A mark is planted `footprint_front_m` **0.11 m ahead of the robot's centre** and is
a lethal disc of radius 0.14 m — so at the instant of planting the robot's own centre
is 0.11 m from the disc centre, i.e. **inside its own lethal mark**. From there:

| what has to become true | centre-to-mark distance | reverse needed |
|---|---|---|
| footprint no longer overlaps lethal (inscribed) | `mark_radius + robot_radius` = **0.28 m** | 0.17 m |
| cost fully free of that mark's inflation | `mark_radius + inflation_radius` = **0.30 m** | 0.19 m |

```
escape_distance_m = mark_radius + inflation_radius = 0.14 + 0.16 = 0.30 m
```

0.30 m of reverse puts the centre 0.41 m from the mark — clear of the inscribed zone by
0.13 m and of the inflated zone by 0.11 m. The margin is deliberate: it is one escape's
worth of slack against odometry error, not a second threshold.

**And one escape does NOT necessarily exit the field.** Mission 2's trap was five marks
inside about a metre; reversing away from one can move toward another, and a single
0.30 m escape has no guarantee of finding free cost. That is not papered over here —
it is precisely why the design re-runs candidate planning from the new pose and why the
escape budget is monotonic (§3): the loop is *escape, re-plan, and if it still does not
plan, escape again from somewhere new, up to four times, then report honestly*.
Convergence is bounded by the counter, not asserted by the geometry. A design that
claimed one reverse always clears a five-mark field would be claiming something this
run's evidence does not support.

**Time bound (AMENDED, F-B):** 0.30 m at 0.10 m/s is 3.0 s of motion *granted at full
commanded speed*, and the first draft made `rung_budget_s` (3.0) the ceiling on that
basis. That is too tight in practice: the supervisor routinely SLOWS rather than
refuses inside its slow band, so a 3.0 s ceiling would report `refused` for escapes
that were working. The ceiling is `give_up_escape_timeout_s` = **6.0 s**, one named
parameter that the controller defaults to and the explorer sends, so the two cannot
drift. The cost is bookkept in §6(d): it doubles the slip budget the `cleared` test
tolerates, from ~0.086 m to ~0.17 m against a 0.30 m bar — still a 1.8x margin, and
re-derivable if anyone raises the timeout again.

---

## 5. The seam: what the controller publishes

**It is an ACTION, and that is a decision, not a menu.** A `Trigger` service whose
handler drives for up to 3 s would run that loop inside the controller's executor
callback, starving the control loop, the scan subscription and the motor-output
subscription for the duration — the D22 executor-starvation family, in a node whose
entire job is to publish at 10 Hz. rclpy has no clean deferred service response, so a
service that "only initiates" has to fake completion through a second channel, which is
the topic-pair the explorer must not have to interpret. An action gives exactly the
right shape: goal, feedback, terminal result, cancellation, and a server that runs in
its own callback group without blocking the control loop.

```
/decisive_controller/escape_in_place   (nav2_msgs/action/BackUp reused, or a small
                                        custom action — see below)
  result.outcome = one of:
      cleared        moved >= escape_distance_m, or the ladder's changed-situation
                     criterion was met (lateral / heading), with the distance achieved
      refused        the supervisor zeroed every cycle (rear blocked) — reported with
                     the supervisor's own reason string, not our guess
      frozen         permitted but immobile: blind contact BEHIND us (see §6)
      declined       the controller is mid-goal or the ladder is active; not our turn
```

The explorer logs and records whichever fact came back. It does not measure the
outcome itself; that is the seam rule (`_open_bearing`'s `None` fix, one batch ago, is
the precedent).

**`declined` IS A LOGIC ERROR, not a retry condition.** In the designed flow the
explorer only ever asks while it is idle with no goal outstanding, so a `declined` means
one of the two components is wrong about the other's state — exactly the class of thing
that hides for weeks if it is swallowed. It must be logged at WARN with both sides'
view, counted as a FAILED escape against the monotonic budget, and never silently
retried. A quiet retry loop here would rediscover the give-up livelock from the other
direction.

**Why an action and not a ladder rung:** there is no active goal at this moment — no
`follow_path`, no execute loop, so no ladder. The controller's escape entry point must
work when it is otherwise idle. This is also why the escape cannot simply be "start a
goal and let the ladder handle it": planning is what already failed.

---

## 6. Failure modes, honestly

**(a) The reverse itself freezes — blind contact behind.** The controller's freeze
classifier is the same one the ladder uses: permitted output, no motion, for the
window. **Literally the same rolling rule** (`WindowedFreezeMonitor`, sharing the
ladder's reference-remark semantics) rather than a similar-sounding one — the first
implementation compared TOTAL travel since the escape began against
`progress_epsilon_m` 0.03, which never fires against a real blind contact, because a
pinned reverse creeps 0.086 m per window (mission 1, measured). That version would
have reported `refused` while the supervisor was permitting motion, and planted no
mark, in precisely the case this feature exists for. The escape must then do exactly what the ladder does — **mark it and stop**,
returning `frozen`. It must NOT escalate into arcs and pivots on its own: that is the
ladder's job during a goal, and duplicating it here is a second author for one motion,
which is the failure the ladder was created to end. The explorer, seeing `frozen`,
records the mark and gives up honestly — a rover boxed both front and back has genuinely
run out of room, and saying so is the right outcome.

**AND THE MARK WOULD GO ON THE WRONG SIDE — a shipped defect this design would
otherwise walk into.** `_freeze_mark_pose` (`decisive_controller_node.py:481`) projects
the mark `footprint_front_m` along `robot_yaw`, unconditionally:

```python
return (robot_x + self._footprint_front_m * math.cos(robot_yaw),
        robot_y + self._footprint_front_m * math.sin(robot_yaw))
```

For a freeze during a REVERSE that plants a lethal disc **0.11 m in FRONT** of a robot
whose obstacle is BEHIND it: the real obstacle goes unmarked, and 0.22 m of clear floor
ahead is poisoned — deepening the very unplannability trap this design exists to break.

**Audit: can today's shipped ladder hit it? No — and the reason is worth stating,
because it is why this has never been seen.** A freeze is classified only in
`StallLadder._begin`, and `step()` returns `_run_rung(...)` immediately whenever a rung
is active, so no freeze can ever be classified while the ladder is commanding a reverse.
At `_begin` the commanded motion is the controller's own drive command, which is never
negative. Across gauntlet missions 1 and 2 (9 and 5 freeze events) not one was
classified mid-reverse. The defect is **latent in shipped code and becomes reachable the
moment an escape reverses on its own** — the same shape as D32/D33, where step 1's
ordering fix made a dormant defect live.

**Fix, in this same diff:** the mark follows the COMMANDED MOTION DIRECTION — leading
edge for forward, trailing edge (`-footprint_rear_m` along yaw) for reverse. Pinned by
its own revert-proof (§8.7), which fails against HEAD.

**(b) Lifecycle interaction.** The escape runs when no goal is active, but the explorer
could send a new goal the instant planning succeeds afterwards. The rule from D34
(`goal-cancel-kills-the-ladder`) applies to whatever this adds: **the escape completes
or is cancelled explicitly; it is never left running while a goal starts.** The
controller must refuse to start a `follow_path` while an escape is in flight (returning
`declined` in the other direction is not enough — the goal must wait or be rejected),
and `mission/stop` must abandon an escape in progress, as it already abandons an
unstick (F5).

**(c) The escape succeeds and planning still fails.** Expected, and not a bug: report
`NO_PLANNABLE_TARGETS` with the new field below. Four such cycles end the mission.

**(d) Odometry lies about the distance.** AMENDED AT IMPLEMENTATION (F-C), because
the first version of this clause promised something that would break the feature.

The clause said the escape would be judged by the ladder's changed-situation criterion
"deliberately not a raw odometry distance". That criterion is *lateral displacement or
heading change*, and it exists to reject exactly what this escape deliberately does:
travel in a straight line along the axis it is facing. Applied here it would refuse to
credit every successful escape ever made. The ladder is asking "did the stall
situation change"; this escape is asking "am I out of my own mark's inflation", and
the honest answer to the second one IS a distance.

So `cleared` is raw odometry travel >= `escape_distance_m`, and the reason that is safe
is measured rather than assumed: **a pinned reverse creeps.** Mission 1, 13 straight
reverse windows against a blind contact with the supervisor granting 79% of cycles,
mean travel achieved **0.086 m per 3 s window** — so fabricating 0.30 m out of slip
takes about 10 s of continuous slipping, and the escape is bounded at
`give_up_escape_timeout_s` **6.0 s** (~0.17 m of pure creep, a 1.8x margin). That bound
is load-bearing for this clause, which is why it lives in a named parameter that both
nodes send and default to, rather than in whichever number each side happened to pick.

If the timeout is ever raised, this margin has to be re-derived: at 10 s the slip
budget reaches the bar and `cleared` stops meaning anything.

**(e) The rover reverses into unmapped space and the map worsens.** Accepted: 0.30 m is
under one robot length, the supervisor gates the whole of it, and SLAM sees the same
scan it always did.

---

## 7. Report fields (D35 discipline)

The mission report gains, alongside the existing counters:

```
"give_up_escapes": {"attempted": 4, "outcomes": {"refused": 3, "cleared": 1},
                    "distinct_poses": 2}
```

Events AND distinct positions, because mission 1's `freeze_marks: 9` meant six places
and I read it as nine obstacles in my own first report — the exact misreading this
field shape prevents.

---

## 8. Revert-proofs

1. `mission_2_ending_reverses_before_giving_up` — replay the recorded ending: 4
   candidates, 96 planner rejections, rear 0.781 m clear, freeze marks at the five
   recorded positions. The explorer must REQUEST an escape and the controller must
   COMMAND a reverse. **Fails against `b684515`**, where the request goes to
   `behavior_server` and returns `backup failed` in 3 ms.
2. `a_refused_reverse_is_reported_as_refused_not_as_success` — supervisor zeroes every
   cycle (`rear_hold`); the explorer must record `refused` and give up honestly rather
   than replan into the same trap.
3. `an_escape_that_freezes_marks_and_stops` — permitted-but-immobile behind: one mark,
   outcome `frozen`, and NO escalation into arcs or pivots.
4. `the_escape_budget_is_monotonic` — four escapes across a mission, then the fifth
   give-up decision proceeds straight to the report; a successful escape does not
   refill the counter.
5. `no_goal_starts_while_an_escape_is_in_flight` — the D34 pairing test for this
   lifecycle: a `follow_path` arriving mid-escape must not silently kill it.
6. `the_explorer_still_publishes_no_velocity` — structural: no `Twist` publisher in
   `coverage_explorer_node.py`, asserted by source scan, as `test_ros_safe_surfaces`
   already does for other nodes.
7. `a_reverse_freeze_marks_behind_the_robot` (R1) — a freeze while the commanded motion
   is negative must place the mark on the TRAILING edge, behind the footprint. **Fails
   against HEAD**, which places it 0.11 m in front along `robot_yaw` regardless of
   direction. Paired negative: a forward freeze still marks the leading edge, so the
   D25 leading-edge correction is preserved rather than traded away.
8. `declined_is_counted_and_logged_not_retried` (R5) — an escape requested while the
   controller is mid-goal returns `declined`, is logged at WARN, spends one unit of the
   monotonic budget, and does not loop.

---

## 9. What this design explicitly does NOT do

* It does not touch the stall ladder. The ladder was invoked nine times in mission 2
  and every escape it ran cleared; it is not the defect.
* It does not touch `collision_stop.py`, the camera brake, or the low-obstacle path.
* It does not make `behavior_server` work. **Approved at review: BackUp/Spin leave the
  explorer's vocabulary in this same diff** — they have now failed in both of the
  situations they exist for (D16's spin refusal, and this), and a recovery that cannot
  run is worse than no recovery because it consumes the branch that would otherwise
  reach a working one. Call-site audit, so "removed" means removed: the ONLY
  construction of those clients in the repo is `coverage_explorer_node.py:283-284`
  (`ActionClient(self, BackUp, "backup")`, `ActionClient(self, Spin, "spin")`).
  `task_node.py`, `task_client.py` and `vlm_explorer_node.py` contain no BackUp/Spin
  client (the one grep hit in `task_client.py` is the word "Spin" in a comment). So
  deleting those two clients takes Nav2 behaviours out of this robot's motion path
  entirely; `behavior_server` itself stays in the launch for now, unused, and retiring
  it from the launch is a separate cleanup.
* It does not change the freeze-mark mechanism, though it interacts with it. Whether
  marks should decay faster, or be suppressed under the robot's own footprint, is a
  real question this run raises and a separate one.
