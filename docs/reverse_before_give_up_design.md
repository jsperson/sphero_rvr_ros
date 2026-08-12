# Design note — reverse before giving up

**Status: design only. No code.** Written 2026-08-12 after gauntlet mission 2
(`run_20260812_125305`), which ended `INCOMPLETE_NO_PLANNABLE_TARGETS` with 0.78 m of
clear floor behind it. Scott, standing over it: *"This one is reasonably
understandable - except that in it's current spot it could have just backed up. New
rule: before giving up the rover should just try reversing."*

Every number below is cited to that run's artifacts or to a line of code.

---

## 0. THE PREMISE IS WRONG, AND THAT CHANGES THE WHOLE DESIGN

The obvious reading of Scott's rule is "add a reverse to the give-up path". **The
give-up path already has one, it already fired, and it failed in two milliseconds.**

From the mission-2 log, the explorer's own recovery running exactly where it should:

```
12:55:49.343  4 target(s) left but none plannable from here
              (0 rejected on CLEARANCE, 4 on the PLANNER) — unsticking (attempt 1/4)
12:55:49.368  unstick: back up did not finish, trying the next
12:55:49.592  unstick: turn toward the target did not finish, trying the next
12:55:49.594  unstick: nothing worked from this pose
```

Four attempts, all four the same, the whole sequence over in 250 ms. What happened
inside those 25 ms is in the behaviour server's own words:

```
12:55:45.424  behavior_server: Running backup
12:55:45.426  behavior_server: Collision Ahead - Exiting DriveOnHeading      <-- 2 ms
12:55:45.426  behavior_server: backup failed
12:55:45.446  behavior_server: Running spin
12:55:45.647  behavior_server: Collision Ahead - Exiting Spin
12:55:45.647  behavior_server: spin failed
```

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
   nothing plans  <-  planner refuses 96/96  <-  every escape behaviour refuses (2 ms)
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
have made lethal — refusing in 2 ms with 0.78 m of measured clear floor behind.*

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
| Assert, don't infer, at seams | The controller publishes the escape's **outcome** (`granted`/`refused`/`freeze`/`cleared`, plus distance achieved). The explorer never times a behaviour and never infers success from a pose delta it measured itself. |
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

A pose is refused while the robot's own body overlaps lethal-or-inflated cost. The mark
is lethal out to 0.14 m from its centre and inflated out to 0.14 + 0.16 = **0.30 m**.
So an escape that travels **0.30 m** takes a robot standing on top of its own mark to a
pose whose centre is outside that mark's inflated field:

```
escape_distance = freeze_mark_radius_m + inflation_radius = 0.14 + 0.16 = 0.30 m
```

That number is not a coincidence with the project's existing "0.30 m minimum start
clearance" figure — it is the same geometry read from the other side, which is the
strongest evidence available that it is the robot's own scale and not this room's.

**Time bound:** 0.30 m at 0.10 m/s is 3.0 s of granted motion, which is exactly
`rung_budget_s`. The escape uses that same budget as its ceiling so a refused or
ineffective escape cannot outlive one rung's worth of time.

---

## 5. The seam: what the controller publishes

New service (or action) on the controller — **request/response, not a topic pair**,
because the explorer must know when the escape ENDED, and a topic gives it only when
something was said:

```
/decisive_controller/escape_in_place   (std_srvs/Trigger, or a small custom srv)
  response.success  = the escape achieved the requested change of situation
  response.message  = one of:
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

**Why a service and not a ladder rung:** there is no active goal at this moment — no
`follow_path`, no execute loop, so no ladder. The controller's escape entry point must
work when it is otherwise idle. This is also why the escape cannot simply be "start a
goal and let the ladder handle it": planning is what already failed.

---

## 6. Failure modes, honestly

**(a) The reverse itself freezes — blind contact behind.** The controller's freeze
classifier is the same one the ladder uses: permitted output, no motion, for the
window. The escape must then do exactly what the ladder does — **mark it and stop**,
returning `frozen`. It must NOT escalate into arcs and pivots on its own: that is the
ladder's job during a goal, and duplicating it here is a second author for one motion,
which is the failure the ladder was created to end. The explorer, seeing `frozen`,
records the mark and gives up honestly — a rover boxed both front and back has genuinely
run out of room, and saying so is the right outcome.

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

**(d) Odometry lies about the distance.** The escape's own judgement uses the ladder's
changed-situation criterion, which is deliberately not a raw odometry distance. A
0.30 m reverse that odometry claims and the room does not corroborate would be caught
the same way rung 1's 0.12 m false success was.

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
   `behavior_server` and returns `backup failed` in 2 ms.
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

---

## 9. What this design explicitly does NOT do

* It does not touch the stall ladder. The ladder was invoked nine times in mission 2
  and every escape it ran cleared; it is not the defect.
* It does not touch `collision_stop.py`, the camera brake, or the low-obstacle path.
* It does not make `behavior_server` work. Recommendation: once this lands, the
  BackUp/Spin behaviours should be **removed from the explorer's vocabulary entirely**
  — they have now failed in both of the situations they exist for (D16's spin refusal,
  and this), and a recovery that cannot run is worse than no recovery because it
  consumes the branch that would otherwise reach a working one.
* It does not change the freeze-mark mechanism, though it interacts with it. Whether
  marks should decay faster, or be suppressed under the robot's own footprint, is a
  real question this run raises and a separate one.
