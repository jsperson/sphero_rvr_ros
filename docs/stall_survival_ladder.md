# Design note — the stall survival ladder

**Objective, in Scott's terms:** *no single stall class may end a mission.* We have
been fixing stall causes one at a time for weeks while the mission contract
surrenders to each new cause. This note proposes one behavioural contract that
covers the whole cluster, and it is deliberately more DELETION than addition.

**Status: design only. Nothing here is implemented.** Every claim below is either
measured on 2026-08-10 (cited) or flagged as an assumption to test.

---

## 1. What actually kills missions — two runs, same night, same code

Both missions on 2026-08-10 ended `ABORTED_GOALS_KEEP_FAILING`, and it is worth
being precise that they died of **different causes**. That is the whole problem:
each cause is individually terminal.

| | run 185048 | run 190528 |
|---|---|---|
| duration | 52.7 s | 76.9 s |
| goals sent / succeeded | 5 / 0 | 9 / 2 |
| coverage | 2.85 m² | 4.50 m² |
| freeze marks | 0 | 0 |
| killer | refused **pivot** | refused **reverse** |

Neither mission was killed by the room, by staging, or by perception. Run 185048
started with 0.60 / 0.80 / 0.90 / 0.82 m clearance on the four sides — well clear of
the 0.30 m inscribed threshold — and drove itself into a pocket. Run 190528 started
even more open and never got further than **1.29 m from its start pose**, spending
**62 of its 77 seconds inside a single 0.25 m cell**.

### Class A — the refused pivot, invisible to the guard

Run 185048, last 14 s: 137 of 140 recorder rows carry `cmd = (0.000, -0.900)` — a
pure pivot — against `out = (0.000, 0.000)`. The supervisor refused every pivot; the
controller asked for the same pivot 137 more times; `odom_yaw_deg` held at -136.4
throughout.

`ProgressGuard.step` never noticed, and it never can. `decisive_control.py:246`:

```python
if not translating:
    # Pivoting or arrived: position is not expected to change, so do not
    # accrue stall time. Re-arm the progress clock on the next drive cycle.
    self._ref = None
    self._ref_t = None
    return GuardResult("drive")
```

`translating = command.mode in ("straight", "arc")` (`decisive_controller_node.py:293`),
so a pivot takes this branch on every cycle and **resets the guard's own progress
clock**. Replaying run 185048's dead 14 seconds through the real guard yields
`{drive: 280}`, zero freezes. Sweeping the duration: a pivot blocked for 5, 10, 30,
60 and 300 seconds all produce **zero** non-drive actions. This is not a tuning
problem; it is unbounded blindness.

The comment is correct that position should not change during a pivot. The bug is
that the guard measures **only x/y**, so it cannot distinguish *turning* from *being
refused permission to turn*. **Yaw is the missing measurement.**

### Class B — the refused reverse, visible but terminal

Run 190528: three of seven aborts share one signature — in the 2 s before the abort,
13 of 20 rows are `state=SLOW reason=rear_hold` with **0 of 20 rows moving**.

Decoded: the back-off reflex asks for straight reverse, the supervisor refuses it
because the rear is inside `rear_stop_distance`, output is `(0,0)`, `back_off_timeout_s`
(3 s) expires, `_give_up()` aborts the goal. Five such aborts in a row end the mission.

So `"boxed in — backing off did not clear it"` means, literally: **we tried the only
escape we have and were refused.** There is no second rung.

One detail makes this especially cheap to fix: `rear_hold` passes **angular through
untouched** (`collision_stop.py:951`). A reverse *arc* would have kept its rotation.
It is the reflex asking for angular exactly zero that guarantees the zero output.

### 1.1 The mission gave up with four escapes available — measured

Run 190528's 44 `rear_hold` rows sit at a near-identical pose: front 0.58, rear 0.23,
left 0.44, right 0.41, refusing a command of exactly `(-0.1, 0.0)` every time. Feeding
that geometry to the real supervisor core:

| candidate rung | command | supervisor verdict | output |
|---|---|---|---|
| straight reverse (the only one tried) | (-0.10, 0.00) | SLOW `rear_hold` | **(0, 0) refused** |
| reverse arc, either way | (-0.10, ±0.40) | SLOW `rear_hold` | (0, ±0.40) **granted** |
| pure pivot, either way | (0.00, ±0.40) | CLEAR `command` | (0, ±0.40) **granted** |
| **forward** | (+0.10, 0.00) | SLOW `front_slow` | **(0.093, 0) granted** |

The rover aborted three goals, and then the mission, **while it had 0.58 m of clear
space in front of it and permission to drive into it at 0.093 m/s.** It gave up
because the single escape it knows — straight reverse — was refused. This is the
strongest possible argument for the ladder, and it is a measurement rather than an
argument.

It also settles rung 2: `rear_hold` fires *before* the trajectory gate and passes
angular through untouched, so a reverse arc is genuinely granted where a straight
reverse is refused. Rung 2 is a rung.

*Caveat, recorded because it bit me earlier the same night:* this reconstruction uses
a scan populated only in the four named sectors, so bearings between them read free.
The `rear_hold` and `front_slow` results are sound — both are sector tests reached
before the trajectory gate — but any rung whose verdict comes from the trajectory gate
(the pivot rows above) is optimistic here and must be re-checked against a real scan.

### The third mechanism: a ladder rung that already exists and is unreachable

`coverage_explorer_node.py` already implements backup-and-spin recovery —
`max_unstick_attempts` (4), `unstick_backup_m` (0.25), `unstick_spin_rad` (1.57),
`unstick_timeout_s` (12.0).

**It fired zero times across both runs.** Its trigger (`coverage_explorer_node.py:453`):

```python
if goal_cell is None and candidates and self._unstick_attempts < self._max_unstick:
```

It runs only when *selection* fails to produce a goal. In both runs selection always
succeeded — 5 and 9 goals sent — and it was the *drive* that failed. The recovery is
wired to a condition that does not occur in the failure mode that kills us.

This is the same defect class as the unreachable pivot controller: plausible code,
never executed, invisible in review. **We do not need to invent a ladder. We need to
rewire the one we have to the condition that actually happens.**

---

## 2. Before and after

### Before — three reactions, three owners, no escalation

```
                    goal active, robot not progressing
                                  |
        +-------------------------+--------------------------+
        |                         |                          |
  translating?                pivoting?              selection failed?
        |                         |                          |
   ProgressGuard             (nothing —            explorer _unstick
   back-off reflex            guard resets          backup + spin
        |                      its clock)                    |
   reverse, straight only          |                    NEVER REACHED
        |                     never escalates            in either run
   refused (rear_hold)             |
        |                     mission dies
   _give_up() -> abort             quietly
        |
   explorer counts a failure
        |
   5 in a row -> mission over
```

Three separate reactions to one condition; two of them unreachable in practice; the
one that runs has a single rung that the supervisor can veto outright.

### After — one condition, one ladder, escalation until genuinely exhausted

```
              NO-PROGRESS DETECTED  (one predicate, all causes)
   position not advancing  OR  yaw not advancing  OR  supervisor
   has zeroed our chosen action for N consecutive cycles
                                  |
                          +-------+-------+
                          |   THE LADDER  |
                          +-------+-------+
                                  |
   rung 1  reverse along the entry direction (known clear — we came that way)
                                  |  refused or ineffective
   rung 2  reverse ARC (rear_hold passes angular; straight reverse does not)
                                  |  refused or ineffective
   rung 3  pivot toward the most-open lidar sector
                                  |  refused or ineffective
   rung 4  drive 0.5 m into that open sector
                                  |  refused or ineffective
                          LADDER EXHAUSTED
                                  |
              mark the pose, report it, and only NOW count a failure
```

**The counter ticks once per exhausted ladder, not once per refused action.** That is
the contract change that makes "no single stall class ends a mission" true: a stall
can only end a mission if *every* escape was tried and refused.

---

## 3. The no-progress predicate

Today "no progress" means "x/y did not advance while translating". It must become the
union of three measurable conditions, each of which we have now seen kill a mission:

1. **Position stalled** while a translating command is active — today's condition, kept.
2. **Yaw stalled** while a pivot command is active — Class A. `odom_yaw_deg` is already
   in the recorder and already available to the controller; nothing new is sensed.
3. **Output suppressed** — the supervisor's `cmd_vel_motor` has been zero for N
   consecutive cycles while we were commanding non-zero. This is the general case that
   subsumes both: it does not care *why* we are not moving.

Condition 3 alone would have caught both nights' failures. Conditions 1 and 2 are worth
keeping because they also catch the case where the supervisor permits motion and the
robot still does not move — which is the *freeze*, and remains genuine discovery.

**Freeze classification and freeze marks stay exactly as they are.** They answer a
different question ("is there something here no sensor can see?") and they answer it
correctly — run 185048's empty `freeze_marks` was the *right* answer, because the
supervisor was refusing motion, not permitting it. The ladder is what *invokes* the
classifier; it does not replace it.

---

## 4. Prevention — do not park in the pocket

Recovery is the second line. The first is not entering the trap.

- **Goal-pose clearance filter.** Reject any candidate goal whose pose has < 0.35 m
  predicted clearance on any side. The global costmap already holds this; it is a
  filter over candidates, not new sensing. Run 185048 ended with 0.22 m on two sides —
  a pose the filter would never have accepted as a *destination*.
- **Abort a leg early when live clearance collapses.** If clearance on any side drops
  below threshold while approaching, abandon the leg *while still able to manoeuvre*,
  rather than continuing to the goal and parking in the pocket. This is the difference
  between "the rover stopped somewhere awkward" and "the rover is now unplannable".

Both are cheap, and prevention is worth more than any ladder rung: the pocket in run
185048 left the robot with 22 cm on two sides, where *every* rung is likely refused.

---

## 5. What gets DELETED

Per the review ruling — if this note proposes addition without deletion, it is wrong.

| Deleted / rewired | Where | Why |
|---|---|---|
| **`ProgressGuard` back-off reflex** as an independent reaction | `decisive_control.py` — the `_backing_off` state machine, `back_off_*` config | Becomes rung 1 of the ladder. Its reverse-only, translating-only scope is exactly the limitation the ladder exists to remove. The guard keeps *detection* and freeze classification; it loses its private recovery. |
| **`_give_up()` → abort on back-off exhaustion** | `decisive_control.py` | Replaced by ladder exhaustion. Today three refused reverses end a goal; that is a single rung failing. |
| **Explorer `_unstick` and its four parameters** | `coverage_explorer_node.py:453, 763` | Deleted outright. Its backup+spin content becomes rungs 1 and 3; its trigger is the wrong one and has never fired. Keeping it alongside the ladder is precisely the fourth layer we were told not to build. |
| **Per-goal failure counting on drive failure** | `coverage_explorer_node.py` | Counter ticks on ladder exhaustion instead. Same 5-strike safety net, correct unit. |
| **The "at different places" diagnostic** | `coverage_explorer_node.py` | Actively false and must not survive. It reports "5 goals in a row failed, at different places — this is the stack, not the room", measured against *goal* positions. In run 190528 the **robot** sat in one 0.25 m cell for 81% of the mission. The message asserts the opposite of what happened and sends the next debugger to the wrong place. |

**Kept unchanged:** the collision supervisor and all its gates (it was correct both
nights — it refused motions that would have hit things); freeze marks and the freeze
layer; goal suppression by location (`stall_suppress_*`) — that is memory, not a
reaction, and it composes with the ladder.

---

## 6. Where the ladder lives

**In the decisive controller**, not the explorer. It authors motion, it already
subscribes to `cmd_vel_motor` so it can see what the supervisor actually permitted, and
it has the pose and yaw. The explorer keeps mission-level accounting only: it counts
exhausted ladders and decides when to stop.

This also means the ladder's rungs are ordinary drive commands through the existing
supervisor, not Nav2 behaviours — which matters, because `behavior_server`'s Spin gate
reads a local costmap that decisive mode removes (D16).

---

## 7. Acceptance — binary, Scott-facing

> **Three consecutive missions in this room, no human rescue, each ending `COMPLETE`
> or honestly blocked.** A mission may fail only by exhausting the ladder everywhere.

Sub-criteria, each falsifiable from artifacts we already collect:

- Zero missions end while any rung remains untried at the final pose.
- Every ladder invocation is reported: which rung ran, whether it was refused, and the
  supervisor's `reason` at refusal (the `reason` column landed in `de8d73b`).
- Class A regression: a blocked pivot must escalate. Directly testable offline against
  `ProgressGuard` — today it produces `{drive: N}` for any N.
- Class B regression: a refused straight reverse must reach rung 2, not abort.

---

## 8. D29 — no armed state (rides along)

Both runs auto-started at launch: `start_explore:=true` begins exploring during bringup,
so run 185048's entire 53 s mission was over before the first bringup gate was read, and
Scott watched a stopped rover with no information. There is no armed/disarmed state, so
"gates, then go" is not expressible with the current launch file.

The ladder needs a mission-control surface anyway (start, stop, status), so D29's fix
folds in: an explicit `mission/start` and `mission/stop` service, with `start_explore`
defaulting to **false**. Launch brings the stack up; a service starts the mission.

---

## 9. What must be measured before implementing

Stated explicitly because two D27 fixes died tonight from confident reasoning that
skipped the measurement.

1. ~~**Would rung 2 (reverse arc) actually have been granted?**~~ **MEASURED — and the
   answer is bigger than the question.** See section 1.1 below.
2. **Yaw-stall threshold.** How much yaw change per second counts as progress? Needs a
   number derived from the pivot rate the supervisor permits (`max_angular_rad_s` 0.4),
   not from this room.
3. **Rung 1's "entry direction"** requires remembering how we arrived. Cheap, but it is
   state the controller does not keep today.

---

## Appendix — D27 status, for completeness

D27 (sunlight phantom low obstacles) is **not** part of this ladder and does not block
it. Three candidate fixes are now dead on measurement:

- **Texture/CV normalisation** — needs a CV ratio of 1.0; measured 0.37.
- **Reference-locality** — measured *worse*: brake-band phantoms 79 → 103, because the
  sun band runs diagonally and a per-column reference fixes the horizontal axis while
  worsening the vertical one (up to 60 grey levels within one column).
- **Height gate** — measured tonight and it **does not separate**. The 79 phantom
  brake-band points have median `est_h` 0.081 m (p25 0.080, p75 0.083), and a real 8 cm
  shoe at 0.45 m projects to 30.2 px against the phantoms' median span of 31 px. The
  phantoms sit exactly on top of the most important real target. Angular extent does not
  separate them either: 38.2° phantom vs 31.0° for a 0.25 m shoe at 0.45 m.

**One candidate survives, untested:** *height uniformity across the span*. The phantom
cluster's `est_h` standard deviation is **0.0030 m across 61 columns spanning 38°** —
a light boundary is a geometric line, so its apparent height is nearly constant, whereas
a real 3D object's silhouette varies across its width.

**Prediction registered in advance of the measurement:** a real shoe's `est_h` stdev
across its span will be substantially larger than 3 mm. If it is not, this candidate is
dead too and D27 needs a different class of solution. This requires the shoe cells of
the four-cell sunlit protocol; it cannot be settled from the existing fixture pair.
