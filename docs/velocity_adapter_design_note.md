# DESIGN NOTE: the sub-floor rotation request — arithmetic, and a config-only fix

**§3a, 2026-08-17. For fast review. Conclusion: this is fixable in CONFIG. No adapter code
should change today.**

## Correction to my own first draft, before anything else

My first draft said RPP's rotate-to-heading *tapers as heading error shrinks*, and
proposed policy work on that basis. **That is wrong.** `RegulatedPurePursuitController::
rotateToHeading()` does not taper on error at all — it commands
`rotate_to_heading_angular_vel` and then clamps it to an **acceleration ramp from the
current measured speed**:

```
angular_vel = clamp(sign * rotate_to_heading_angular_vel,
                    curr_speed.angular.z - max_angular_accel * dt,
                    curr_speed.angular.z + max_angular_accel * dt)
```

It is an acceleration limit, not an error taper. Everything below follows from that, and
the policy menu in the first draft was solving the wrong problem.

## The arithmetic

```
control_duration dt   = 1 / controller_frequency = 1/20.0        = 0.050 s
max_angular_accel                                                = 1.20 rad/s^2
FIRST rotate command from rest = max_angular_accel * dt          = 0.060 rad/s
```

**The measured first command in §3a was `cmd_wz = 0.06`.** Exact match — the mechanism is
confirmed by a number, not by a story.

### SECOND CORRECTION — the "59-cycle ramp" was my arithmetic, and the data refutes it

I then computed `3.55 / 0.06 = 59 cycles = 2.96 s` of sub-floor commands. **That assumed
the measured speed follows the commanded ramp. It does not** — the clamp is against
`curr_speed` from **odom**, and the robot snaps to full rate on the very first command, so
odom immediately reports a large rate and the window jumps with it. Measured from the bag:

```
 t        cmd_wz    odom_wz just before    accel window
179.76     0.060          0.000            [-0.06, 0.06]  CLAMPED   <- the onset ramp
179.81     0.060          0.000            [-0.06, 0.06]  CLAMPED      ...is THREE
179.86     0.060          0.000            [-0.06, 0.06]  CLAMPED      commands long
181.75     3.550          0.111
181.95     3.550          0.999
182.15     3.550          3.320
182.35     3.550          3.564            [3.50, 3.62]   CLAMPED
```

**The up-ramp collapses in one to two cycles, exactly as predicted on review. Three
sub-floor commands at onset, not 59.**

### The mechanism the data actually shows: a REVERSAL limit cycle

The sub-floor commands cluster **after overshoot and sign reversal**, not at onset:

```
182.49   -2.162    odom +3.187      182.74   -2.140    odom -2.502
182.54   -2.293    odom +2.504      182.84   -1.937    odom -3.331
182.64   -0.926    odom -0.866      ...
```

And the measured rotation rate is violent and alternating: **odom `|wz|` reaches 4.68 rad/s,
p95 3.56** — above the floor itself. The loop is:

> command → **floor snap to ±3.55** → odom reports a large, noisy, sign-alternating rate →
> RPP's acceleration window is centred on *that* → the next command lands sub-floor or
> reversed → floor snap the other way.

**The controller and the drivetrain are in a limit cycle, and the acceleration clamp is
the coupling.** Onset contributes three commands; the limit cycle contributes the rest of
the 35 sub-floor commands out of 64.

Aggravating factor, bounded: **odom publishes at ~10 Hz while the controller runs at
20 Hz**, and at floor rate the robot sweeps ~20° between odom samples. The controller's
notion of "current speed" is therefore stale and coarse by construction. This is a
residual wobble source; it is bounded by the tolerance quantum (0.178 rad vs 0.30 rad
tolerance) and is not the primary defect.

The quantum that bounds convergence:

```
one control cycle at floor rate = 3.55 * 0.05 = 0.178 rad
yaw_goal_tolerance                            = 0.300 rad   (1.7 quanta -- OK)
rotate_to_heading_min_angle                   = 0.600 rad   (3.4 quanta -- OK)
```

Both tolerances are comfortably larger than the quantum, so **convergence is not the
problem. The ramp is.**

## The fix, config-only

**Make the ramp irrelevant by making the first command already legal.** Require:

```
max_angular_accel * dt  >=  minimum_clean_rate(pivot_min_duty)
max_angular_accel       >=  3.55 / 0.05  =  71.0 rad/s^2
```

Proposed `max_angular_accel: 80.0` (and `rotational_acc_lim: 80.0` for `behavior_server`,
which has the identical 1.20 and therefore the identical defect in Spin).

**The reversal mechanism makes this a STRONGER argument for 80, not a weaker one.** At
80 rad/s² the clamp window is ±4.0 rad/s per cycle — wider than the full range odom ever
reports (max 4.68, p95 3.56). **The clamp stops binding at all**, so RPP emits its intended
±3.55 every cycle instead of whatever the noisy measured speed permits, and the coupling
that closes the limit cycle is removed. A small limit does not damp this system; it *is*
the feedback path.

**What that does NOT settle, and the simulator must:** whether the hunt actually resolves
once every command is ±3.55. The robot will still snap, still overshoot by up to one
0.178 rad quantum, and odom will still be coarse. Removing sub-floor commands satisfies
the fix's invariant; it does not by itself prove convergence. **That is the question the
closed-loop proof exists to answer, and it is why the config change does not fly on
argument alone.**

**80 rad/s² looks absurd and is honest.** An acceleration limit expresses "do not change
speed faster than this". **This drivetrain physically cannot change rotation speed more
slowly than 0 → 3.55 in one command** — there is nothing between the dead zone and the
floor. So a small acceleration limit is not a safety property here; it is a fiction that
generates commands the hardware converts into exactly what a large limit would have
produced, minus the honesty. Setting it above the floor/dt threshold makes the config
describe the machine.

**What this changes in behaviour:** every rotation command RPP emits is at or above the
floor, so the driver's raise-and-log never fires on the rotate path, and commanded rate
equals executed rate. The hunt loop's cause is removed without touching the adapter.

**What this does NOT fix, stated plainly:** the supervisor still clamps rotation to
0.4 rad/s and the driver still raises it back — the two cancelling errors remain, and the
supervisor pivot-clamp fix stays the first reviewed change afterwards. With this config
change the round trip becomes *consistent* rather than *accidental*, but it is still a
round trip.

## Arcs are NOT implicated — answering the direct question

The `vx>0` rows went out on the tank-SI path as float tread speeds, never as duties:

```
left 0.150 / right 0.250 m/s  ->  v 0.200, w +0.400
left 0.250 / right 0.150 m/s  ->  v 0.200, w -0.400
left 0.237 / right 0.163 m/s  ->  v 0.200, w -0.296   <- sub-0.4 modulation preserved
BackUp: left -0.150 / right -0.150 -> straight reverse, clean
```

**Arc rate modulation works and is not quantized.** The defect is confined to the in-place
rotation path. That also means fine heading correction *while moving* is already
available; it is only stationary correction that is impossible.

## Every candidate, evaluated against the ACCELERATION-RAMP mechanism

The first draft's menu was written against an error-taper that does not exist. Re-run
against the real mechanism — a THREE-command onset ramp plus a reversal limit cycle —
the options score very differently:

**(a) refuse-and-zero (deadband).** Sub-floor requests command nothing. Against the ONSET
ramp this is **actively broken**: the first command is 0.06, refusing it leaves
`curr_speed.angular.z` at 0, so the next command is clamped to 0.06 again. **The ramp never
escapes its own first step — rotation never starts at all, permanently.** A deadlock, not a
stall risk. It was my first draft's preferred option and it would have hung the robot on
its first turn. (The onset ramp is only three commands when they are EXECUTED — refusing
them is what makes it permanent.)

**(b) config-only: raise the acceleration limit above floor/dt.** Every emitted rotation
command is at or above the floor from the first cycle. Commanded rate equals executed
rate. No adapter code, four numbers, derivable from `pivot_curve`, guardable by one
inequality. **Preferred.**

**(b′) config-only variant: raise `rotate_to_heading_min_angle` instead**, so in-place
rotation is never entered for small angles and fine correction falls to tracking arcs.
**Does not work alone** — the ramp problem is on *entry*, not on the angle threshold. Even
a large-angle rotation starts at 0.06 rad/s and walks up. Raising `min_angle` reduces how
*often* the defect is met without fixing a single instance of it. Worth doing later for
other reasons; not a fix.

**(c) deadband + coast hybrid.** Execute at floor until within a band, then refuse. Two
thresholds, both needing the quantum, and it inherits (a)'s deadlock at the ramp's start
unless it is *entry*-aware. More machinery than (b) for no additional property.

**(d) reshape sub-floor pivots as arcs.** Uses the regime that demonstrably works, and is
the *right* long-term answer for fine correction — but it requires room to translate, and
arc rates are unmeasured (`run_card_arc_rate_FUTURE.md`). Not a today decision, and it
should not be made under field pressure.

## Pre-registered acceptance for the chassis-off closed-loop proof

Stated **before** the simulator is built, so the criteria cannot be tuned to whatever it
produces:

| criterion | threshold | goal 2's actual |
|---|---|---|
| **yaw economy** — total yaw swept per metre of net progress | **< 360 °/m** (at most one full turn per metre) | **4029 °/m** — fails by >11× |
| **net progress** toward a 1.0 m goal | **≥ 0.8 m** | **0.069 m** |
| **every emitted rotation command** | at/above floor, or exactly zero | 35 of 64 sub-floor |
| **hunting signature** — `out_wz` sign alternations per metre of progress | **< 20 /m** | 9 alternations for 0.069 m ≈ 130 /m |

Yaw economy is deliberately scale-free: a goal that legitimately needs a 90° turn is not
penalised, and a robot that spins in place to go nowhere fails regardless of route length.

**THE SIMULATOR MUST FAIL THE BROKEN CONFIG.** Run against `max_angular_accel: 1.20` it
has to reproduce the hunt — bad yaw economy, near-zero progress. **A simulator that cannot
reproduce the field failure has not earned the right to certify the fix**, and that run is
the falsifier, not a formality. It must be curve-faithful, not idealised: the dead zone
(≤10 → *exactly* zero, no creep), the floor snap (any sub-floor request that is executed
comes out at the floor rate), the measured `0.1344·duty − 0.213` inside the band, and the
walk band treated as hostile.

## Falsifier attempt 1 FAILED, and the re-registration for attempt 2

**Attempt 1 (empty world, all-clear scan) could not reproduce the hunt**: on the BROKEN
config the goal SUCCEEDED, yaw economy was 223 °/m (passing the <360 threshold against the
field's 4029), and only 3 rotation commands were issued against the field's 64. Cause: the
rig was scoped with no obstacles, so RPP never collides, never aborts, never reverses —
and the reversal limit cycle cannot occur. **A rig that cannot fail on the known-bad config
cannot certify the good one**, so the result was discarded rather than reported as a pass.

Attempt 2 raycasts mission 2's real saved map. **Expectations registered BEFORE running,
so they cannot be tuned to whatever appears:**

| signature | BROKEN config must show | FIXED config must show |
|---|---|---|
| goal outcome | ABORTED, or completed with terrible economy | SUCCEEDED |
| yaw economy | **> 1000 °/m** (field: 4029) | **< 360 °/m** |
| sub-floor rotation commands | **> 25 %** of pure-rotation commands (field: 35/64 ≈ 55 %) | **zero** |
| `out_wz` sign alternations | **> 40 /m** (field: ~130 /m) | **< 20 /m** |
| collision aborts + recoveries | **present** — this is the generator | may occur; must not hunt |

**If attempt 2 still fails to reproduce the hunt, that is a finding about the hunt's true
generator — supervisor latching, scan timing, odom noise — and we iterate on the model
rather than lowering the bar.** The bar is written down here first precisely so it cannot
drift.

## Why config-only rather than the policy redesign

The policy question — what *should* happen to a sub-floor request — remains real and
unanswered, and `refuse-and-zero` versus `reshape-as-arc` deserves the thought the first
draft tried to give it. But **if no layer ever emits a sub-floor rotation request, the
policy is not on the critical path today.** Config-only changes four numbers, is
derivable from `pivot_curve`, and can be guarded by a test that pins the relation. The
adapter redesign can then happen with arc-rate data in hand rather than under field
pressure.

## The guard this needs

One constant, one author: assert from `pivot_curve` that
`max_angular_accel * (1/controller_frequency) >= minimum_clean_rate(pivot_min_duty)` for
both the controller and the behaviour server — so if anyone lowers the acceleration limit,
or raises `pivot_min_duty`, or changes the controller frequency, the test names the
inequality rather than the robot discovering it on a floor.
