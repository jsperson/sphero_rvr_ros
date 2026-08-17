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

```
cycles to ramp from 0 to the 3.55 rad/s config value = 3.55 / 0.06 = 59 cycles
                                                     = 59 * 0.05  = 2.96 SECONDS
```

**So RPP spends ~3 seconds walking up through the entire sub-floor band**, emitting ~59
consecutive commands *below* what this drivetrain can execute. Every one of them is raised
to floor duty and leaves at **3.55 rad/s**. RPP believes it is accelerating gently from
rest; the robot is snapping at full rate from the first command. Observed `cmd_wz` values
— 0.06, 0.11, 0.21, 0.54, 0.92, 1.03, 1.21, 1.34, 1.39, 1.46 — are that ramp, and **35 of
64 pivot commands were below the floor.**

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
against the real mechanism — a ~59-cycle acceleration ramp from rest — the options score
very differently:

**(a) refuse-and-zero (deadband).** Sub-floor requests command nothing. Against a ramp,
this is **actively broken**: RPP's first ~59 commands are sub-floor, so the robot would
refuse *all* of them, `curr_speed.angular.z` would stay 0, and the next command would be
clamped to 0.06 again. **The ramp never escapes its own first step — rotation never starts
at all, permanently.** This is a deadlock, not a stall risk. It was my first draft's
preferred option, and it would have hung the robot on its first turn.

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
