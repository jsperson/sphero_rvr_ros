# RUN CARD — measure the pivot duty that actually moves this robot

**Twenty minutes. Scott present. First act with the robot, ahead of any architecture
work.**

> **⚠ THIS CARD WAS REWRITTEN AFTER REVIEW. Its first version was defective in three
> independent ways and would have produced a confident wrong number.** What it got wrong
> is recorded in §8, because the failure mode — a measurement procedure that manufactures
> its own answer — is the same class as the defect it is trying to measure.

---

## 0. What we are measuring, and why it is a DUTY and not a rate

An in-place pivot on this robot does **not** execute the commanded angular rate. For
`|linear| < 0.005` and `|angular| > 0`, `driver.py:708` takes a **closed-loop pivot
controller** that uses the command only for its **sign**, then drives toward a fixed
internal target of 1.3 rad/s by ramping a duty:

```python
sign  = 1.0 if velocity.angular_rad_s > 0.0 else -1.0
error = self._pivot_target_rate_rad_s - abs(self._measured_yaw_rate)   # 1.3, always
self._pivot_duty_cmd += self._pivot_duty_gain * error                  # gain 0.6
self._pivot_duty_cmd = min(pivot_max_duty, max(pivot_min_duty, ...))   # ceiling 32
```

**Commanded 0.4 and commanded 0.9 are the same command to this controller.** Sweeping
commanded rate would return one answer seven times and "prove" a drivetrain that cannot
turn.

The open question is the ceiling. `pivot_max_duty` is **32** on the ±127 scale of
`drive_tank_normalized`; the only moving-duty figure in the repo is a comment about the
**raw-motor** branch on a 0–255 scale ("≤128 does not move at all, 140–160 breaks away").
**32/127 ≈ 64/255 — about half of what that comment says does nothing.** Whether the two
paths are equivalent in torque is an inference, and this test exists to replace it with a
measurement.

**THE NUMBER WE WANT: the lowest `drive_tank_normalized` duty that produces sustained
in-place rotation.**

> **CORRECTION (2026-08-16, found while building the tool): there are TWO deployed
> ceilings, and the mission ran the higher one.** `32` above is the `RVRNodeConfig`
> dataclass default, which `config/rvr.yaml` never overrides. The gauntlet launches
> through `explore.launch.py`, whose default `rvr_params_file` is
> `config/lean_rvr_tank_si.yaml`: **`pivot_min_duty 28`, `pivot_max_duty 45`,
> `pivot_duty_gain 1.0`**. So the ceiling under test is **45/±127 ≈ 90/255**, about 70%
> of the documented no-move duty rather than half of it. Both ceilings are still below
> 128, so the verdict logic in §3 is unchanged — but the sweep must span **23, 28, 32
> and 45**, and it does. See the second correction block in the autopsy.

---

## 1. Safety envelope — non-negotiable

* **Scott present, hand on the power switch.** This test deliberately commands duties in
  and above the range documented to grind; the rover has been powered down twice by
  in-place grinding.
* **Rotation in place only. `linear` stays 0.0.**
* **Open floor, > 0.5 m clear all round** — a tank drive rotating can walk.
* **Battery ≥ 25%** (duty behaviour is voltage-dependent; record the level with the
  result, because this number is only valid near the battery state it was taken at).
* **Bounded bursts: 2 s per step, ≥ 3 s stopped between.** Sustained sub-moving duty is
  the damaging case.
* **Abort on:** any translation over ~5 cm, grinding that does not resolve into rotation
  within the burst, or any hot-motor smell.
* **Do not launch the lidar or the explorer.** Driver only.

---

## 2. Procedure — a DIRECT DUTY SWEEP, bypassing the pivot controller's ceiling

The pivot controller cannot be swept from outside (it discards the command), and its
ceiling is the thing under test. So drive the duty directly through the driver's own
command builder, with the driver node **not running** — one process, one authority, no
clamps in between.

```bash
# on the Pi. NOTHING ELSE RUNNING -- the tool checks, but confirm `ros2 node list`
# is empty anyway. It REFUSES without --arm; that is the point of --arm.
cd ~/ros2_ws/src/sphero_rvr_ros
python3 diagnostics/pivot_duty_sweep.py --arm          # default ladder spans 12..100
```

The default ladder is `12 16 20 23 28 32 36 40 45 50 56 62 70 76 84 92 100`: below both
production floors, through **all four** deployed pivot constants (23/28/32/45), and past
the ±127 equivalents of the documented raw-motor breakaway region (140→70, 160→80). To
override: `--duties 20,28,32,40,50,60,70,80,90,100 --burst-s 2.0 --settle-s 3.0`. The
tool refuses a ladder that starts at or above the production floor or stops below 80,
because either one would produce a table that cannot answer the question.

`diagnostics/pivot_duty_sweep.py` **is written and offline-tested** (`tests/
test_pivot_duty_sweep.py`, 47 tests, mutation-checked). What it does:

1. opens the serial transport directly and sends `drive_tank_normalized(seq, -d, +d)`
   at the control period for `burst-s`, then `(0,0)` — one process, one authority, and
   the driver's control loop is never given a velocity so it never sends anything;
2. **reads yaw from the IMU gyro** (`enable_imu_streaming` in the core driver — no ROS
   needed) and reports **achieved rad/s** per duty. The gyro measures BODY rotation,
   which is the question; the production loop regulates on WHEEL encoders, which is a
   different question and the one D32 has never had answered. So encoder counts are
   polled at each burst boundary too, and **wheel-vs-body disagreement is printed as a
   result** — a grinding wheel that slips reports rotation the chassis did not make.
   The gyro path has never run in production (`publish_imu` is false everywhere), so the
   tool proves the stream is alive before commanding anything and prints **INSTRUMENT
   DEAD** and refuses rather than recording a table of zeros;
3. refuses to run if anything else holds `/dev/ttyAMA0` (reads `/proc/*/fd` and names
   the pid), if `--arm` is absent, or if the battery is under 25%;
4. stops on the first duty that produces sustained rotation **and one step beyond**, so
   we get the knee and its confirmation without climbing into the bog;
5. aborts mid-run on a firmware motor fault, on the gyro stream going quiet, or on more
   than 5 cm of encoder-measured translation — a pivot that walks is an abort, not a
   data point;
6. writes a CSV to `~/breakaway_<stamp>.csv` with the whole run configuration in the
   header (battery, ladder, bursts, git SHA, gyro noise floor), because a duty number
   without the conditions it was taken under is not a measurement;
7. counts the motor packets the driver actually wrote. **An all-zero sweep cannot tell a
   dead drivetrain from a dead command path**, so the verdict says which one the write
   counter shows.

**Why not through ROS:** `rvr_node` and the supervisor each clamp `max_angular_rad_s` to
0.4, both read their config **once at `__init__`** (`rvr_node.py:155`,
`collision_stop_node.py:190`) with **no** `add_on_set_parameters_callback` anywhere — so
`ros2 param set` changes the parameter server while the live clamp keeps the cached
value, and `ros2 param get` cheerfully confirms the change that did not take effect. Two
authorities for one constant, in the measurement procedure itself.

---

## 3. Reading it

The tool prints this; nothing has to be transcribed by hand.

```
duty  raw255  achieved rad/s (gyro)  peak    n   pkts  encoders rad/s  translation
  23      46                  0.000   ...                              <- pivot_min_duty (defaults)
  28      56                  0.000   ...                              <- pivot_min_duty (missions)
  32      64                  0.000   ...                              <- ceiling, defaults
  45      90                  0.000   ...                              <- CEILING MISSIONS RAN
  ...
VERDICT: MOVING_DUTY_FOUND | SWEEP_INVALID | NO_ROTATION_IN_RANGE
BATTERY = ___%
```

**The verdict this test delivers,** evaluated against **both** deployed ceilings and
printed for each: if moving-duty is **above a ceiling**, the closed-loop pivot on that
config is structurally incapable of turning this robot, every in-place pivot it ran was
a no-op, and the historical freeze record from it is largely phantom. If moving-duty is
**at or below the ceiling**, that ceiling is fine and the mission-1 episodes had another
cause — say so plainly and reopen the autopsy. The mission ceiling is **45**; the 32 is
what the non-mission launches use.

**Behavioural verification, not parameter confirmation:** the only evidence that a duty
took effect is a **measurably different achieved yaw rate**. If two adjacent duties give
the same achieved rate, something upstream is clamping and the sweep is invalid — stop
and find it before trusting any reading.

---

## 4. What the number feeds

| constant | now | becomes |
|---|---|---|
| `pivot_max_duty` | **45 (missions)** / 32 (defaults) | **above moving-duty with margin** — the primary fix, in BOTH places |
| `pivot_min_duty` | **28 (missions)** / 23 (defaults) | at or just below moving-duty, so the ramp starts useful |
| `max_angular_rad_s` (supervisor **and** `rvr_node`) | 0.4 both | re-derived for the non-pivot paths |
| `rotate_to_heading_angular_vel`, `min_rotational_vel` (stock prototype) | 0.9 provisional | derived from the achieved-rate curve |

Every **MEASURE-FIRST** marker in `config/lean_nav2_stock.yaml` waits on this.

---

## 5. D32, no longer opportunistic — it is the instrument

This card originally listed "record IMU gyro alongside odom yaw" as a nice-to-have.
It is now the **primary** measurement, and the encoders are the cross-check, because
the question is whether the BODY turned and encoders only know what the WHEELS did.
The tool prints the disagreement either way, so D32's wheel-odom-vs-measured question
gets answered by this run whatever the duty result is.

---

## 6. After

Processes by explicit PID. `ros2 node list` empty, `ros2 daemon stop`. Archive the CSV to
`03_validation/breakaway_2026-08-16/` with a README naming the moving duty, the battery
level, the binary, and every reading.

---

## 7. Offline prerequisite — DONE

`diagnostics/pivot_duty_sweep.py` and `tests/test_pivot_duty_sweep.py` were written and
tested offline on 2026-08-16, **against a fake driver, with no hardware involved**. The
suite runs 47 tests over the refusals, the instrument-death paths, the ladder's bounds
and early stop, the validity verdict in both directions, the scale conversion, and the
CSV header; 23 deliberate mutations of the tool were each confirmed to turn the suite
red. A test that needs debugging while a human stands over a robot wastes the expensive
resource in the room, which is Scott.

**The tool has never been run against the rover.** First contact is Scott's staging.

---

## 8. What the first version of this card got wrong

Kept deliberately: a procedure that manufactures its own answer is the same failure class
as the defects being investigated, and this one was caught by review rather than by the
floor.

1. **It swept commanded angular rate.** The pivot path discards the magnitude, so all
   seven steps would have returned the same result.
2. **It raised only the supervisor's clamp.** `rvr_node` holds a second
   `max_angular_rad_s: 0.4` at the driver's door; everything above 0.4 would have been
   re-clamped there.
3. **It verified with `ros2 param set` + `param get`.** Both nodes cache config at
   `__init__` and register no parameter callback, so the live clamp would never have
   moved and the confirmation would have read the wrong authority.

Together those three would have produced a clean-looking dataset showing a drivetrain
dead above 0.3 rad/s — **a wrong autopsy, manufactured by its own procedure, and exactly
what the original card's falsifier section was written to fear.** The falsifier is what
made it findable; that part was right and is kept.
