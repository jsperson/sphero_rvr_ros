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
# on the Pi. NOTHING ELSE RUNNING -- confirm `ros2 node list` is empty first.
cd ~/ros2_ws/src/sphero_rvr_ros
python3 diagnostics/pivot_duty_sweep.py --duties 20,28,32,40,50,60,70,80,90,100 \
                                        --burst-s 2.0 --settle-s 3.0
```

`diagnostics/pivot_duty_sweep.py` **does not exist yet — write it first** (offline, no
robot). It must:

1. open the serial transport directly and send `drive_tank_normalized(seq, -d, +d)`
   at the control period for `burst-s`, then `(0,0)`;
2. read yaw from the **wheel-encoder stream** and report **achieved rad/s** per duty --
   encoders, because that is the same source the production pivot loop regulates on
   (`rvr_node.py:563`), so the sweep measures what the controller sees. **This is the
   bulk of the work and the reason this is not a small script:** it needs the sensor
   stream configured, the streaming packets decoded, and the odom tracker driven, all
   without the ROS node. Budget for that honestly; do not start it on a tired context.
   (An IMU cross-check is worth recording alongside, since encoders measure WHEEL
   rotation and a grinding wheel that slips would under-report BODY rotation.);
3. refuse to run if any ROS node holds `/dev/ttyAMA0`;
4. stop on the first duty that produces sustained rotation **and one step beyond**, so
   we get the knee and its confirmation without climbing into the bog;
5. write a CSV to `~/breakaway_<stamp>.csv`.

**Why not through ROS:** `rvr_node` and the supervisor each clamp `max_angular_rad_s` to
0.4, both read their config **once at `__init__`** (`rvr_node.py:155`,
`collision_stop_node.py:190`) with **no** `add_on_set_parameters_callback` anywhere — so
`ros2 param set` changes the parameter server while the live clamp keeps the cached
value, and `ros2 param get` cheerfully confirms the change that did not take effect. Two
authorities for one constant, in the measurement procedure itself.

---

## 3. Reading it

```
duty  achieved rad/s   note
  20      ____         [silent / grinding / rotating]
  28      ____         <- pivot_min_duty
  32      ____         <- pivot_max_duty, the CEILING under test
  40      ____
  ...
MOVING DUTY = ___      (lowest duty with sustained rotation)
BATTERY = ___%
```

**The verdict this test delivers:** if moving-duty **> 32**, then the closed-loop pivot
controller is structurally incapable of turning this robot, every in-place pivot in
production has been a no-op, and the historical freeze record is largely phantom.
If moving-duty **≤ 32**, the ceiling is fine and the mission-1 episodes had another
cause — say so plainly and reopen the autopsy.

**Behavioural verification, not parameter confirmation:** the only evidence that a duty
took effect is a **measurably different achieved yaw rate**. If two adjacent duties give
the same achieved rate, something upstream is clamping and the sweep is invalid — stop
and find it before trusting any reading.

---

## 4. What the number feeds

| constant | now | becomes |
|---|---|---|
| `pivot_max_duty` | 32 | **above moving-duty with margin** — the primary fix |
| `pivot_min_duty` | 23 / 28 | at or just below moving-duty, so the ramp starts useful |
| `max_angular_rad_s` (supervisor **and** `rvr_node`) | 0.4 both | re-derived for the non-pivot paths |
| `rotate_to_heading_angular_vel`, `min_rotational_vel` (stock prototype) | 0.9 provisional | derived from the achieved-rate curve |

Every **MEASURE-FIRST** marker in `config/lean_nav2_stock.yaml` waits on this.

---

## 5. Opportunistic, only if zero-risk

Record IMU gyro alongside odom yaw — same session answers D32's wheel-odom-vs-measured
question at no extra robot time. **Skip if it needs extra bringup.**

---

## 6. After

Processes by explicit PID. `ros2 node list` empty, `ros2 daemon stop`. Archive the CSV to
`03_validation/breakaway_2026-08-16/` with a README naming the moving duty, the battery
level, the binary, and every reading.

---

## 7. Offline prerequisite

**Write and unit-test `diagnostics/pivot_duty_sweep.py` before Scott stages.** A test
that needs debugging while a human stands over a robot wastes the expensive resource in
the room, which is Scott.

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
