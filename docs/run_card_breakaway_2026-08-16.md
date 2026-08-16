# RUN CARD — measure the breakaway threshold

**Fifteen minutes. Scott present. First act with the robot, ahead of any architecture
work.** This is the number three safety constants are derived from and nobody has ever
measured it.

---

## 0. Why this is first

The RVR's motors have a **breakaway** threshold: below it the tracks grind instead of
turning. Everything downstream of that fact is currently folklore:

* `pivot_rate_rad_s: 0.9` — chosen to be *"above breakaway"*, value unmeasured
* `max_angular_rad_s: 0.4` — the supervisor's clamp, **below** it
* `rotate_to_heading_angular_vel: 0.4` — the Nav2 config, **below** it

On gauntlet mission 1 that gap killed the mission:
**41 consecutive commanded pure rotations at exactly 0.400 rad/s produced 0–1 mm of
motion.** The robot didn't move, the freeze classifier blamed an invisible obstacle,
planted marks, and the marks buried the rover's own cell
(`docs/autopsy_phantom_freeze_2026-08-16.md`).

**All we know is breakaway ∈ (0.4, 0.9].** Both the bespoke stack and the stock-middle
prototype need the real number, so this measurement is useful whichever way Scott rules
on the architecture.

---

## 1. THE CONSTRAINT THAT BITES — read before staging

**The supervisor clamps every angular command to 0.4 rad/s** (`collision_stop.yaml:156`,
`collision_stop.py:1048`). **You cannot measure above 0.4 through the normal path** — the
clamp silently rewrites the command and every step above 0.4 would return the same
answer.

**Raise the cap for the test, do not bypass the supervisor:**

```bash
ros2 param set /lidar_collision_stop_supervisor max_angular_rad_s 1.0
ros2 param get /lidar_collision_stop_supervisor max_angular_rad_s   # confirm 1.0
```

Keeping the supervisor in the loop means the collision gates, the ToF brake and the D39
hold all stay live during the sweep. **Bypassing it by publishing to `/cmd_vel_motor`
directly would remove every safety layer at once — do not.**

**Restore it afterwards** (`0.4`), or restart the stack, and confirm by reading the
param back. A raised cap left behind is a rover that turns faster than every constant in
the config assumes.

---

## 2. Safety envelope — non-negotiable

* **Scott present, hand on the power switch.** This test deliberately commands the rates
  documented to grind the motors; the rover was powered down twice by this in the past.
* **Rotation in place ONLY. Zero translation commands.** `linear.x` stays 0.0 throughout.
* **Open floor**, > 0.5 m clear all round — a tank drive rotating can walk slightly.
* **Battery ≥ 25%.**
* **Abort immediately** on: any translation, any grinding noise that does not resolve
  into rotation, any smell of hot motor, or a rover that walks more than ~5 cm.
* **Bounded bursts: 2 s per step, ≥ 3 s stopped between.** Sustained sub-breakaway
  commanding is the damaging case; short bursts with cooling gaps are not.
* **Do not launch the lidar.** It is not needed and its disc is one more thing to spin
  down. If it comes up with the stack anyway, `/stop_motor` before teardown.

---

## 3. Procedure

```bash
# on the Pi -- driver + supervisor only, no explorer, no nav2, no camera
ros2 launch sphero_rvr_driver explore.launch.py start_motion_stack:=true \
    start_explore:=false use_coverage_explorer:=false use_decisive_controller:=false

ros2 param set /lidar_collision_stop_supervisor max_angular_rad_s 1.0

# recorder, so the sweep is an artifact and not a memory
cd ~/ros2_ws/src/sphero_rvr_ros/diagnostics
python3 run_recorder.py 600 ~/breakaway_$(date +%Y%m%d_%H%M%S).csv
```

Then, for **w in 0.30 0.40 0.50 0.60 0.70 0.80 0.90**:

```bash
# 2 s of pure rotation, then stop and let it settle
timeout 2 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: W}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}"
sleep 3
```

**Record for each step:** commanded `w`, `odom_yaw_deg` before and after, and whether it
was audibly grinding, rotating, or silent. The CSV captures the first two; the third is
Scott's ear and it matters — *grinding* and *not moving* are different failures.

---

## 4. The reading

**Breakaway is the lowest `w` that produces sustained rotation** — not the lowest that
produces *any* movement. A step that twitches and stops is below it.

Expect a sharp knee rather than a gradient. Report as:

```
w=0.30  ->  __ deg in 2 s   [grinding / silent / rotating]
w=0.40  ->  __ deg          (expected ~0: this is mission 1's measured dead value)
...
w=0.90  ->  __ deg          (expected clean rotation)

BREAKAWAY = ___ rad/s        (lowest w with sustained rotation)
```

**Sanity check against the field:** 0.40 must come back at or near zero. If it rotates
cleanly, then mission 1's 41 dead commands had a *different* cause and the autopsy needs
reopening — say so rather than explaining it away.

---

## 5. What the number feeds, immediately

| constant | now | becomes |
|---|---|---|
| `max_angular_rad_s` (supervisor clamp) | 0.4 | **≥ breakaway**, or the supervisor keeps commanding stalls |
| `pivot_rate_rad_s` (decisive controller) | 0.9 | breakaway × ~1.25, derived rather than asserted |
| `rotate_to_heading_angular_vel` (stock prototype) | 0.9 provisional | breakaway × ~1.25 |
| `min_rotational_vel` (stock Spin behaviour) | 0.9 provisional | breakaway |

Every **MEASURE-FIRST** marker in `config/lean_nav2_stock.yaml` is waiting on this one
number, and `tests/test_stock_middle_config.py` refuses to let that config be considered
flyable until it exists.

---

## 6. Opportunistic, only if zero-risk

If the IMU is already up, record gyro alongside odom. Same session answers D32's
discriminator question (wheel-odom yaw vs measured yaw under a known command) at no
extra robot time. **Skip it if it needs any extra bringup** — this card's job is one
number.

---

## 7. After

Restore `max_angular_rad_s` to `0.4` **and read it back**. Stop the lidar motor by
service if it came up. Processes by explicit PID. Confirm `ros2 node list` is empty, then
`ros2 daemon stop`.

Archive the CSV to `03_validation/breakaway_2026-08-16/` with a README naming the
measured value, the binary, and the seven readings.
