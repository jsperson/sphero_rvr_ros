# Tank-SI Mapping Gate (Before Lean Explore)

`explore.launch.py` and every autonomous Nav2 goal are **quarantined** after the
2026-08-01 speed-control incident. Do not launch the motion graph or send a goal
until Scott has run the attended floor check below and it reports `PASS`.

The corrected future command path is:

```text
/navigate_to_pose -> Nav2 -> /cmd_vel
  -> lidar_collision_stop_supervisor (sole /cmd_vel_motor publisher)
  -> sphero_rvr_driver
  -> RVRDriver.set_velocity(native_tank_si)
  -> drive_tank_si_units(left_mps, right_mps)
```

The dedicated `config/lean_rvr_tank_si.yaml` caps requested linear speed at
`0.05 m/s` and angular speed at `0.4 rad/s`. The deployed `config/rvr.yaml` is
unchanged. The unsafe RC-SI mode is rejected by `RVRDriver`.

## What this gate measures

`rvr_tank_si_mapping_validate` opens the normal `SerialTransport`, constructs
the real `RVRDriver` in `native_tank_si` mode, and refreshes motion exclusively
through `RVRDriver.set_velocity()`. It never calls a direct drive packet or raw
motor bypass. It reads onboard left/right encoder counts and converts them with
the same `4337.768 counts/m` calibration used by ROS odometry.

Each bounded trial has a 0.5 s warmup, a 2.0 s measured interval, an immediate
driver STOP, and a 0.75 s stationary check. It reports commanded versus measured
velocity and post-STOP travel for `0.03` and `0.05 m/s`. The process has a 30 s
overall timeout and rejects any requested speed above `0.05 m/s`.

Floor acceptance requires:

- the `0.05 m/s` measured average to be within ±20% (`0.04–0.06 m/s`);
- post-STOP encoder travel of at most `0.01 m` for every trial;
- final report `acceptance: PASS`.

A wheels-up result is only a packet/direction precheck and is never ground-speed
acceptance.

## Prepare the Pi

Build the focused PR, then make the rover UART exclusively available:

```bash
ssh sphero-pi-2
cd /home/jsperson/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select sphero_rvr_driver
source install/setup.bash

systemctl --user stop rvr-hierarchical-mission.service rvr-telemetry.service
sudo fuser /dev/ttyAMA0
```

`fuser` must print no owner. Do not start `explore.launch.py`, Nav2, the collision
supervisor, or another driver beside this exclusive validation harness.

## Default no-motion check

This exact command prints the bounded plan and exits without opening serial:

```bash
ros2 run sphero_rvr_driver rvr_tank_si_mapping_validate
```

Required: `motion` is `DISABLED` and `MOTION_SKIPPED` is printed.

## Optional wheels-up precheck

With Scott present, chassis securely supported, treads clear, and a hand at
power, run:

```bash
ros2 run sphero_rvr_driver rvr_tank_si_mapping_validate \
  --armed --attended --surface wheels-up
```

Stop immediately for wrong direction, vibration, or unexpected speed. The
expected final label is `BENCH_ONLY_NOT_GROUND_SPEED_ACCEPTANCE`; proceed only
if the behavior is calm and forward.

## Mandatory attended floor check

Place the rover on a clear, level, bounded floor with no stairs, ledges,
drop-offs, obstacles, or chair-mat ridge. Aim along at least 0.5 m of clear
travel. Scott keeps one hand at chassis power and runs exactly:

```bash
ros2 run sphero_rvr_driver rvr_tank_si_mapping_validate \
  --armed --attended --surface floor --floor-area-clear \
  --output /tmp/rvr-tank-si-mapping.json
```

Cut chassis power immediately for unsafe motion; do not wait for software. The
harness also sends STOP and disconnects on normal completion, failure, timeout,
or Ctrl-C.

Read the printed table-equivalent JSON for each trial's `commanded_mps`,
`measured_mps`, `relative_error`, and `post_stop_travel_m`. A nonzero exit or any
result other than `acceptance: PASS` fails the gate.

## Stop after the gate

After either outcome, verify the UART is released:

```bash
sudo fuser /dev/ttyAMA0
```

Do not run `explore.launch.py` and do not send a Nav2 goal in this procedure.
Even a mapping `PASS` only unlocks review of a separate, later attended
forward-goal step; it does not itself authorize autonomous motion.
