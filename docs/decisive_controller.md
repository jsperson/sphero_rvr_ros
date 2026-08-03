# Decisive controller

A Nav2 `FollowPath` controller for the RVR tuned to *how this drivetrain actually
moves*, not to Nav2's careful default style.

## Why

The RVR tank drive is good at driving straight and arcing at a real speed, and
**bad at slow, precise moves** — below a "breakaway" speed the motors don't turn,
they grind in place. Nav2's default controller (RPP + RotationShim) tracks the
planned path *exactly*, correcting its heading continuously and stopping to pivot
in place to re-face the path. On this drivetrain that means constant slow in-place
pivots — which grind the motors (observed repeatedly; the rover had to be powered
down twice). There was always plenty of clearance; the failure was never "about to
hit something," it was "moving too slowly / turning when it didn't need to."

## The policy

Only ever turn for a reason, and prefer the motion that keeps the motors rolling:

- **Straight** when roughly aligned with the target — a heading **deadband**; do
  not correct a heading error that does not need correcting.
- **Arc** (turn while rolling) for a moderate course change — both tracks stay
  above breakaway, so it does not grind.
- **Pivot in place** only for a large heading change, and do it **decisively**
  (a rate above breakaway) — never a slow creep.
- Stop when within tolerance of the path end.

## Pieces

- `sphero_rvr_core/decisive_control.py` — pure, ROS-free decision core:
  `select_target_point` (lookahead on the path), `heading_error_to_point`,
  `compute_drive_command` (the straight/arc/pivot decision). Unit-tested
  (`tests/test_decisive_control.py`).
- `sphero_rvr_driver/decisive_controller_node.py` — a Nav2 `FollowPath` action
  server wrapping the core. Publishes `/cmd_vel` (through the collision-stop
  supervisor like every motion source). Not lifecycle-managed.

## Enable it (opt-in; RPP is the default)

```bash
ros2 launch sphero_rvr_driver explore.launch.py \
    start_motion_stack:=true start_explore:=true enable_imu_fusion:=true \
    use_decisive_controller:=true
```

When on, the launch runs `decisive_controller` instead of `controller_server` and
drops `controller_server` from the lifecycle manager.

## Tunables (node params, defaults in the config dataclass)

`cruise_speed_mps` 0.20, `heading_deadband_rad` 0.17 (~10°),
`pivot_threshold_rad` 1.22 (~70°), `arc_gain` 1.2, `max_arc_angular_rad_s` 0.8,
`pivot_rate_rad_s` 0.9, `goal_tolerance_m` 0.10, `lookahead_m` 0.4. All above the
motor breakaway on purpose — the controller must never command a below-breakaway
speed.

## Status / validation

Built, unit-tested (357 suite pass), and build- + launch-validated on the Pi.
**UNTESTED on hardware.** One clean run validates it. Watch specifically for
grinding: if it only ever drives / arcs / does rare decisive pivots and still
grinds, that is the drivetrain's answer, not the controller's.
