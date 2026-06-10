# Sphero RVR ROS Project Status

Updated: 2026-06-09T22:52:00Z

## Current repo state

- Branch: `main`
- Latest deployed code baseline before this status update: `a344718 Revert "tune: tolerate slower RVR turn key repeat"`
- Local `HEAD` matches `origin/main` after this status update.
- `sphero-pi-2` was verified on the deployed code baseline `a344718` before this docs-only update; a follow-up SSH pull for this status file timed out.
- Local test suite at handoff: `86 passed`.
- Target Pi workspace: `/home/jsperson/ros2_ws/src/sphero_rvr_ros`
- Target Pi install: pulled/rebuilt with `colcon build --symlink-install --packages-select sphero_rvr_driver`
- Background driver running through `ros2 launch sphero_rvr_driver rvr.launch.py`.
- Convenience wrapper installed: `/home/jsperson/.local/bin/rvr-console`

## Target hardware/runtime

- Target host: `sphero-pi-2`
- OS/runtime: Ubuntu Server 24.04 LTS, ROS 2 Jazzy
- UART: `/dev/ttyAMA0`, 115200 baud
- ROS workspace: `/home/jsperson/ros2_ws`
- Driver package: `sphero_rvr_driver`

## Safety-critical context

Earlier testing showed unsafe forward motor activity around `rvr-console` `/exit` and repeated keyboard commands. The root stop-path fix is still in place:

- `RVRCommands.stop()` sends validated `raw_motors(0, 0, 0, 0)`.
- `RVRCommands.emergency_stop()` sends validated `raw_motors(0, 0, 0, 0)` and software-gates future velocity.
- `RVRDriver.clear_emergency_stop()` is software-only.
- `rvr-console` shell cleanup and `RVRROSClient.close()` no longer call ROS `/stop`.
- TUI `/exit`/safe-stop publishes zero `/cmd_vel`.

Current safety rule for future sessions:

- Before any live motor-capable action, explicitly warn: `WARNING: this can start the RVR motors`.
- Do not run `rvr-console`, `rvr_tui`, ROS driver launches, `/cmd_vel`, `/stop`, or any live driver command without explicit user approval after that warning.
- Build/install/unit-test/fake-client checks are okay without motor approval because they do not talk to the robot.

## Current deployed control behavior

Driver velocity scaling and turn direction have been tuned from live floor testing:

- Positive angular commands now map to physical left turns correctly.
- `max_raw_motor_duty` is `160` in `config/rvr.yaml` and `RVRNodeConfig`.
- Max turn payloads at duty cap:
  - left: `02a001a0`
  - right: `01a002a0`
- TUI timing constants:
  - `KEY_STOP_SECONDS = 0.30`
  - `TURN_KEY_STOP_SECONDS = 0.09`
  - `TURN_HOLD_DETECT_SECONDS = 0.15`
  - `KEY_REPEAT_SECONDS = 0.10`

Pure turn behavior:

- First left/right tap is a short nudge: one turn command, then zero after `0.09s`.
- If the same turn key repeats within `0.15s`, the TUI treats it as a held turn and internally republishes at `0.10s` cadence until the normal `0.30s` timeout.
- Forward/reverse still use the normal `0.30s` timeout and internal repeat.

## Live floor-test findings

Best current version is commit `a344718`.

Observed behavior on the physical RVR:

- Original turn direction was reversed; fixed by changing the tank mix to `left = linear - angular`, `right = linear + angular`.
- Suspended turns worked at lower duty, but on the floor the RVR needed much more torque because skid-steer turning must overcome tread/floor friction.
- Raising `max_raw_motor_duty` from `64` to `96` improved turns but was still weak.
- Raising `max_raw_motor_duty` to `160` gave usable floor-turn authority.
- `/speed` affects forward/reverse only; left/right uses `/turn`.
- Useful turn range during testing:
  - `/turn 0.3` did not reliably turn.
  - `/turn 0.4` turned too fast.
  - `/turn 0.35` was the main working test value.
- A long turn sustain timeout (`0.75s`) caused one tap to pivot about 90 degrees, which was too coarse.
- A short turn nudge (`0.09s`) gives good one-click precision.
- Attempts to make held turns more tolerant by extending held-turn detect/stop windows made continuous turning too fast.
- Current best tradeoff: tap precision is good; continuous turns are still not fully reliable and may bog down.

## Current operator commands

Start TUI on the Pi:

```bash
rvr-console
```

Inside TUI:

```text
/battery
/status
/arm
/speed 0.2
/turn 0.35
/disarm
/stop
/estop
/clear-estop
/exit
```

Keyboard controls after `/arm`:

```text
↑ / w      forward
↓ / s      reverse
← / a      turn left
→ / d      turn right
space      stop
q          quit
```

Logs:

```bash
tail -100 ~/.local/state/sphero_rvr/rvr-console.log
tail -100 ~/.local/state/sphero_rvr/rvr-driver.log
```

## Suggested next engineering work

1. Keep the current tap behavior as baseline; do not extend the one-tap timeout again without an explicit nudge/hold split.
2. Add a dedicated turn profile instead of tuning only timeouts:
   - tap: short high-duty nudge
   - hold: lower sustained duty or cadence-controlled turn
   - optional breakaway kick for 100-200ms, then lower sustain duty
3. Add `/nudge-turn` or TUI mode settings if manual tuning remains necessary.
4. Add a `--dry-run`/fake-driver mode for `rvr-console` so TUI timing behavior can be tested interactively without hardware.
5. Add driver-level velocity coalescing so repeated `/cmd_vel` updates cannot queue stale nonzero motor packets.
6. Add a mock ROS/TUI integration test that simulates repeated direction commands followed by `/exit` and asserts the final command path is zero velocity/raw motor off.
