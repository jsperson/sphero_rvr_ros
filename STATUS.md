# Sphero RVR ROS Project Status

Updated: 2026-06-09T21:50:45Z

## Current repo state

- Branch: `main`
- Current commit: `659c445 fix: use validated raw motor off for RVR stop paths`
- CI: passed for commit `659c445`
- Local test suite at handoff: `81 passed`
- Target Pi workspace: `/home/jsperson/ros2_ws/src/sphero_rvr_ros`
- Target Pi install: pulled/rebuilt with `colcon build --symlink-install --packages-select sphero_rvr_driver`
- Convenience wrapper installed: `/home/jsperson/.local/bin/rvr-console`

## Target hardware/runtime

- Target host: `sphero-pi-2`
- OS/runtime: Ubuntu Server 24.04 LTS, ROS 2 Jazzy
- UART: `/dev/ttyAMA0`, 115200 baud
- ROS workspace: `/home/jsperson/ros2_ws`
- Driver package: `sphero_rvr_driver`

## Safety-critical context

The RVR showed unsafe forward motor activity while testing `rvr-console`, especially around `/exit` and repeated keyboard commands. Treat live keyboard driving as requiring a suspended/restrained robot until revalidated.

Current safety rule for future sessions:

- Before any live motor-capable action, explicitly warn: `WARNING: this can start the RVR motors`.
- Do not run `rvr-console`, `rvr_tui`, ROS driver launches, `/cmd_vel`, `/stop`, or any live driver command without explicit user approval after that warning.
- Build/install/unit-test/fake-client checks are okay without motor approval because they do not talk to the robot.

## Root fix shipped after unsafe behavior

The suspected root cause was the shutdown/stop path using unvalidated drive commands:

- Old `/stop`: `drive_with_heading(speed=0, heading=0, flags=0)`
- Old `emergency_stop`: fake/unvalidated internal drive command
- Old wrapper/client cleanup: automatically called ROS `/stop` on exit

Current behavior in commit `659c445`:

- `RVRCommands.stop()` sends validated `raw_motors(0, 0, 0, 0)`.
- `RVRCommands.emergency_stop()` sends validated `raw_motors(0, 0, 0, 0)` and software-gates future velocity.
- `RVRDriver.clear_emergency_stop()` is software-only and no longer sends the old unvalidated fake hardware packet.
- `rvr-console` shell cleanup no longer calls ROS `/stop`.
- `RVRROSClient.close()` no longer calls ROS `/stop`.
- TUI `/exit`/safe-stop publishes zero `/cmd_vel` instead of calling `/stop` directly.

Pi-side non-live verification after install:

```text
stop 1 00000000
estop 1 00000000
```

Command ID `1` is `CID_RAW_MOTORS`; payload `00000000` is both tracks off.

## Current TUI behavior

Start command on Pi:

```bash
rvr-console
```

Inside TUI:

```text
/battery
/status
/arm
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

## Important caveat

The root fix has been unit-tested and installed, but it has **not** yet been live revalidated with the physical RVR after the runaway report. First live retest should be suspended/restrained and minimal:

1. Start `rvr-console`.
2. Run `/battery` and `/status` only.
3. Run `/arm`.
4. Tap a single direction key briefly.
5. Confirm no queued/stale motion.
6. Test `/disarm`, space, `/exit`, and physical behavior carefully.

If anything odd happens, physically power off/restrain first; do not rely solely on software stop.

## Suggested next engineering work

1. Add driver-level velocity coalescing so repeated `/cmd_vel` updates cannot queue stale nonzero motor packets.
2. Add a mock ROS/TUI integration test that simulates repeated direction commands followed by `/exit` and asserts the final command path is zero velocity/raw motor off.
3. Consider publishing continuous hold-to-drive at a fixed cadence instead of one-shot direction key packets.
4. Add a `--dry-run`/fake-driver mode for `rvr-console` so TUI behavior can be tested interactively without hardware.
5. After suspended validation, document the exact safe floor-test parameters.
