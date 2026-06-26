# RVR control interface plan

Source-of-truth spec for extending `rvr-console` / `rvr_tui` into a safer ROS control interface for RVR lidar mapping.

This interface is intentionally conservative: the default path is lidar/SLAM inspection with no RVR driver and no `/cmd_vel` exposure. Anything that can start the motors must be visually obvious, explicitly confirmed, logged, and recoverable by STOP/ESTOP.

## Goals and non-goals

### Goals

- Make the TUI the supervised operator console for manual lidar mapping.
- Keep `ros2 launch sphero_rvr_driver mapping.launch.py` safe by default with `start_rvr:=false`.
- Give the operator a single status pane that answers: devices present, active launch profile, mapping mode, motor state, odometry calibration, and last STOP/ESTOP event.
- Prefer calibrated fixed nudges over joystick/free-drive while odometry and SLAM are still being validated.
- Provide a dry-run/fake-driver path that exercises the UI, state machine, and command gating without RVR hardware.
- Make the interface easy to operate under field conditions: menus and help should be plentiful, commands should be discoverable, and the operator should not have to memorize obscure syntax.
- Make startup easy: one documented operator entrypoint should start the console and guide launch choices instead of requiring a chain of scripts or hand-run setup commands.

### Non-goals

- No autonomy or Nav2 goal driving in this phase.
- No arbitrary user-provided `/cmd_vel` publishing from the TUI.
- No raw motor commands, firmware/admin commands, factory/reset flows, or calibration writes from the TUI.
- No hidden driver startup from a motor-capable command. If the RVR driver is not running, the UI must say so and require the launch command below.

## Safety language

Every UI path that can start the live RVR driver or publish nonzero `/cmd_vel` must display this exact warning before the operator confirms it:

```text
WARNING: this can start the RVR motors
```

The warning must be shown in the status/history pane and included in the log. The command must not proceed until the operator enters the exact confirming slash command named in this document. Abbreviations, aliases, and implicit confirmation are not allowed for motor-capable actions.

Hard safety invariants:

- Safe inspection modes (`idle`, `lidar-only`, and `dry-run`) must not construct or publish through a live ROS `/cmd_vel` publisher. Showing that `/cmd_vel` exists elsewhere in the ROS graph is diagnostic context only; it is not permission for the TUI to publish.
- `rvr-console` must not accept a startup flag or environment variable that skips the in-UI motor warning/confirmation flow. The only live motor-capable launch path is the exact `/mapping full confirm` command below.
- Dry-run may simulate motor-capable state transitions for tests, but the simulated client must be visibly fake and must not launch ROS, open hardware devices, or publish to any live ROS topic.
- If an implementation adds any new path that can start the live driver, create a live `/cmd_vel` publisher, or publish nonzero velocity, this document must be updated first and the path must be listed in the motor-capable command table.

## Runtime modes

The UI tracks two independent modes:

| Field | Values | Meaning |
|---|---|---|
| `hardware_mode` | `live`, `dry-run` | Whether ROS actions can reach real hardware. `dry-run` never opens `/dev/ttyAMA0`, never launches the live driver, and never publishes to a real `/cmd_vel`. |
| `mapping_mode` | `idle`, `lidar-only`, `motor-capable`, `stopping`, `failed-launch` | The active mapping launch state. |
| `motor_armed` | `false`, `true` | Whether keyboard/nudge commands are allowed to publish nonzero velocity. |
| `estop_inhibited` | `false`, `true` | Persistent software inhibit after ESTOP. While true, no nonzero velocity may be published even if `motor_armed=true`. |

`estop_inhibited=true` dominates every other state. Clearing ESTOP is deliberately separate from arming; `/clear-estop confirm` may clear the inhibit, but it must leave `motor_armed=false`.

`hardware_mode=dry-run` dominates launch and publishing behavior: motor-capable commands may move the fake state machine for parser/UI tests, but they must not create live ROS publishers, start `mapping.launch.py`, or touch `/dev/ttyAMA0`/`/dev/rplidar`.

## Status pane requirements

The top pane must be visible without opening a modal and must refresh at least once per second while the TUI is running.

Required fields:

```text
RVR Control Console
Hardware mode: live|dry-run
RVR driver: present|missing    /cmd_vel: available|not-exposed (publishers=<count>)
Battery: <percent|waiting> / <voltage|waiting> (<fresh|stale|waiting> age)
Odom: <fresh|stale|waiting> pose=(x, y, yaw=<rad>) distance=<meters>
Scan: <fresh|stale|waiting> ranges=<count> valid=<count> min=<meters> max=<meters>
Services: /stop ok|missing  /estop ok|missing  /clear_estop ok|missing
TF: odom->base_link ok|missing|waiting  base_link->laser ok|missing|waiting  map->odom ok|missing|waiting
Armed: false|true    Estop: false|true
Speed: <m/s>    Turn: <rad/s>
Diagnostics: <latest diagnostic message>
```

Notes:

- Future status-pane work should add explicit `/dev/rplidar` and `/dev/ttyAMA0` device checks plus the live `odom_counts_per_meter` config value. The current shipped pane reports ROS graph/sensor readiness without opening those hardware devices directly.
- `Launch profile`, shown in `/status` and `/mapping status`, describes what the TUI launched or attached to:
  - `none`: no managed launch.
  - `lidar`: `ros2 launch sphero_rvr_driver lidar.launch.py`.
  - `mapping-lidar`: `ros2 launch sphero_rvr_driver mapping.launch.py start_rvr:=false`.
  - `mapping-motor`: `ros2 launch sphero_rvr_driver mapping.launch.py start_rvr:=true`.
  - `dry-run`: fake graph/client only.
- `ROS graph /cmd_vel exposed` must be shown as dangerous context, not as permission to drive.
- `ROS graph /cmd_vel exposed` means a topic exists somewhere in the graph; it must not become `exposed` merely because the TUI created a publisher in a safe mode. Safe modes should not create that publisher at all.
- `Last STOP/ESTOP` must update for keyboard stop, slash `/stop`, slash `/estop`, timeout zero-publish, exit cleanup zero-publish, and launch-stop cleanup.

## Startup requirements

Startup should be boring and operator-friendly. The normal path must be a single documented command, not a scavenger hunt through several setup scripts.

Required startup behavior:

- Provide one primary operator entrypoint, preferably `rvr-console`, that starts the console and then guides the operator through lidar-only, mapping, dry-run, or motor-capable choices from inside the UI.
- The entrypoint may perform prerequisite setup internally, such as sourcing ROS setup files or selecting config paths, but the operator should not have to run multiple scripts manually before the console is usable.
- If environment setup is missing, the console must report the missing prerequisite and the exact fix in plain language instead of failing with a raw shell/ROS traceback.
- Safe startup must default to no live RVR driver and no `/cmd_vel` exposure. A simple startup path is not permission to bypass the motor warning/confirmation gates.
- Development/test shortcuts are allowed, but they must not become the documented operator workflow.

## Operator usability requirements

The console must be designed for a tired operator standing near a physical robot, not for someone who has memorized a private command language.

Required usability behavior:

- `/help` must be comprehensive and organized into clear sections: safe inspection, mapping launch controls, STOP/ESTOP, dry-run/testing, and `MOTOR-CAPABLE` commands.
- Every screen or mode must show enough hints for the next safe action, especially after warnings, failed launches, STOP, ESTOP, and dry-run transitions.
- The command input should support autocomplete or tab completion when the UI framework allows it. At minimum, partial-command suggestions must be shown for known slash commands.
- Invalid or incomplete commands must produce corrective help that names the closest valid command and, for motor-capable commands, the exact required confirmation form.
- Help text must prefer plain language over internal ROS jargon. ROS topic/service names are useful diagnostics, but they must not be the primary way an operator learns what to do.
- Common safe workflows should be menu-selectable or prompt-driven where practical, especially: start lidar-only mapping, check devices, dry-run start/stop, STOP/ESTOP, clear ESTOP, and show status.
- Motor-capable workflows may use menus to explain options, but they must still require the exact confirmation commands specified in this document before live launch, arming, or motion.

## Slash commands

### Non-motor commands

These commands must never publish nonzero velocity and must never launch the live RVR driver:

| Command | Semantics |
|---|---|
| `/help` | Show all command groups and identify motor-capable commands. |
| `/status` | Append one-line status summary using the same fields as the status pane. |
| `/devices` | Re-check `/dev/rplidar`, `/dev/ttyAMA0`, and current ROS graph topics/services. |
| `/battery` | Show latest battery state or `waiting`. |
| `/speed <mps>` | Set future keyboard-drive linear speed only. Must reject negative values and values above configured `max_linear_mps`. Does not publish `/cmd_vel`. |
| `/turn <rad_s>` | Set future keyboard-drive angular speed only. Must reject negative values and values above configured `max_angular_rad_s`. Does not publish `/cmd_vel`. |
| `/disarm` | Set `motor_armed=false`, immediately publish zero velocity if `/cmd_vel` is available, and record a STOP event with source `disarm`. |
| `/stop` | Immediate STOP behavior below. No confirmation required because STOP only reduces motion. |
| `/estop` | Immediate ESTOP behavior below. No confirmation required because ESTOP only reduces motion and inhibits future motion. |
| `/clear-estop confirm` | Clear the software ESTOP inhibit through `/clear_estop`, leave `motor_armed=false`, and require a later `/arm confirm` before motion. Bare `/clear-estop` must only print the exact recovery command. |
| `/mapping status` | Show managed launch state, process id if owned by the console, and expected ROS topics. |
| `/lidar start` | Start lidar-only launch: `ros2 launch sphero_rvr_driver lidar.launch.py`. No RVR driver, no `/cmd_vel`. |
| `/lidar stop` | Stop a managed lidar launch. |
| `/mapping start` | Start safe mapping launch: `ros2 launch sphero_rvr_driver mapping.launch.py start_rvr:=false`. This is lidar + SLAM only. |
| `/mapping stop` | Transition to `stopping`, publish zero velocity if `/cmd_vel` is available, stop the managed launch process, then transition to `idle` or `failed-launch` with a visible result. |
| `rvr-console --dry-run` | Start fake-client/fake-driver mode. Must not require ROS or hardware. Uses fake battery/diagnostic/device values clearly labeled fake. |
| `/quit` | Publish zero velocity if `/cmd_vel` is available, disarm, stop managed launch if owned by the console, close ROS client, and exit. Alias `/exit` may remain for compatibility but must display as `/quit` in help. |

### Motor-capable commands

These are the only motor-capable TUI commands. The UI must list them under a `MOTOR-CAPABLE` help section, show the exact warning, and require the exact confirmation form below.

| Command | Confirmation required | Semantics |
|---|---:|---|
| `/mapping full confirm` | yes | In live mode, start full mapping graph: `ros2 launch sphero_rvr_driver mapping.launch.py start_rvr:=true`. Exposes `/cmd_vel` through the RVR driver but must leave `motor_armed=false`. Bare `/mapping full` must warn only. In dry-run mode, simulate the same state transition with the fake client only; do not launch ROS or publish to live topics. |
| `/arm confirm` | yes | Set `motor_armed=true` only if `mapping_mode=motor-capable`, `/cmd_vel` is available from the managed motor-capable graph or fake dry-run client, and `estop_inhibited=false`. Does not itself publish motion. Bare `/arm` must only print the warning and the exact confirmation command. No external-driver attach path exists in this phase. |
| `/nudge-forward <meters> confirm` | yes | Run one calibrated forward nudge using `/cmd_vel`, then publish zero velocity and disarm unless `--keep-armed` is implemented later. |
| `/nudge-back <meters> confirm` | yes | Run one calibrated reverse nudge using `/cmd_vel`, then publish zero velocity and disarm unless `--keep-armed` is implemented later. |
| `/nudge-left <degrees> confirm` | yes | Run one calibrated in-place left turn nudge using `/cmd_vel`, then publish zero velocity and disarm unless `--keep-armed` is implemented later. |
| `/nudge-right <degrees> confirm` | yes | Run one calibrated in-place right turn nudge using `/cmd_vel`, then publish zero velocity and disarm unless `--keep-armed` is implemented later. |
| Keyboard `↑/w`, `↓/s`, `←/a`, `→/d` | gated by prior `/arm confirm` | Publish bounded nonzero `/cmd_vel` only while armed and not ESTOP-inhibited. Prefer nudge commands for mapping; keyboard drive is secondary. |

No other slash command may publish nonzero `/cmd_vel`. If a future command can move the robot, it must be added to this table before implementation.

## STOP behavior

STOP is motion-reducing and must never require confirmation.

Triggers:

- slash `/stop`
- space key
- `/disarm`
- key timeout zero-publish
- `/mapping stop`
- `/quit` / `/exit`
- process cleanup on TUI shutdown

Required behavior:

1. Immediately publish `Twist(linear.x=0.0, angular.z=0.0)` to `/cmd_vel` if the topic is available.
2. If the `/stop` service is available and the source is explicit `/stop` or space key, call `/stop` after publishing zero velocity. Do not wait for service availability before the zero publish.
3. Set internal active motion to none.
4. Set `motor_armed=false` for `/disarm`, `/mapping stop`, and `/quit`; for explicit `/stop` and space key, preserving `motor_armed=true` is allowed only if the UI clearly shows `Stopped; still armed`.
5. Update `Last STOP/ESTOP` with timestamp, source, and result.
6. Append a visible history message.

If publishing zero velocity fails, the UI must show `STOP FAILED` and suggest ESTOP/power-off. The implementation must still attempt the `/stop` service if available.

## ESTOP behavior

ESTOP is motion-reducing and must never require confirmation.

Triggers:

- slash `/estop`
- `e` key
- implementation-detected unsafe state, if added later

Required behavior:

1. Immediately publish `Twist(linear.x=0.0, angular.z=0.0)` to `/cmd_vel` if the topic is available.
2. Call the `/estop` service if available. Do not wait for service availability before the zero publish.
3. Set `motor_armed=false`.
4. Set `estop_inhibited=true` locally even if the service call fails; failed service calls are still unsafe until the operator resolves them.
5. Reject every motor-capable command while inhibited, including `/arm confirm`, nudges, and keyboard motion.
6. Update status pane to show `ESTOP inhibited: true` and a high-visibility message.
7. Update `Last STOP/ESTOP` with timestamp, source, and result.

Recovery:

1. Operator runs `/clear-estop confirm`.
2. UI calls `/clear_estop` if available and clears local inhibit only on success.
3. UI leaves `motor_armed=false`.
4. Operator must separately run `/arm confirm` before any motion.

## Mapping launch controls and state transitions

### Launch profiles

| Profile | Command | ROS launch | Motor capable |
|---|---|---|---:|
| `lidar` | `/lidar start` | `ros2 launch sphero_rvr_driver lidar.launch.py` | no |
| `mapping-lidar` | `/mapping start` | `ros2 launch sphero_rvr_driver mapping.launch.py start_rvr:=false` | no |
| `mapping-motor` | `/mapping full confirm` | `ros2 launch sphero_rvr_driver mapping.launch.py start_rvr:=true` | yes |
| `dry-run` | `rvr-console --dry-run` | fake in-process client/driver | no live hardware |

`mapping.launch.py` must keep `start_rvr:=false` as the default. Any implementation that changes that default fails this plan.

### State machine

```text
idle
  /lidar start                -> lidar-only
  /mapping start              -> lidar-only
  /mapping full               -> idle + warning only
  /mapping full confirm       -> motor-capable if launch succeeds, failed-launch if not
  rvr-console --dry-run       -> idle with hardware_mode=dry-run and launch_profile=dry-run

lidar-only
  /mapping full               -> lidar-only + warning only
  /mapping full confirm       -> motor-capable after stopping/replacing incompatible managed launch, failed-launch if not
  /mapping stop               -> stopping -> idle
  launch process exits clean  -> idle
  launch process exits error  -> failed-launch

motor-capable
  /arm                        -> warning only; remain motor-capable + motor_armed=false
  /arm confirm                -> motor-capable + motor_armed=true if not ESTOP-inhibited
  /stop or space              -> motor-capable + zero velocity; armed state per STOP rule
  /disarm                    -> motor-capable + motor_armed=false + zero velocity
  /estop or e                 -> motor-capable + motor_armed=false + estop_inhibited=true
  /mapping stop               -> stopping + zero velocity + motor_armed=false -> idle
  launch process exits        -> failed-launch + motor_armed=false + zero velocity attempt

stopping
  stop complete               -> idle
  stop timeout/error          -> failed-launch + visible remediation

failed-launch
  /mapping status             -> remain failed-launch and show error
  /mapping stop               -> stopping -> idle if cleanup succeeds
  /lidar start or /mapping start -> retry safe profile
  /mapping full confirm       -> retry motor profile after warning/confirmation
```

A transition into `failed-launch` must include a visible reason: missing executable/package, missing `/dev/rplidar`, missing `/dev/ttyAMA0`, ROS launch exit code, missing required topics, or timeout.

Dry-run uses the same visible states for UI coverage, but every transition must be backed by the fake client. In dry-run, `/mapping full confirm` means `mapping_mode=motor-capable`, `launch_profile=mapping-motor`, fake `/cmd_vel` available, and no live ROS process or hardware device opened.

## Calibrated nudge controls

Nudges are the preferred movement interface for early mapping because they are bounded, logged, and easier to validate than free-drive.

Commands:

```text
/nudge-forward <meters> confirm
/nudge-back <meters> confirm
/nudge-left <degrees> confirm
/nudge-right <degrees> confirm
```

Initial limits:

- Forward/back distance: `0.02 <= meters <= 0.25`.
- Turn angle: `5 <= degrees <= 30`.
- Linear velocity: use configured `max_linear_mps`, capped at `0.05 m/s` for nudges until hardware validation raises it.
- Angular velocity: use configured `max_angular_rad_s`, capped at `0.35 rad/s` for nudges until hardware validation raises it.
- Distance calibration: use `odom_counts_per_meter=4337.768` for expected encoder-distance reporting, not for blindly trusting distance closure.

Nudge execution semantics:

1. Require `hardware_mode=live`, `mapping_mode=motor-capable`, `motor_armed=true`, and `estop_inhibited=false`.
2. Display the exact motor warning and require the exact command with `confirm`.
3. Publish bounded `/cmd_vel` at a fixed cadence.
4. Stop by the earliest of: requested duration estimate, odometry threshold, operator `/stop`, operator `/estop`, stale-command timeout, or execution timeout.
5. Always publish zero velocity at the end.
6. Log requested units, velocities, duration estimate, odometry delta if available, STOP/ESTOP state, and result.
7. Default to disarming after each nudge. If an implementation later adds a keep-armed option, it must be a separate explicit confirmed command and update this document.

Keyboard/free-drive remains available only after `/arm confirm`, but help text must say: `Prefer /nudge-* commands for mapping validation; keyboard drive is secondary.`

## Dry-run and fake-driver mode

Required entrypoint:

```bash
rvr-console --dry-run
```

Dry-run slash commands (`/dry-run start`, `/dry-run stop`) are future work. The shipped non-hardware path is the `rvr-console --dry-run` entrypoint plus the same TUI slash commands used in live mode.

Dry-run requirements:

- Must run on macOS/non-ROS development hosts without RVR hardware.
- Must not source ROS, open `/dev/ttyAMA0`, open `/dev/rplidar`, launch `rvr.launch.py`, or publish to a live `/cmd_vel`.
- Must use a fake client with the same UI-facing methods as `RVRROSClient`: `publish_velocity`, `stop`, `estop`, `clear_estop`, and status fields.
- Must visibly label all fake values as fake.
- Must support scripted tests of `/mapping full confirm`, `/arm confirm`, nudges, STOP, ESTOP, `/clear-estop confirm`, and failed-launch paths without touching hardware.
- Must be selectable from tests without curses by constructing the TUI around a fake client.

## Test plan

### Non-hardware validation path

Run on development hosts before any hardware validation:

```bash
PYTHONPATH=src python -m pytest \
  tests/test_tui_commands.py \
  tests/test_tui_keymap.py \
  tests/test_tui_app.py \
  tests/test_rvr_console_script.py \
  tests/test_ros_node_config.py \
  tests/test_ros_safe_surfaces.py

git diff --check
```

Required new/updated unit coverage for implementation cards:

- Command parser accepts every exact slash command in this document and rejects misspellings/aliases for motor confirmation commands.
- Safe modes should not create a live ROS `/cmd_vel` publisher; the current live-mode publisher is a known follow-up, and only the managed motor-capable graph or fake dry-run client may be treated as a safe velocity sink.
- Bare `/arm` and `/mapping full` warn but do not arm, launch the live driver, or publish `/cmd_vel`.
- `/mapping full confirm` leaves `motor_armed=false` after launch.
- `/mapping full confirm` in dry-run simulates state only and does not source ROS, launch ROS, open devices, or publish to live topics.
- `/arm confirm` only arms when `/cmd_vel` is available and `estop_inhibited=false`.
- STOP publishes zero velocity before any `/stop` service wait.
- ESTOP publishes zero velocity, calls `/estop`, sets persistent inhibit, and blocks later motor-capable commands.
- `/clear-estop confirm` clears inhibit only on service success and leaves `motor_armed=false`.
- Every nudge command requires `confirm`, validates units/limits, publishes zero at completion, and logs result.
- `rvr-console --dry-run` does not source ROS setup or launch the live driver.
- Status pane renders the required device/config/state fields.
- `/help`, menus/prompts, autocomplete or command suggestions, and invalid-command recovery make safe workflows discoverable without requiring memorized obscure commands.
- The documented operator startup path is a single entrypoint, preferably `rvr-console`, and does not require manually chaining setup/launch scripts.
- Failed launch transitions to `failed-launch` with a visible reason.

### ROS environment no-motion validation

Run on `sphero-pi-2` after sourcing ROS and rebuilding, without launching the live RVR driver:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch sphero_rvr_driver mapping.launch.py start_rvr:=false
ros2 topic list
ros2 topic echo --once /scan
```

Expected:

- `/scan` exists when lidar is connected.
- `/cmd_vel` is not exposed by the safe default launch.
- Status pane reports `mapping_mode=lidar-only`, `launch_profile=mapping-lidar`, lidar `/dev/rplidar` at `460800`, and RVR UART status without trying to open the RVR driver.

## Hardware validation gates

Hardware validation requires explicit approval after showing:

```text
WARNING: this can start the RVR motors
```

Gate order:

1. **Bench/no-motion lidar gate**
   - `/lidar start` or `/mapping start` only.
   - Confirm `/dev/rplidar` at `460800`, `/scan`, and `base_link -> laser`.
   - Confirm `/cmd_vel` is not exposed.

2. **Motor graph restrained gate**
   - Robot physically restrained/suspended or in a clear controlled area.
   - Run `/mapping full confirm`.
   - Confirm `/cmd_vel`, `/odom`, diagnostics, battery, and `odom -> base_link` appear.
   - Confirm `motor_armed=false` immediately after launch.

3. **STOP/ESTOP restrained gate**
   - Run `/arm confirm`.
   - Run `/stop`; verify immediate zero velocity and visible last STOP event.
   - Run `/arm confirm` again.
   - Run `/estop`; verify zero velocity, `estop_inhibited=true`, and later motion commands are rejected.
   - Run `/clear-estop confirm`; verify `motor_armed=false`.

4. **Tiny nudge gate**
   - Use `/nudge-forward 0.02 confirm` first.
   - Verify final zero velocity, odometry delta, and no continued motion.
   - Only then try larger bounded nudges within this document's limits.

5. **Manual mapping gate**
   - Use nudge commands for the first map-quality scan path.
   - Keyboard drive may be used only after the nudge gate passes and the operator explicitly chooses free-drive with `/arm confirm`.
   - Save maps only after STOP/ESTOP and odometry sanity checks pass in the same session.

## Implementation acceptance criteria

An implementation satisfies this plan only if all of these are true:

- `mapping.launch.py` still defaults to `start_rvr:=false`.
- The status pane includes every required field listed above.
- The interface is discoverable and easy to operate: plentiful `/help`, menu/prompt guidance, autocomplete or suggestions where possible, and corrective invalid-command help.
- Startup is a single documented operator entrypoint, preferably `rvr-console`, with setup guidance inside the console rather than multiple manual scripts.
- Every motor-capable command is exactly listed in the `Motor-capable commands` table and implemented with confirmation before live launch or nonzero `/cmd_vel` publication.
- Safe startup and safe mapping modes should not construct a live `/cmd_vel` publisher or otherwise expose a new motion path from the TUI; current live-mode publisher creation is tracked as follow-up work.
- Bare `/arm` and bare `/mapping full` cannot move the robot or launch the motor-capable graph.
- STOP and ESTOP publish zero velocity immediately before service waits.
- ESTOP creates a persistent inhibited state that survives failed service calls and blocks all later motion until `/clear-estop confirm` succeeds.
- `/clear-estop confirm` does not arm the robot.
- Mapping state transitions match the state machine above, including `failed-launch` reasons.
- Nudges use fixed units (`meters`, `degrees`), bounded ranges, final zero velocity, and log odometry/result data.
- Dry-run/fake-driver mode validates command gating and state transitions without ROS or hardware.
- Non-hardware tests cover parser, state machine, STOP/ESTOP, dry-run, status rendering, and nudge gating.
- Hardware validation follows the gate order above and records results before treating the interface as mapping-ready.
