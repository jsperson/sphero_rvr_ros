# Pi web mission stack

This runbook deploys the integrated prompt planner, persistent service, and web
console without starting the rover driver or granting physical execution.

## Authority boundary

```text
browser
  -> authenticated Tailscale Serve HTTPS
  -> 127.0.0.1 rvr_mission_web
  -> LiveMissionWebAdapter
  -> user-only Unix socket
  -> persistent MissionService + SQLite
  -> Pi-local Codex CLI using ChatGPT OAuth
```

Planning credentials, proposal persistence, approval identity, and the exact
digest gate stay on the Pi. The browser does not receive OAuth material and has
no ROS, serial, motor, or OpenAI route. The deployed owner is intentionally
proposal-only: `live_execution_enabled=false`, so no route executor is
instantiated, and the two systemd units start no motor-capable process.

## Deployment layout

Use a dedicated workspace so an older or dirty rover checkout remains untouched:

```text
~/ros2_ws_mission_stack/
  src/sphero_rvr_ros/       exact reviewed integration SHA
  build/ install/ log/      isolated colcon output
~/.config/sphero_rvr/mission-stack.env
~/.config/systemd/user/rvr-mission-*.service
~/.local/state/sphero_rvr/missions.sqlite3
~/.local/state/sphero_rvr/mission-service.sock
```

Before deployment, record the current repo status, service state, Serve config,
ROS graph, serial users, and process list. Preserve any existing checkout or
Serve configuration instead of resetting it.

## Build and no-hardware gate

Fetch the reviewed integration branch into the dedicated workspace, verify its
SHA, then build only this package:

```bash
cd ~/ros2_ws_mission_stack
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select sphero_rvr_driver
source install/setup.bash
```

Run ROS-free tests from the deployed checkout only through the bounded runner:

```bash
cd ~/ros2_ws_mission_stack/src/sphero_rvr_ros
python3 scripts/run_pytest_bounded.py --timeout 60 -- -vv \
  tests/test_mission_service.py tests/test_prompt_mission_controller.py \
  tests/test_live_mission_service.py tests/test_mission_web.py \
  tests/test_prompt_drive.py tests/test_live_route_runner.py \
  tests/test_odometry.py tests/test_ros_node_config.py \
  tests/test_ros_safe_surfaces.py tests/test_system_validation.py \
  tests/test_package_metadata.py
```

These commands do not start the rover driver, route runner, collision supervisor,
lidar, camera, or serial access.

## Install the user services

Install without enabling or starting:

```bash
~/ros2_ws_mission_stack/install/sphero_rvr_driver/share/sphero_rvr_driver/scripts/install-rvr-mission-stack-services \
  ~/ros2_ws_mission_stack/install/sphero_rvr_driver/share/sphero_rvr_driver
```

Edit `~/.config/sphero_rvr/mission-stack.env`, keep it mode `0600`, and set:

```text
RVR_ROS_WORKSPACE=/home/jsperson/ros2_ws_mission_stack
RVR_SOURCE_SHA=<exact-reviewed-sha>
RVR_DEPLOYED_SHA=<same-exact-sha>
RVR_LIVE_EXECUTION_ENABLED=false
RVR_LIVE_EXECUTION_REVIEWED_SHA=
RVR_WEB_PORT=8765
RVR_WEB_ORIGIN=https://sphero-pi-2.tailab4000.ts.net
```

Do not add OAuth tokens, API keys, cookies, or credentials. Verify the Pi-local
session separately:

```bash
$HOME/.local/bin/codex login status
```

The required output is `Logged in using ChatGPT`.

Start the owner and web service, then enable user lingering only if the reviewed
Pi should run the units across logout and reboot:

```bash
systemctl --user enable --now rvr-mission-service.service rvr-mission-web.service
systemctl --user --no-pager --full status rvr-mission-service.service rvr-mission-web.service
```

## Tailnet-only HTTPS

The application listens only on `127.0.0.1:8765`. Publish it with Tailscale
Serve, not Funnel, and preserve tailnet ACLs:

```bash
sudo tailscale serve --bg --yes 8765
sudo tailscale serve status
```

The expected URL is `https://sphero-pi-2.tailab4000.ts.net`. Tailscale Serve is
tailnet-only and applies tailnet access controls. It strips client-supplied
identity headers and supplies `Tailscale-User-Login` to a localhost backend; the
web service requires that header for every state-changing request. See the
[Tailscale Serve documentation](https://tailscale.com/docs/features/tailscale-serve)
and [Serve CLI reference](https://tailscale.com/docs/reference/tailscale-cli/serve).

Never enable Funnel for this stack. Do not expose port 8765 on a LAN address, add
an unrelated reverse proxy, or weaken tailnet ACLs.

## Proposal-only acceptance

With no hardware nodes running:

1. Open the tailnet HTTPS URL and verify the banner says
   `LIVE - PROPOSAL ONLY / EXECUTION LOCKED`.
2. Submit a bounded prompt and wait for `PROPOSED` or a structured `REJECTED`
   result from Pi-local OAuth planning.
3. Confirm model, reasoning effort, limits, route segments, source/deployed SHAs,
   and full digest are visible.
4. Confirm approval remains disabled and no route is queued.
5. Cancel a proposal and confirm the durable event and terminal reason.
6. Restart the service and prove an approved/queued/running synthetic database
   record becomes `recovery_required`; no work resumes automatically.
7. Capture desktop/mobile screenshots and browser console/network evidence.

After each step, verify the ROS graph, serial users, relevant processes, socket
owner, database access, and absence of nonzero motion commands. Missing live map,
pose, collision, and sensor sources must display as unavailable or stale—not as
fixtures.

STOP and ESTOP must display `UNKNOWN` when neither the collision supervisor nor
the dedicated control-state source supplies fresh authoritative evidence. Do not
treat an absent source as READY/CLEAR.

## Rollback

Because this deployment uses an isolated workspace, rollback does not change the
older rover checkout:

```bash
systemctl --user disable --now rvr-mission-web.service rvr-mission-service.service
sudo tailscale serve --https=443 off
```

Confirm both units are inactive, `tailscale serve status` has no application
mapping, the Unix socket has no owner, and no mission/ROS/hardware processes are
left. Preserve the SQLite database and service logs as evidence; do not delete
them during rollback. Re-enabling requires revalidating the exact deployed SHA
and environment file.

## Attended physical execution gate

The integrated browser-to-track path has been proven once with the rover
suspended. That run also exposed an unacceptable terminal-stop discrepancy, so
it is motion-path evidence rather than calibrated-distance acceptance. The
runner now keeps a reached segment nonterminal until fresh odometry/encoder
samples are stable, reports settle duration and final target error, and fails
closed on continued motion or excessive final error. A zero command now takes
the driver's validated immediate-stop path. This software still requires a new
operator-attended 10 cm revalidation before the turn stage.

While Scott is present, prepare the supervised ROS graph and verify fresh odom,
collision `CLEAR`, STOP `READY`, ESTOP `CLEAR`, route-runner request/status graph
edges, and zero command. Then stop the mission service, set these two values in
`~/.config/sphero_rvr/mission-stack.env`, and restart it:

```text
RVR_LIVE_EXECUTION_ENABLED=true
RVR_LIVE_EXECUTION_REVIEWED_SHA=<exact RVR_DEPLOYED_SHA>
```

Startup must fail unless the reviewed, source, and deployed SHAs all match. A LIVE/execution-enabled banner
only means proposals may become confirmable; it is not permission for an unattended
mission. During one Scott-attended calibration series, the exact-SHA gate may
remain enabled for up to three repeats of the same bounded stage. For each newly
planned route, Scott reviews its typed effect and clicks **Approve and run** once;
he never enters a GUID, digest, code, or hash. The server reloads and binds the
unchanged proposal internally and rechecks safety before recording confirmation
and again before route submission.

An attended orchestration layer may treat an unambiguous confirmation such as
`Ready for Run 2` as that one approval action when the exact bounded stage has
already been stated, the newly planned typed proposal matches that stage, the
operator identity is authenticated, and current safety evidence passes. The
orchestrator supplies `confirm_current_proposal`; the server still reloads and
binds the complete digest internally. Never require the operator to transcribe a
GUID, digest, code, or hash. A changed proposal, ambiguous confirmation, expired
approval, different stage, or failed safety check requires a new human-readable
review and confirmation.

After the reviewed repeat series—or immediately after any failed check—restore
`false` and blank, restart the mission service, stop the supervised ROS graph,
issue/verify zero, and repeat the process/device cleanup audit. A later turn,
multi-step prompt, or materially different stage starts a new attended series.
Follow `docs/motion_calibration.md` and stop on missing, stale, or inconsistent
evidence.

### Exact attended procedure

These commands are physical-test preparation, not authorization. Do not run them
unless Scott is present, the rover is restrained for the first two stages, the
area is clear, and the operator can use physical power/ESTOP immediately.

1. In a Pi shell, prove the candidate and config are still locked:

   ```bash
   cd /home/jsperson/ros2_ws_mission_stack/src/sphero_rvr_ros
   candidate=$(git rev-parse HEAD)
   test -z "$(git status --short)"
   grep -E '^RVR_(SOURCE_SHA|DEPLOYED_SHA|LIVE_EXECUTION_ENABLED|LIVE_EXECUTION_REVIEWED_SHA)=' \
     ~/.config/sphero_rvr/mission-stack.env
   codex login status
   ```

   Both SHA values must equal `candidate`, execution must be `false`, the reviewed
   SHA must be blank, and Codex must report `Logged in using ChatGPT`.

2. Source the installed exact candidate. Start lidar/TF in terminal A:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source /home/jsperson/ros2_ws/install/rplidar_ros/share/rplidar_ros/local_setup.bash
   source /home/jsperson/ros2_ws_mission_stack/install/setup.bash
   ros2 launch sphero_rvr_driver lidar.launch.py
   ```

   The isolated mission workspace intentionally contains the reviewed rover
   package only. The single `rplidar_ros` local setup line supplies the Pi's
   existing lidar executable without overlaying the older rover package; the
   final source line must still resolve `sphero_rvr_driver` from the isolated
   exact candidate.

   In terminal B, after sourcing `/opt/ros/jazzy/setup.bash` and the isolated
   mission workspace setup, start only the
   supervised driver, collision supervisor, and deterministic route runner:

   ```bash
   ros2 launch sphero_rvr_driver supervised_rvr.launch.py \
     start_collision_stop:=true \
     start_live_route_runner:=true \
     start_range_motion:=false
   ```

   This second command is motor-capable. The driver is remapped to
   `/cmd_vel_motor`; only the collision supervisor may publish there.
   The normal forward slow corridor remains `-45` to `+45` degrees. When Scott
   has physically identified a stable return as safely off the measured
   straight lane, an attended calibration session may start the supervisor with
   an explicitly reviewed narrower corridor, for example:

   ```bash
   ros2 launch sphero_rvr_driver supervised_rvr.launch.py \
     start_collision_stop:=true \
     start_live_route_runner:=true \
     start_range_motion:=false \
     front_slow_min_angle_deg:=-35.0 \
     front_slow_max_angle_deg:=35.0
   ```

   These are startup-only launch inputs loaded into the supervisor's immutable
   arbitration configuration. A successful `ros2 param set` after startup is
   not an override and must never be treated as one. Record the selected
   corridor and the excluded return's measured bearing in the session evidence.
   Turn safety uses the current-command trajectory projected through the command
   timeout plus measured stopping horizon. It sweeps the configured rectangular
   footprint with the startup-only `trajectory_clearance_margin_m` rather than
   applying the forward stop threshold to a whole side sector. Do not alter the
   footprint, trajectory margin, inner stop sector, stop/slow distances,
   stale-data policy, or TF requirements without a reviewed physical-geometry
   change and exact-SHA validation.

3. In terminal C, source `/opt/ros/jazzy/setup.bash` and the isolated mission
   workspace setup, then capture readiness without submitting a route:

   ```bash
   ros2 node list
   ros2 topic info /cmd_vel --verbose
   ros2 topic info /cmd_vel_motor --verbose
   ros2 topic info /mission_api/v2/live_route/request --verbose
   ros2 topic info /mission_api/v2/live_route/status --verbose
   timeout 5 ros2 topic echo /scan --once
   timeout 5 ros2 topic echo /odom --once
   timeout 5 ros2 topic echo /encoder_counts --once
   timeout 5 ros2 topic echo /collision_stop/state --once
   timeout 5 ros2 topic echo /cmd_vel_motor --once
   ros2 service list | grep -E '^/(stop|estop|clear_estop)$|^/live_route/cancel$'
   curl -fsS http://127.0.0.1:8765/api/web/state | python3 -m json.tool
   ```

   Require one node per component, live route request/status graph edges, fresh
   scan/odom/encoder samples, collision `CLEAR`, STOP `READY`, ESTOP `CLEAR`, and
   an exactly zero motor-bound Twist. The collision state and web safety strip
   must show the authoritative forward-corridor clearance and the startup
   corridor bearings. Stop immediately on any discrepancy.

4. Only after Scott reviews that evidence, edit
   `~/.config/sphero_rvr/mission-stack.env` so execution is `true` and the reviewed
   SHA equals the complete `candidate`, then restart only the mission service:

   ```bash
   chmod 0600 ~/.config/sphero_rvr/mission-stack.env
   systemctl --user restart rvr-mission-service.service
   systemctl --user is-active rvr-mission-service.service rvr-mission-web.service
   ```

   A typo or SHA mismatch must make service startup fail. Do not alter the unit,
   YAML defaults, Tailscale config, collision policy, or approval TTL to make it
   start.

5. Open the tailnet HTTPS page. Confirm the LIVE physical-execution banner and
   fresh safety state. Enter only the current stage prompt. Scott reviews the
   complete typed proposal, physical effect, limits, and model identity, then
   clicks **Approve and run** once. The Pi reloads that current persisted proposal
   and supplies its exact digest to the mission service; the browser does not
   manufacture approval authority and Scott does not see or copy a hash. Repeat
   this single-click review for each route in the bounded series.

6. At terminal state, use the page's Terminal evidence panel and Terminal result
   JSON link. Record start/final pose and timestamps, final heading, route-local
   measurements, left/right encoder deltas, per-track distances, collision state,
   terminal reason, `terminal_settled`, settle duration, final target error, and
   independent stationary post-terminal samples. Require `terminal_settled=true`
   and a bounded target error. Do not advance if a required field is null, the
   tracks disagree unexpectedly, heading drift is unexplained, the pose continues
   changing, or any safety/error state occurred.

   Capture `/diagnostics` throughout short turn tests, not only before and after.
   The driver reports:

   - completed host transport writes and motion-capable write counts;
   - the last motor command ID, sequence, payload, and host write timestamp;
   - motor-stall notification count and latest motor/index state;
   - current motor-fault state;
   - left/right motor temperature and raw thermal-protection status; and
   - battery voltage refreshed every 0.5 seconds.

   A completed raw-motor transport write proves that the Pi finished writing the
   packet to UART. The RVR raw-motor command is fire-and-forget, so this evidence
   does **not** claim firmware receipt or application. Compare the write counter,
   sequence, payload, encoder progress, protection notifications, and voltage
   across the full attempt. If the tracks stop while motion writes continue,
   stop the series and diagnose firmware protection, power, or traction rather
   than increasing speed.

7. Relock after the bounded repeat series, or immediately after any failed
   readiness or execution check:

   ```bash
   ros2 service call /stop std_srvs/srv/Trigger '{}'
   timeout 5 ros2 topic echo /cmd_vel_motor --once
   ```

   Restore `RVR_LIVE_EXECUTION_ENABLED=false` and blank
   `RVR_LIVE_EXECUTION_REVIEWED_SHA`, restart the mission service, verify the web
   state reports `live/proposal-only`, then stop terminals B and A. Finish with:

   ```bash
   systemctl --user is-active rvr-mission-service.service rvr-mission-web.service
   ss -ltnp | grep '127.0.0.1:8765'
   ss -xlpn | grep 'mission-service.sock'
   ros2 node list --no-daemon
   pgrep -af '[r]vr_node|[l]ive_route_runner|[l]idar_collision_stop|[r]os2 launch' || true
   lsof /dev/ttyAMA0 || true
   ```

   The two proposal-only services may remain active. The only ROS node may be
   `/live_mission_service`; no hardware/launch process or serial owner may remain.
