# Pi web mission stack

This runbook deploys the integrated adaptive mission product. Boot and proposal
generation start no rover hardware; one authenticated approval activates the
fixed supervised graph for that mission only.

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
no ROS, serial, motor, or OpenAI route. The reviewed product owner selects
Adaptive mission and installs an exact-SHA approval-activation capability. The
motor-capable unit remains inactive until a Tailscale-authenticated operator
approves the current proposal, and it is stopped again before a terminal result
is reported.

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
RVR_APPROVAL_ACTIVATION_ENABLED=true
RVR_APPROVAL_ACTIVATION_REVIEWED_SHA=<same-exact-sha>
RVR_APPROVAL_ACTIVATION_TIMEOUT_S=30.0
RVR_ADAPTIVE_MISSION_ENABLED=true
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

Do not enable `rvr-adaptive-mission.service`. Its `BindsTo` relationship and
exact-SHA `ExecStartPre` are additional fail-closed controls; MissionService
starts it only after authenticated approval and stops it on every terminal,
cancellation, activation failure, or owner restart.

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

Adaptive mission semantic movement uses the dedicated composed graph:

```bash
ros2 launch sphero_rvr_driver adaptive_mission_perception.launch.py
```

That command is the safe sensor-only default: `start_rvr=false` and
`start_live_route_runner=false`. It starts lidar, camera, moving-SLAM
configuration, and the no-authority semantic producer so its status topics can
be inspected without a rover driver. The semantic producer publishes no
`Twist`, route request, serial command, or motor topic.

Only an attended, exact-SHA-reviewed physical session may additionally set
`start_rvr:=true start_live_route_runner:=true`. The composed graph still routes
typed Adaptive mission intents through `live_route_runner → /cmd_vel →
lidar_collision_stop_supervisor → /cmd_vel_motor`. The mission service receives
camera, localization, and semantic-map status on its existing Mission API
topics. It withholds semantic tracks from the planner unless camera,
localization, and semantic-map receipts are all fresh, and it never treats
recognition as collision evidence.

The integrated browser-to-track path has been proven once with the rover
suspended. That run also exposed an unacceptable terminal-stop discrepancy, so
it is motion-path evidence rather than calibrated-distance acceptance. The
runner now keeps a reached segment nonterminal until fresh odometry/encoder
samples are stable, reports settle duration and final target error, and fails
closed on continued motion or excessive final error. A zero command now takes
the driver's validated immediate-stop path. The July 24 attended ground attempt
then moved about 1–2 inches (0.04438 m in odometry, with matched 193/192 encoder
counts) before continuous slow progress was falsely classified as a software
stall. The translation activity checkpoint was reduced from 0.015 m to 0.005 m
without changing target/settle tolerances, speed, timeout, or safety policy.
The exact-SHA repeat at `ba968286372053042724e94af8cb0c697c80c7fa` then moved
0.01072 m before the collision supervisor correctly zeroed output and terminated
the mission when source-stamp age reached 0.340 s. Rosbag and installed-driver
source evidence showed that the 10 Hz RPLidar stamps each scan before blocking
to acquire a full revolution; the old 0.30 s source-stamp comparison therefore
conflated normal acquisition time with time since evidence receipt. Freshness
now uses independent fail-closed checks: 0.30 s since callback receipt detects a
stopped/delayed stream before the driver's 0.50 s command watchdog; source
stamps must advance monotonically to reject frozen/replayed scans; and a 0.75 s
source-stamp sanity ceiling rejects abnormally delayed acquisition. Both ages
are reported in collision diagnostics.

The next exact-SHA real-OAuth Adaptive mission run proved the physical controller seam at
the installed 0.10 m/s ceiling but failed closed on settled target error:
0.138780 m for a requested 0.10 m, with matched 600/604 encoder counts,
0.211-degree heading change, collision `CLEAR`, and final zero output. The bag
showed one 10 Hz odometry period between the last safe continue decision and
zero, followed by 0.038268 m of drivetrain coast. The route controller now
reserves a 0.25 s measured-progress stopping horizon before the target. The
measured rate is capped at the requested primitive speed, and the unchanged
stationary and 0.03 m terminal-error gates still fail closed. This software
still requires a new operator-attended 10 cm revalidation at the new exact SHA
before the turn stage.

The exact-SHA `a65c50ff2c30a114025d570b04ed66f879774eff` revalidation passed
the real browser/OAuth Adaptive mission loop. The model selected a 0.10 m
`move_distance`, the route runner requested zero at 0.082646 m, and the rover
settled at 0.090715 m after 0.008069 m of additional coast. Terminal evidence
reported `target_reached`, `terminal_settled=true`, 0.600 s settle duration,
0.009285 m target error, matched 392/395 encoder deltas, collision `CLEAR`, and
zero motor output. The updated world snapshot then caused a second real provider
call to choose `stop`, completing the mission with two distinct revisions.

The subsequent attended 45 degree Adaptive mission turn failed closed at the same SHA.
The bounded request and collision-supervised command both remained 0.35 rad/s,
but authoritative odometry measured 3.21 rad/s of yaw. Zero was requested at
43.256 degrees; the next 10 Hz sample had already reached 58.430 degrees and the
rover settled at 60.326 degrees, producing a 15.326 degree `target_error`.
There was no retry or second provider call. Independent STOP, stationary
samples, relock, and hardware cleanup all succeeded. The controller now keeps
the translation estimator capped at requested speed but uses measured turn
progress with a reviewed 3.5 rad/s estimator ceiling.

The next exact-SHA revalidation at
`f9f1ebfcf53aa87ea271660238e864b74e869f37` proved that the shared 0.25 second
horizon was not valid for a turn. It released at 18.757 degrees and settled
stationary at 30.506 degrees, a 14.494 degree error. The bag measured
11.749 degrees of travel from the last pre-zero sample and only 0.158 degrees
after the first post-zero sample. The mission again failed closed after one
provider call, issued no retry, and was independently stopped and relocked.
Turn release now uses a separate 0.10 second horizon. An exact-SHA attempt to
reduce the primitive request to 0.15 rad/s then moved only 2.318 degrees before
the stall gate stopped it; the previous 0.25 rad/s calibration had also
stalled. The route primitive therefore uses the lowest demonstrated breakaway
request, 0.35 rad/s, and retains the last measured yaw rate so the 20 Hz
controller can issue zero between 10 Hz odometry samples. The stale-odom gate
bounds that projection.

The first exact-SHA run of that combination then measured a slower drivetrain
response: zero was requested at 34.826 degrees and the rover settled at 36.670
degrees, 8.330 degrees short. It failed closed as `target_error`, issued no
retry, and was independently stopped, relocked, and cleaned up. The executor
then allowed at most two corrections only after verified stationary undershoot,
inside the same correlated intent and original 5-second timeout.

The exact-SHA `940e8503cc957ede46a4553b15df9f743d14f0a0` revalidation proved
that an odometry-dependent correction was still too coarse. The initial
0.35 rad/s command was nonzero for 0.300 seconds and settled at 37.882 degrees.
One correction remained nonzero for 0.150 seconds while the 20 Hz controller
waited for the next 10 Hz odometry update, then coasted to 69.283 degrees. The
mission failed closed as `target_error` with a 24.283 degree error, collision
`CLEAR`, final zero output, no second provider call, and no automatic resume.
Independent STOP, two unchanged encoder samples, relock, process cleanup, and
restart preservation of the terminal result all passed. The sealed evidence is
under
`/home/jsperson/rvr_runs/adaptive_mission_turn45_correction_940e850_20260724T182000Z`.

The exact-SHA `a0edb78620ab7ae8ddc332e95a746dd4c47a6581` repeat showed
that the bounded pulses were effective but stopped just outside the then-current
acceptance threshold.
The initial command settled at 34.352 degrees; two corrections advanced it to
35.353 and 39.568 degrees. The final error was 5.432 degrees, only 0.432 degrees
outside the then-current 5 degree gate, so the mission truthfully failed
`target_error`. Collision remained `CLEAR`, the real provider was called once,
both final command topics were zero, independent STOP and stationary encoder
checks passed, and restart preserved `auto_resume=false`. The sealed evidence
is under
`/home/jsperson/rvr_runs/adaptive_mission_turn45_pulse_a0edb78_20260724T191500Z`.

That bag also found a control-period boundary error: the second nominal 0.05
second correction was refreshed once because the next timer tick arrived at
49.84 ms, just below the elapsed-time comparison. Corrections now emit exactly
one nonzero command publication, and the very next control tick emits zero and
starts another mandatory stationary-settle check even when it arrives slightly
early. The 20 Hz node period is validated at startup. At most three corrections
are allowed, because the attended pulse response showed that one additional
stationary-verified pulse is needed to enter the existing tolerance without
raising speed or weakening the error gate. The executor never corrects
overshoot or a safety/failure terminal, and wall time—not lagged odometry
sample time—enforces the original intent timeout. The installed
collision/driver ceiling remains 0.4 rad/s and the stationary gate is
unchanged. The product decision after this trace was to accept settled turns
within 10 degrees and proceed to capability validation. The prior 39.568-degree
measurement is within that threshold. This changes only terminal turn
precision; collision, freshness, STOP/ESTOP, speed, timeout, cancellation, and
lease gates remain fail-closed.

The exact-SHA `45ae1adc3942850bd43b9a31679869d3339d309a` Adaptive mission
capability session then exercised the complete physical loop. A first approved
mission completed `observe` and a 0.194449 m translation; the real provider's
third revision proposed a 30-degree turn, but a contemporaneous
`SENSOR_STALE reason=rear_unknown` supervisor state vetoed submission with
requested and supervised motion both zero. Later `CLEAR` evidence did not
resume the terminal mission. Independent STOP, stationary encoder samples,
relock, cleanup, and restart preservation passed. Its sealed evidence is
`/home/jsperson/rvr_runs/adaptive_mission_capability_45ae1ad_20260724T203000Z`.

A separately approved retry completed four real
`openai-codex-oauth/gpt-5.6-sol` decisions:
`observe → turn_angle(30°) → move_distance(0.10 m) → stop`. The turn settled at
26.133 degrees with collision `CLEAR`; it crossed a floor seam and is treated
as bounded capability evidence, not a calibration measurement. The translation
settled at 0.117339 m. Requested and supervised motion agreed for both movement
intents, and the terminal progress recorded four intents, one observation,
26.133 degrees cumulative rotation, 0.117339 m cumulative translation, and
`planner_stop`. The one approval was authenticated by Tailscale and bound the
prompt, exact SHAs, proposal digest, limits, safety policy, and 15-minute lease.
Independent STOP, two zero motor samples, unchanged encoder samples, relock,
restart/no-resume, and process cleanup passed. Its sealed evidence is
`/home/jsperson/rvr_runs/adaptive_mission_capability_retry_45ae1ad_20260724T204000Z`.

## Archived manual calibration procedure

The remainder of this section records the pre-product manual calibration
workflow. Do not use its `RVR_LIVE_EXECUTION_*` toggling as product operation.
The supported product flow is the approval-activation configuration above:
generate a bound proposal while locked, then let authenticated approval start
and later stop the fixed supervised unit automatically.

Set `RVR_ADAPTIVE_MISSION_LEASE_S=900.0` for a default and maximum 15-minute
lease, or a smaller positive deployment maximum. The value cannot exceed 900
seconds. The browser's **Lease minutes** box may select any positive duration
up to that maximum; the selected value is passed into the proposal, OAuth
authority prompt, approval expiration, UI label, and terminal record.
Telemetry remains lease-managed and cannot be toggled off between replans or
authenticated objective updates. An objective update never extends the
original expiry. A model `stop` commands no further motion but keeps the
session and telemetry safely idle until that expiry, so the approving operator
can submit another objective without generating or approving a new lease.

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

### Adaptive mission attended extension

Do not select Adaptive mission during the straight, turn, or composed-route calibration
series. After those gates and the attended collision exercise pass, a separate
reviewed Adaptive mission session also sets `RVR_ADAPTIVE_MISSION_ENABLED=true`. The operator
reviews “Explore the room” as an adaptive 15-minute lease and approves once;
subsequent intents do not ask again. The page must show every snapshot,
rationale, requested/supervised movement, and revision. STOP, ESTOP, collision,
stale evidence, cancellation, timeout, provider loss, or restart ends that lease
without resumption. Relock both `RVR_ADAPTIVE_MISSION_ENABLED` and
`RVR_LIVE_EXECUTION_ENABLED` immediately after the attended run.

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
