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
proposal-only: `live_execution_enabled=false`, no route executor is installed,
and the two systemd units start no motor-capable process.

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
  tests/test_prompt_drive.py tests/test_package_metadata.py
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

## Physical execution remains a separate gate

This runbook does not start or validate physical execution. A later reviewed
change must bind the deterministic route executor without bypassing the collision
supervisor and must first resolve route-local versus absolute-odometry reporting.
Each restrained 10 cm translation, 45-degree turn, and multi-step prompt requires
fresh operator approval of its exact proposal digest while the operator is
present.
