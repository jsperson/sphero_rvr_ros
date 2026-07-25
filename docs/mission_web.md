# Mission web console

`src/sphero_rvr_driver/mission_web.py` provides the map-driven browser slice for
the rover product. It combines natural-language mission entry, typed proposal
review, explicit digest-bound approval, mission and safety state, event history,
and a responsive map in one page.

The browser never talks to ROS, serial devices, motor controls, or OpenAI. Its only
boundary is `MissionWebAdapter`.

## Operations-console layout

The interface treats spatial and visual evidence as the primary operating view.
On desktop, a wide main column contains the live map and then the camera at the
same width; the narrower operations sidebar scrolls independently. The safety
strip remains above both columns. At tablet and mobile widths, the page becomes a
single column in this order: safety state, map, camera, one mission status,
mission controls, and a running mission log, then collapsed evidence and diagnostics. The page does
not require horizontal scrolling.

The map renders available occupancy or obstacle evidence, rover pose and heading,
route and traveled path, goal, safe corridor, and semantic tracks with confidence
and uncertainty. Localization state, quality, snapshot identity, source, and
freshness are attached to the map. Stale, degraded, and unavailable spatial
evidence receives a textual overlay as well as a status color.

The camera preserves the supplied frame aspect ratio with `object-fit: contain`
and overlays supplied detection boxes only when frame pixels and dimensions
exist. Frame ID, source timestamp, receive timestamp, freshness, observation
focus, viewpoint, and detections remain visible. Metadata-only replay evidence,
an unavailable source, an interrupted stream, and a stale last frame are distinct
states; the browser never invents pixels or detections.

Safety state and active warnings are never placed in collapsed details. Mission
status, prompt, submission, approval/cancel controls, and the combined mission
log remain in the operational flow. World snapshots, terminal evidence,
event history, and lower-level authority diagnostics use collapsed disclosure
panels so long histories do not displace the map and camera.

## Mock/replay mode

Run locally from an editable development install:

```bash
python -m sphero_rvr_driver.mission_web --host 127.0.0.1 --port 8765
```

Or use the installed command:

```bash
rvr_mission_web --host 127.0.0.1 --port 8765
```

Open `http://127.0.0.1:8765/`. The page is unmistakably marked `MOCK / REPLAY`.
The deterministic fixtures cover success, model rejection, cancellation, STOP,
ESTOP, collision blocking, and stale telemetry. An executable proposal remains
`PROPOSED` until the exact `APPROVE <digest>` phrase is entered. The approved route
is discarded because the mock adapter has no physical executor.

## Rolling LLM replay mode

Rolling replay uses the same loopback browser, digest confirmation, and persistent
`MissionService` event/result seam, but replaces the fixed fixture route with
one finite leased intent at a time:

```bash
rvr_mission_web \
  --mode rolling-replay \
  --host 127.0.0.1 \
  --port 8876 \
  --replay-database artifacts/perception_action_replay/replay.sqlite3 \
  --replay-reasoning-effort none
```

This mode requires the real local Codex CLI boundary to report `Logged in using
ChatGPT`. Each provider call receives a versioned fresh world snapshot and must
return one typed steering, speed, corridor, observation-focus, viewpoint, and
finite-lease update. The replay clock, localization, camera detections,
object/face tracks, semantic map, and simulated pose keep advancing in a
separate thread while the provider call runs. Valid compatible intents replace
one another atomically; deterministic code inserts no zero command.

An introduced obstacle makes the clear-left corridor mandatory. An uncertain
shoe track requires an alternate-left observation focus. These are validation
constraints over the model's decision, not a substituted rules planner. Stale
localization, lease expiry, collision, STOP, ESTOP, or cancellation may command
simulated zero immediately without waiting for the provider.

The mode never imports ROS, exposes a publisher, binds a physical adapter, or
enables live execution. Terminal JSON records decision snapshots, provider
identity, intent revisions and rationales, path, command samples, detections,
semantic tracks, concurrency metrics, and the deterministic stop reason.

## Pi live/proposal-only mode

`LiveMissionWebAdapter` connects only to the user-only Pi mission-service Unix
socket. It submits planning requests, polls durable service state, forwards an
authenticated approval confirmation and cancellation requests, and renders only
authoritative evidence. It does not contain planning or safety logic. For live
missions the operator reviews the route and clicks **Approve and run** once; the
Pi then reloads the persisted proposal, recomputes the exact full-digest approval
phrase, and sends it through the existing mission-service approval boundary.
The digest binding and approval audit remain server-owned, but the operator no
longer copies or enters a GUID, digest, code, or hash.

```text
browser
  -> authenticated Tailscale HTTPS
  -> loopback rvr_mission_web
  -> LiveMissionWebAdapter
  -> MissionServiceClient (Unix socket)
  -> persistent MissionService
```

Start live mode only behind a reviewed HTTPS origin:

```bash
rvr_mission_web \
  --mode live \
  --host 127.0.0.1 \
  --port 8765 \
  --mission-socket "$HOME/.local/state/sphero_rvr/mission-service.sock" \
  --session-id rvr-web-console \
  --operator tailscale-serve \
  --public-origin https://sphero-pi-2.example-tailnet.ts.net
```

Live mode remains loopback-only and requires an exact `Origin` plus same-origin
fetch metadata on state-changing requests. It also requires the
`Tailscale-User-Login` header injected by Tailscale Serve, and records that identity
as the approval operator. Tailscale Serve strips spoofed identity headers before
injecting authenticated tailnet identity when proxying to localhost. Do not place
another untrusted proxy between Serve and this listener, do not expose the port on
the LAN, and do not use Tailscale Funnel.

The product Pi configuration reports approval-time activation capability while
the physical session remains locked. Proposal generation persists only the
prompt and exact reviewed envelope; it does not start sensors or make a stale
model request. **Approve 15-minute lease** is the authenticated authorization
boundary: it starts the fixed supervised graph, waits for fresh authoritative
camera/lidar/localization and safety evidence, and only then invokes the model.
Missing or unsafe STOP/ESTOP/collision evidence prevents execution. Every
terminal path stops the graph again, and a relock failure is shown as
`RECOVERY_REQUIRED`.

When stationary or adaptive perception is configured, the safety strip includes
an authenticated **Camera + lidar** telemetry toggle. It starts only the fixed
no-motion `rvr-telemetry.service` user unit and can stop both that unit and the
explicit-start adaptive mission unit that may own the same sensors. It never enables either unit at boot
and exposes no arbitrary service name or shell command. Startup is rejected
unless the service snapshot proves live execution, physical execution, and
motion authority are all false. A runtime preflight also refuses startup when
a driver, route/motion process, motion-topic publisher command, or rover UART
owner is present. The unit contains lidar, Camera 3, stationary SLAM, tracking,
and semantic perception only—no rover driver, route executor, serial transport,
motor graph, or motion publisher. Sensor state and startup errors remain
visible. Shutdown first calls the fixed RPLidar `/stop_motor` service, then sends
`SIGINT` through the ROS launch so the driver executes its motor-stop cleanup.
The web request does not report a verified stop until systemd is inactive/dead,
sensor descendants and the `/dev/rplidar` owner are gone, and authoritative map
and camera evidence is no longer fresh. A timeout or failed postcondition remains
visible as a degraded lifecycle error rather than `Off`; retained pixels and
metadata are explicitly labeled stale.

Proposal, stationary confirmation, cancellation, and sensor requests disable
their controls while in flight and announce progress, success, or a specific
error through live regions. A disabled stationary confirmation names the missing
prerequisite: a proposal, the fixed sensor unit, or fresh lidar/camera/
localization/semantic-map evidence. Live stationary pages continue polling after
a terminal mission so newly requested map and camera evidence can still appear.
Read-only live polling also continues through a temporary Pi or network
interruption and clears the connection error when authoritative state becomes
reachable again; mock/replay mutation polling still stops on an error.

## HTTP routes

```text
GET  /api/web/state
GET  /api/web/scenarios
GET  /api/web/artifacts/terminal-result   # terminal missions only
POST /api/web/mission/propose
POST /api/web/mission/approve
POST /api/web/mission/advance   # mock/replay only
POST /api/web/mission/cancel
POST /api/web/telemetry  # boolean active; fixed service targets only
```

Direct motor, arbitrary write, ROS, `/cmd_vel`, and `/cmd_vel_motor` routes are
rejected. Responses use `Cache-Control: no-store`, the page uses no local or
session storage, and credentials are never accepted by the adapter.

The terminal artifact endpoint returns only the current mission's persisted JSON
result through the same adapter snapshot. It accepts no path, reads no arbitrary
file, and returns an error before terminal evidence exists. The page renders that
result and links to the no-store JSON response so final heading, per-track
measurements, collision state, and terminal reason remain reviewable.

## Map truthfulness

Mock mode renders fixture-backed pose, route, path, obstacles, and semantic
objects. Live mode renders each layer only when it is supplied by fresh,
authoritative mission-service data. Missing or stale semantic-map evidence is
shown as unavailable; fixtures are never substituted into a live view. The map is
only a visualization and does no browser-side inference or planning.

Rolling replay may include camera frame identity and detection metadata
without image bytes. In that case the camera explicitly reports that pixels were
not supplied while retaining the evidence timestamp and frame ID. Live
stationary perception may supply the latest JPEG preview and tracked detections
through the same additive adapter fields.

## Adaptive mission closed-loop modes

Adaptive mission adds `--mode adaptive-mission-replay`. Proposal generation calls the real
Codex/ChatGPT OAuth provider with the starting typed world snapshot and shows
the interpreted objective plus the first bounded intent before approval. One
digest-bound approval starts the 15-minute replay lease. The page then shows
each updated snapshot, intent and rationale, requested and collision-supervised
movement, cumulative travel, remaining lease, and terminal outcome.

The normal Adaptive mission CLI requires `--public-origin`, exact same-origin POSTs, and
an authenticated `Tailscale-User-Login` supplied by Tailscale Serve. The
server-owned identity and authentication source are stored with the proposal
digest. Supplying that header directly to a plain loopback server does not
authenticate approval. `--allow-loopback-test-approval` is an explicit
repository/browser-harness escape hatch for replay tests only; it cannot be
combined with a public origin.

The `adaptive-mission-replay` mode is deliberately replay-only. Its executor implements
the same `AdaptiveMissionExecutor` boundary as the production physical adapter, while
`live_execution_enabled`, `physical_execution_enabled`, and `motion_authority`
remain false. See `adaptive_mission_controller.md`.

In live mode, a mission service configured with `adaptive_mission_enabled=true` exposes
the same proposal and rolling projection. The page labels the deployment as
locked or supervised-live from authoritative service state, shows the active
900-second lease, every world snapshot and LLM revision, and requested versus
supervised velocities. When the moving perception producer is present, it also
shows whether semantic perception is fresh and localized plus counts of object,
known-face, and unknown-face tracks; the authoritative camera and map panels
continue to show the underlying evidence. It can submit, authenticate one
Tailscale approval, poll, and cancel. It cannot choose an intent, publish a
topic, enable either deployment gate, or own motion. Packaged configuration
keeps Adaptive mission and live execution false.

## Installed Pi services

The package installs:

- `rvr-mission-service.service`, the persistent owner with execution default off;
- `rvr-mission-web.service`, the loopback live/proposal-only UI;
- `mission-stack.env.example`, which contains SHAs, port, and public origin only;
- `install-rvr-mission-stack-services`, which installs but does not enable or
  start the units. Its execution variables default to `false` and blank.

No OAuth token belongs in the environment file. The planner uses the Pi-local
Codex CLI session, and `codex login status` must report `Logged in using ChatGPT`.

## Validation

Run focused tests only through the bounded runner:

```bash
python3 scripts/run_pytest_bounded.py --timeout 60 -- -vv \
  tests/test_rolling_replay.py tests/test_mission_web.py tests/test_prompt_mission_controller.py \
  tests/test_live_mission_service.py tests/test_adaptive_mission_controller.py \
  tests/test_adaptive_mission_physical.py tests/test_adaptive_mission_live_controller.py \
  tests/test_package_metadata.py
```

The suite covers the typed adapter boundary, every replay outcome, mock exact
approval, live server-owned digest binding, proposal-only state, authoritative
Tailscale approval identity, origin and cross-site rejection, unavailable live
maps, forbidden routes, persistence, and restart recovery.
