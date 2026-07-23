# Mission web console

`src/sphero_rvr_driver/mission_web.py` provides the map-driven browser slice for
the rover product. It combines natural-language mission entry, typed proposal
review, explicit digest-bound approval, mission and safety state, event history,
and a responsive map in one page.

The browser never talks to ROS, serial devices, motor controls, or OpenAI. Its only
boundary is `MissionWebAdapter`.

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

Stage B uses the same loopback browser, digest confirmation, and persistent
`MissionService` event/result seam, but replaces the fixed fixture route with
one finite leased intent at a time:

```bash
rvr_mission_web \
  --mode rolling-replay \
  --host 127.0.0.1 \
  --port 8876 \
  --replay-database artifacts/perception_action_stage_b/replay.sqlite3 \
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
longer copies or enters a GUID, digest, code, or hash. The raw digest is available
only in the collapsed **Technical approval audit** detail.

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

The normal Pi configuration reports `live_execution_enabled=false`; therefore the page is
marked `LIVE - PROPOSAL ONLY / EXECUTION LOCKED`, proposal-only OAuth planning is
available, and approval is disabled. The web command cannot enable motion. An
attended operator may use the separately reviewed Pi configuration gate described
in `docs/pi_mission_stack.md`; the service then installs the deterministic route
executor only when the reviewed, source, and deployed SHAs match. Even in that mode,
the UI and server keep approval disabled until authoritative odometry and safety
evidence are fresh and clear. Missing STOP/ESTOP evidence renders `UNKNOWN`.

## HTTP routes

```text
GET  /api/web/state
GET  /api/web/scenarios
GET  /api/web/artifacts/terminal-result   # terminal missions only
POST /api/web/mission/propose
POST /api/web/mission/approve
POST /api/web/mission/advance   # mock/replay only
POST /api/web/mission/cancel
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
  tests/test_live_mission_service.py tests/test_package_metadata.py
```

The suite covers the typed adapter boundary, every replay outcome, mock exact
approval, live server-owned digest binding, proposal-only state, authoritative
Tailscale approval identity, origin and cross-site rejection, unavailable live
maps, forbidden routes, persistence, and restart recovery.
