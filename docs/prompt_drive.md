# Prompt-driven rover MVP

`rvr_prompt_drive` converts one natural-language command into a bounded route
proposal using Codex CLI authenticated with ChatGPT OAuth on the Raspberry Pi.
Proposal-only is the default: it does not import ROS, publish a route, or move
the rover.

All operational components run on the Pi. The Mac is development/test only and
must not hold the rover OAuth session or host a motor-capable rover runtime.

## Pi OAuth setup

From the deployed Pi checkout, install the prompt-drive prerequisites:

```bash
scripts/install-rvr-prompt-drive
```

The installer adds the official standalone Codex CLI and the Linux `bubblewrap`
sandbox prerequisite. It does not authenticate automatically. Complete the
headless ChatGPT OAuth flow explicitly:

```bash
codex login --device-auth
codex login status
```

The required status is `Logged in using ChatGPT`. Direct Platform API keys are
not accepted by the Pi prompt-drive provider. Codex CLI owns OAuth token refresh;
its credential store must be treated as a secret and must never be committed or
included in run manifests.

## No-motion proposal

On the authenticated Pi, source the ROS workspace and run:

```bash
rvr_prompt_drive "Move forward 20 centimeters, turn left 90 degrees, then stop."
```

The default model is `gpt-5.6`, and the default reasoning effort is `high`.
Use `--model` or `--reasoning-effort` to select another Codex-accessible model.
Each proposal invocation is ephemeral and disables shell, unified exec, apps,
MCP, multi-agent, and web-search tools. The generated manifest is written under
`artifacts/prompt_drive/` and contains no credential material.

The model can select only these physical effects:

- positive/forward `move_distance`, with at most 0.5 m cumulative translation;
- signed `turn_angle`, with at most 180 degrees per call;
- no more than three motion calls.

Linear speed, angular speed, timeouts, runtime, and collision policy are trusted
executor settings. They are not model parameters.

## Physical execution

Physical execution is a separate, explicit mode:

```bash
rvr_prompt_drive \
  "Move forward 10 centimeters, then stop." \
  --execute \
  --operator operator:scott
```

The command prints the complete physical proposal and a SHA-256 digest. It then
requires an interactive terminal and the exact phrase `APPROVE <full-digest>`.
Any proposal change invalidates approval. There is intentionally no noninteractive
approval flag.

After approval, the client waits for both live-route request and status graph
connections, publishes only to `/mission_api/v2/live_route/request`, and waits
for a correlated terminal manifest. It never publishes `Twist`, raw motor data,
or `/cmd_vel_motor`. The existing live route runner publishes `/cmd_vel` above
the collision supervisor and measures completion from odometry.

If terminal status times out, the client requests `live_route/cancel`. If the
cancel acknowledgement cannot be confirmed, the command reports that physical
STOP/ESTOP must be used rather than claiming cleanup succeeded.

Do not use `--execute` until the rover environment, collision supervisor,
odometry, STOP, and ESTOP have been checked under the staged MVP0 procedure.
