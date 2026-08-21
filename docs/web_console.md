# The web console — running it

Design + ratifications: `docs/design_web_console_2026-08-20.md`. Built and
certified overnight 2026-08-20 (batches A–E). LAN-only, no auth — v1 scope.

## On the rover (or the rig)

```bash
# beside a live stack (needs task_node for chat; map/status/stop degrade honestly)
ros2 run sphero_rvr_driver web_console
# then open http://<pi-hostname>:8088/ on a phone or anything with a browser
```

Watch-only mode: without `~/.config/synthetic/api_key` the console still serves
map/status/stop; instructions answer 503 saying exactly that.

Flags mirror `task_client` (`--model`, `--max-tool-calls`, …) plus `--host`,
`--port`, `--photo-dir`.

## Morning-demo shape (chassis-free)

```bash
ros2 launch sphero_rvr_driver sim_closed_loop.launch.py start_coverage_explorer:=true
ros2 run sphero_rvr_driver task_node
ros2 run sphero_rvr_driver web_console
```

Type an instruction in the chat ("explore", "where are you?"). NOTE the honest
limit from the design note: the RIG's map is a recorded map served static — the
pose moves, missions run, the map does not grow here. The growing map is the
flight case (slam_toolbox publishes the same `/map` the console renders).

## UI development without a robot

```bash
python3 scripts/fake_web_console.py --port 8090
```

Real production HTTP server + pure core, fake map/pose/transcript. Any POSTed
instruction replays a canned mission with every ending class the chat renders
(tool calls, a look card, model-failure, say, budget). Iterate on
`src/sphero_rvr_driver/web_static/` and refresh.

## The stop button is not an e-stop

It cancels whatever `task/goto` is doing (anyone's goal, via the action's
standard cancel door), then calls `task/stop`. The robot coasts to a halt; the
collision supervisor is untouched and owns real stopping. The UI says this too.

## Measured load (counter method, 2026-08-20, rig)

60 s windows on the Pi; bar was ±5% of nominal:

| window | condition | /scan Hz | status Hz | /diagnostics Hz |
|---|---|---|---|---|
| A | idle, 0 clients | 10.000 | 0.967 | 21.18 |
| B | mission DRIVING + 2 SSE + 1 Hz map poll | 10.000 | 1.000 | 31.68 |
| C/D2 | idle, 0 clients (repeats) | 9.98–10.0 | 0.97–1.0 | 21.2–22.0 |
| E | idle + 2 SSE + 1 Hz map poll | 9.983 | 0.967 | 22.00 |

`/scan` and explorer status held nominal in every window (worst 0.8%).
Diagnostics with clients at idle (E) equals idle baseline exactly; window B's
+50% diagnostics is the driving mission's own nav2 traffic, isolated by C/D2/E.
**The console's presence costs the pipeline nothing measurable.**
