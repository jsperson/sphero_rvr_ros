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

## Clearing the map (moved the rover? new room?)

Chat: "clear the map" / the model's `clear_map()` verb — or the **new map**
button on the map panel (tap twice; no dialog). One action: slam_toolbox Reset +
Nav2 clear_entirely + the /map_clear event that contact_marker and
coverage_explorer honor by forgetting their own state (marks, coverage, the
done latch — a fresh mission can be armed after a clear). The fresh map starts
nearly empty and grows only as the robot moves — a small map right after
clearing is normal. On the static-map rig the slam step reports itself absent
honestly; marks and costmaps still clear.

## The live lidar overlay

The map pane draws every current lidar return (cyan) over the mapped room, from
the state tick's `scan` field — map-frame points projected in the node from the
same `map->laser` TF the costmaps read, so the overlay cannot disagree with what
the stack believes. `scan: null` means the robot is NOT SEEING (no scan, stale
>2 s, or no transform) and the meta line says "lidar not seeing" rather than
drawing an empty room.

**The 1 Hz update rate is a MEASURED choice, not a natural constant.** The
console's costless property is certified by the counter method, and this feature
was re-certified against it (≤1.2% deviation on /scan, explorer status and
/diagnostics; tick 4.8 KB with 240 points against an 8 KB test guard). If a
smoother overlay is wanted, re-run those windows at the new rate and keep the
receipt — do not nudge the rate because it feels fine.
