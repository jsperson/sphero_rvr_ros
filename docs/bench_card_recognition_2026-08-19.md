# BENCH CARD — the recognition primitive (look_and_recognize)

*Certification before any mission depends on it (design + consensus
2026-08-19). Zero motion authority anywhere on this card: the rover never
receives a motion command. ~25 minutes of Scott's staging.*

## Chassis state (the call, stated per the ruling)

**Chassis ON, rover STATIONARY, full stack's sensing half up** (driver +
lidar + supervisor + tof — no Nav2 needed). Reason: the stationarity gate
reads the SUPERVISOR'S state line, and certifying the verb with
`require_stationary:=false` would leave the gate itself uncertified. The
fail-closed branch is certified live (R1c); the commanded-motion branch stays
unit-pinned — certifying it live would require commanding motion, which this
card forbids by design. Stated honestly here so nobody reads R1 as more than
it is.

Node under test: `ros2 run sphero_rvr_driver recognition` (target set via
`ros2 param set /recognition target "<thing>"`, invoked via
`ros2 service call /recognition/look_and_recognize std_srvs/srv/Trigger`).

## What Scott stages

- THREE target objects: the Dr Pepper bottle + two of his choosing (distinct
  colors/shapes), placed one at a time ~1.2 m ahead of the parked rover in
  clear view.
- Tape marks at lateral offsets from the camera axis at that range: 0, ±0.15,
  ±0.30, ±0.45, ±0.60 m (for R4's FOV measurement — ~7° steps at 1.2 m).
- The room otherwise as-is; ambient light, no hard sun (standing constraint).

## Items, in order (refusals FIRST — falsifier before certifier)

### R1. THE REFUSAL LADDER refuses, loudly, before the happy path is trusted

(a) no target set → invoke → **PASS: "REFUSED: no target set..."**, and no
camera process ever appears. (b) `api_key_file` pointed at a missing path →
**PASS: refusal names the PATH and never any contents; no camera**; restore.
(c) kill the supervisor's state feed (stop the supervisor node) → invoke →
**PASS: the fail-closed stationarity refusal**; restart supervisor.
FAIL on any silent no-op or camera activity during a refusal = stop the card.

### R2. HIT RATE on staged objects

Each of the 3 objects, centered, 3 invocations each (9 total).
**PASS: ≥ 8/9 seen=true with a sane description; 9/9 schema-valid** (any parse
refusal is investigated, never shrugged — the contract is the product).

### R3. ABSENT-TARGET FALSE-POSITIVE RATE — the product number

Against the same staged scenes, 6 invocations for objects NOT present
(Scott picks absurd-but-plausible: "banana", "red shoe", ...).
**PASS: 0/6 seen=true. ZERO, not low** — a searcher that finds absent things
is worse than none. One false positive → the prompt/confidence design gets a
consensus round before any mission use; the card continues for data.

### R4. FOV MEASUREMENT — 66° vendor number becomes a measured constant

One object walked across the tape marks (9 positions), one invocation each.
Record where_in_frame per position. **PASS: monotonic right→center→left with
stable sector boundaries; derive the measured HFOV** (boundary offset ×
range arithmetic) — the `HORIZONTAL_FOV_DEG` constant gains the measured
value + this card as receipt in a reviewed commit. No pass bar on the VALUE
(it is whatever it measures); the bar is monotonicity + repeatability.

### R5. ToF NON-INTERFERENCE — the counter method (the CPU lesson, measured)

tof runs the whole card. Frames-counter rate over ≥10 s spans DURING
camera-up/snapshot/VLM windows vs a quiet-stack baseline span.
**PASS: every span ≥ 6.5 Hz (the band does not move); report the delta vs
baseline** — the number goes in the card whatever it is.

### R6. CAMERA-DOWN RECEIPTS, including induced error paths

After EVERY invocation above: camera process absent (`ps -eo pid,comm`, no
camera node on `ros2 node list`). Then three induced failures: (a) bad
`base_url` (VLM query fails) — camera down after; (b) `image_topic` pointed at
a silent topic (frame timeout) — refusal + camera down after; (c) restore
everything, one clean invocation as the recovery proof.
**PASS: camera down after every path with no manual cleanup, ever.**

## After the card

All pass → the primitive is certified for composition (the search design round
may begin); FOV constant updated with the measurement; results recorded here
verbatim. Any fail → register row with the receipts, primitive stays
bench-only. Nothing on this card changes flight behavior either way — the
node is a leaf no mission invokes yet.
