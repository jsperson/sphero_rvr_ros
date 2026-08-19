# Design note (b): the recognition primitive — look_and_recognize

*Scott's ruling (2026-08-19, verbatim): "We need to build primitives right — the
ability to recognize things using the camera. Search would build on that
capability." One verb, built right, bench-certified before anything depends on
it. Design round only.*

## The verb

`look_and_recognize(target: str) -> RecognitionResult` — one on-demand
invocation, one structured answer, full provenance. No loops, no driving, no
mission logic inside the verb.

### Choreography (the camera charter, applied)

1. **Require stationary**: refuse unless the supervisor reports no commanded
   motion (the charter's intelligence-only clause — the camera NEVER runs
   concurrent with driving; the CPU/ToF-starvation receipt is why, and the new
   counter-based tof gate arithmetic is how the bench proves non-interference).
2. **Camera up**: start the camera stream (the pinned ~/.local libcamera +
   camera_ros path — the only working route; `camera.launch.py`).
3. **Snapshot**: grab N=3 frames, keep the sharpest (cheap Laplacian pick);
   decode STRIDE-SAFE via `sphero_rvr_core.image_decode.imgmsg_to_array` (the
   Pi 5's step=2432 shear trap — cv_bridge is forbidden here).
4. **Camera down**: stop the stream before anything else happens —
   snapshot-then-down, unconditionally, error paths included.
5. **VLM query**: the known-good config VERBATIM — `syn:large:vision`,
   `json_mode: true`, `max_tokens ~1500` (small returns null; short caps
   truncate JSON), key from the 1Password Hermes vault route. Prompt asks one
   question about one target.
6. **Structured answer**, schema test-pinned:
   `{seen: bool, where_in_frame: left|center|right|null, confidence: 0-1,
   description: str}` — parse failures are LOUD refusals, never empty results.

### Provenance (a result nothing can argue with)

Every result carries: photo path (saved frame), map pose at capture (TF at
frame stamp), capture yaw + derived TARGET BEARING (pose yaw + where_in_frame
mapped through the measured horizontal FOV), timestamp, VLM model id. A
"found it" without provenance is D24's lying-telemetry class wearing a camera.

### Constraints stated as constraints (not to be fixed here)

- **Fixed mount** (ruled 2026-08-09): FOV and low blind zone are what they
  are; bearing precision is bounded by where_in_frame granularity (~FOV/3).
- Cloud cost: one VLM call per invocation, by design visible in the verb's log.
- Hard sun: missions avoid it (standing constraint) — the verb inherits this
  as an operational note, not a code path.

### Absorption of ad-hoc practice

Existing camera one-offs (photos-to-Downloads, point-at-the-boot checks)
become INVOCATIONS of this verb — same choreography, same provenance, no
parallel camera paths left standing. One camera authority.

## Bench certification (before any mission depends on it)

Stationary or chassis-free; no motion authority anywhere in the plan:
1. **Contract receipts**: N known objects staged (the Dr Pepper bottle among
   them), M invocations each — recognition hit rate, where_in_frame vs true
   bearing, confidence distribution; plus N absent-target invocations (the
   false-positive rate IS the product number — a searcher that "finds" absent
   things is worse than none).
2. **Non-interference receipt**: /tof frames-counter rate during camera
   up/snapshot/down cycles vs quiet baseline — the counter-gate method, ≥10 s
   spans, band intact (the CPU lesson, now measurable exactly).
3. **Charter receipts**: camera provably DOWN after each invocation including
   induced error paths (stream absent on the graph); refusal fires when
   invoked while motion is commanded.
4. Structured-output contract: schema round-trip + truncation/null-return
   regression fixtures from the known-bad configs.

## Search, as one composition paragraph (proving the layering)

Search("bottle") = vantage selection over the current map (coverage-style
candidate viewpoints at VIEWPOINT_STANDOFF, facing unswept bearings) → drive to
each via NavigateToPose (the stock middle, all of today's machinery under it)
→ at each vantage, rotate through K headings via the PRECISE-TURN GATEWAY
(bench-passed, −1.3° class accuracy — the recognition sweep is a precision
consumer) invoking `look_and_recognize` at each heading → aggregate results →
report best find: found/not + photo + map location + bearing, provenance
attached. Nothing in that sentence is new except the aggregation — that is the
point of building the primitive right.

## Rollback / blast radius

The verb is a new leaf (one node or task-verb + one core module); nothing
existing consumes it until search composes it. Bench cert gates any mission
use; the charter gates any concurrency. No costmap, no marks, no motion
authority — zero overlap with the safety envelope.
