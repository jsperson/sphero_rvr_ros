<!-- COMMITTED 2026-08-11, HISTORICAL AND SUPERSEDED. The other approved-but-
     uncommitted design note found in the same scratchpad sweep. Committed for the
     same reason as design_d25_freeze.md: an approved design that is not in the repo
     cannot be diffed against the code, which is how drift survives review.

     STATUS: D27 is SUPERSEDED BY HARDWARE -- Scott ordered a rangefinder on
     2026-08-10 and the optical low-obstacle approach is retired. Its recommendation
     (reference-locality) was later MEASURED AND REFUTED: brake-band phantoms went
     79 -> 103, i.e. 30% worse, because the sun band runs diagonally and a per-column
     reference fixes the horizontal axis while worsening the vertical one. A third
     candidate, the height gate, also failed to separate. See the appendix of
     docs/stall_survival_ladder.md for all three dead candidates with numbers.

     Its D28 section (the "0.45 m blind zone" is not a geometric fact; nearest
     visible floor is 0.30 m) stands and is the reason no code changed for D28.

     The DEPLOYED camera layer is NOT to be removed or degraded until the rangefinder
     is validated side by side -- Scott's standing rule.

     Kept as written: this is the record of what was approved, not of what was true. -->

# Design note — D27 light-boundary guard, and D28 resolved

## D28 FIRST: NOT A BUG. The register's blind-zone number is wrong.

Measured with the deployed calibration (fx 680.56, fy 679.97, cx 538.17, cy 299.24,
camera height 0.1143 m, tilt 3 deg UP) through the real `pixel_to_ground`:

| image row | projects to |
|---|---|
| 599 (bottom) | **0.301 m** |
| 550 | 0.37 m |
| 500 | 0.48 m |
| 450 | 0.68 m |
| 400 | 1.20 m |
| 350 | 5.17 m |
| 320 | above horizon |

The bottom row lands at 0.301 m, so the 15 points at 0.3 m are the **geometrically
correct** projection of the bottom rows — not a span-math defect. Rows 511-599 (89
rows, 15% of the image) project inside 0.45 m.

**So the "~0.45 m near-blind zone" repeated in the register and in several of my own
commit messages is not a geometric fact.** Nearest visible floor is 0.30 m. I do not
know the provenance of 0.45; it may have come from focus, from detection reliability,
or from an earlier tilt/height estimate. Recommend the register records the measured
0.30 m and drops 0.45 unless someone can source it. **No code change for D28.**

**This matters for D27**, which is why it is first: the detector samples its floor
REFERENCE from the bottom 12% of the image — rows 528-599 — i.e. the strip from
**0.30 to 0.39 m in front of the robot**. Reference and detection region are different
patches of floor, and can be under different light.

## D27 root cause, exactly

`detect_obstacle_spans` computes `dist = ||pixel - median(bottom band)||` in RGB and
thresholds it. **On the NoIR camera R=G=B, so that Euclidean distance is just
|brightness difference| x sqrt(3).** Illumination and material are perfectly
confounded: there is no colour information left to tell "different stuff" from "same
stuff, different light".

Measured today: sunlit carpet mean 153.7 vs shadowed carpet mean 51.3 — a **3.0x**
step, ~102 grey levels, ~177 in RGB distance. The adaptive threshold is
`clip(2 x p95(band), 25, 60)` — **hard-capped at 60**. A sun edge exceeds the maximum
possible threshold by ~3x. No amount of adaptive-threshold tuning can reject it; the
clamp guarantees failure.

## Candidates evaluated AGAINST TODAY'S DATA (not preference)

**(a) Texture/gradient-shape discrimination — TESTED AND REJECTED.**
The theory: illumination scales intensity multiplicatively, so normalized texture
(CV = std/mean) is illumination-invariant; a light edge preserves it, an object edge
does not. I measured it on the two frames:

| patch | mean | CV | |lap|/mean |
|---|---|---|---|
| sunlit carpet | 153.7 | 0.123 | 0.125 |
| shadowed carpet | 51.3 | 0.332 | 0.074 |
| ratio | 3.0x | **0.37x** | **1.68x** |

If the model held, CV ratio would be 1.0. It is 0.37. Auto-exposure and gamma break
the multiplicative assumption, and sensor noise dominates CV in the dark patch. **The
elegant option does not survive contact with this camera.** Rejecting it here so
nobody re-proposes it later.

**(b) Brightness-ratio gating on the boundary.** Reject a boundary whose non-floor
side is much BRIGHTER than the floor reference. Rejects: sun patches (3x brighter),
reliably. Risks missing: a genuinely bright low obstacle — a white cable, a pale shoe
— in a dim room. That is a real safety hole in the exact class the camera layer
exists to cover, and it fails silently.

**(c) Reference-locality (RECOMMENDED).** The failure is not the threshold, it is that
ONE global median describes a floor whose illumination varies by 3x across the frame.
Compare each column against floor near THAT column instead: sunlit carpet is then
compared with sunlit carpet, and the step vanishes — while a real object still differs
from the floor directly beneath it, because that comparison is local and unaffected.

I attempted an offline check of (c) and **it was inconclusive, which I am reporting
rather than hiding**: my harness called `detect_floor_boundary` alone and got 800/800
columns flagged on BOTH frames, including the sun-blocked one. That is expected and
my harness's fault — every column genuinely has wall or door above the floor; the
discrimination happens downstream in the node's HEIGHT GATE
(`min_obstacle_height_m` 0.02 to `max_obstacle_height_m` 0.20). A valid offline test
must replay the whole chain: spans -> `object_height_m` -> height gate -> range gate.

### Recommendation: (c) reference-locality, with (b) as a bounded secondary

Primary: replace the single global `floor_reference` with a per-column local
reference (median of the bottom band over a +/-N column window). Minimal change,
lives in the pure core, keeps every existing knob.
Secondary, only if (c) proves insufficient: a **signed, generous** brightness gate —
reject a boundary only when the non-floor side is >2.5x the local reference AND the
span's height estimate is implausible for a real object. Two conditions, so a pale
shoe (bright but with a plausible height and a local floor contrast) survives.
I am NOT proposing (b) alone, for the safety-hole reason above.

**Honest risk in (c):** if the sun edge runs ACROSS the reference band itself, the
local reference near those columns is sampled from a mix of lit and shadowed floor
and the median may sit between them, leaving a residual step. The four-cell protocol
below will show that directly — it is the case most likely to fail, so it must be
tested, not assumed.

## Bench protocol — four cells, one session, no chassis motion

Rover stationary, sunlit scene reproduced (late afternoon, same room/pose). Measure
**brake-band point count (0.40-0.50 m)** and **total obstacle points**:

| | no shoe | real shoe at ~0.45 m |
|---|---|---|
| **sun on** | must be **0** brake-band points | shoe **detected** |
| **sun off** (towel) | must be 0 | shoe detected |

Acceptance: top-left cell goes 72 -> 0 while both right-hand cells still detect. A fix
that zeroes the phantoms and also loses the shoe is a regression, not a fix — that is
the whole point of the second column.
Additionally, capture a frame + cloud for each cell so the fixtures are reusable.

**Cheaper and available BEFORE the sun returns:** today's two frames
(`rvr_cloudcheck_183001.png` sun-lit, `rvr_sunblocked_183447.png` blocked) are already
a matched pair differing only in light. Building the full-pipeline offline replay
(spans -> height gate -> projection -> brake-band count) turns them into a permanent
regression fixture, so the fix can be validated tonight and the sunlit session only
has to add the shoe cells. **I recommend building that replay first** — it is the
difference between validating in one session and waiting on weather.

## Scope

In: the detector guard (pure core + its node params), the offline replay fixture, the
register correction for the 0.30 m geometry. Out, per your ruling: camera->planning
marking, and any change to freeze-as-sensor. No chassis motion required for any of it.
