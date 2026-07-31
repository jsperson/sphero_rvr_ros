# Milestone 8 Phase 0A — existing physical drive-trace analysis

## Outcome

The existing synchronized trace is sufficient to select the next fix; a Phase
0B speed sweep is **not required before Phase 1**. The run ended because the
second semantic decision returned `finish` while the first Nav2 leg was still
active, not because Nav2 completed the leg or declared a progress failure.
Controller completion arrived 7.293 seconds after motion began, against the
active goal's initial 13.500-second ETA and 1.350 m remaining distance.

The only goal was also 145.0 degrees behind the rover. Nav2 therefore spent
77.79% of the available motion interval asking for pure rotation and only
22.21% asking for forward motion. The downstream motor stream contained only
1.288 seconds of forward command, split across nine windows. The configured
0.10 m/s floor reached the motor stream for 0.907 seconds total, but no one
continuous floor window exceeded 0.284 seconds. This evidence cannot establish
the loaded-surface physical breakaway threshold.

Phase 1 should implement one deterministic **active-leg finish-eligibility**
fix: a semantic `finish` must not terminate an active, safe Nav2 leg before the
leg has produced a material traversal outcome. Safety vetoes, cancellation,
lease/freshness failures, and explicit operator STOP/ESTOP remain immediately
authoritative. Candidate forward-bias remains a separately ranked follow-up,
not a second change bundled into that fix.

## Provenance and reproducibility

No robot run, service start, behavior change, configuration change, or motor
command was made for this analysis.

- Private trace on `sphero-pi-2`:
  `/home/jsperson/.local/state/sphero_rvr/drive-diagnostics/drive-trace-m7-canonical-e90f7828e13843d981eab942b16751a4.jsonl`
- Mode/size: `0600`, 2,793,335 bytes
- Raw SHA-256:
  `54a6af978736c1f1dfac2fe405b20b7a0049429b175d5bcb6ee146172dbc49a0`
- Mission: `m7-canonical-e90f7828e13843d981eab942b16751a4`
- Executable/deployed source:
  `c8cbff35d156332806f0fe8d16b47b23514eac6d`
- Context canonical SHA-256:
  `ef7d589c38b8ca2a8785b125dd50a5b1ed0805199c37cad102f22818449952c7`
- Goal-dispatch payload SHA-256:
  `66a0b304c07db49b0b3b9458a5b9eeea11a599f372cbed5a7baf2839d2612b98`
- Semantic-finish payload SHA-256:
  `901d962ebfbb244152171d47dc094eb374b23af567e0b4191f47fb6388c888f9`

The ROS-free analyzer is `src/sphero_rvr_driver/drive_trace_analysis.py`.
The committed derivatives are
`artifacts/m8_phase0_drive_trace_analysis/context.json` and `report.json`.
The report is regenerated with:

```bash
PYTHONPATH=src python3 -m sphero_rvr_driver.drive_trace_analysis \
  /path/to/private-trace.jsonl \
  --context artifacts/m8_phase0_drive_trace_analysis/context.json \
  --output artifacts/m8_phase0_drive_trace_analysis/report.json
```

The working evidence branch was reconciled as follows: branch HEAD
`5f86db41a103cac2d32e6d275af796a11ee59376` is the evidence-only child that
committed `artifacts/physical_drive_jitter_comparison/report.json`.
`origin/main`, local `main`, the Pi checkout, installed package, and all deployed
SHA bindings remain the executable parent
`c8cbff35d156332806f0fe8d16b47b23514eac6d`. The extra evidence commit neither
changed nor replaced the deployed executable.

## Alignment method

Two intervals are reported:

1. The mission-bound controller interval begins with its first status carrying
   this mission ID and ends at the first `complete` status: 27.533 seconds.
2. The active-navigation interval begins at the first nonzero Nav2 request and
   ends at that same controller completion: 7.293 seconds.

Command samples are converted to time-weighted, zero-order-hold segments. A
command expires to zero after 0.500 seconds; collision state expires after
0.300 seconds. `linear_x > 0.001 m/s` is forward and a command with no linear
component but `|angular_z| > 0.01 rad/s` is pure rotation. Odometry uses the
last sample at or before a window start and the first sample at or after the
window end plus a fixed 0.250-second response allowance. This avoids invalid
per-sample comparisons between differently sampled command and odometry topics.

The per-window response intervals can overlap, so their displacement values are
diagnostic and **must not be summed**. Encoder counts are unavailable in the
trace. Discrete angular jerk is a finite difference across command-value
changes; it is useful for same-pipeline comparisons but is sample-timing
sensitive. Yaw per metre is similarly unstable when net translation is only a
few centimetres.

## Command-time results

Across the full 27.533-second mission-bound controller interval:

| Stream | Forward | Pure rotation | Zero | Motion starts |
|---|---:|---:|---:|---:|
| Nav2 request | 5.88% | 20.61% | 73.51% | 1 |
| Supervisor request | 4.75% | 1.83% | 93.42% | 16 |
| Motor output | 4.68% | 1.51% | 93.81% | 19 |

Across the 7.293-second interval during which Nav2 was actually active:

| Stream | Forward | Pure rotation | Zero | Angular nonzero duty | 0.35-floor duty | Motion starts |
|---|---:|---:|---:|---:|---:|---:|
| Nav2 request | 22.21% | 77.79% | 0.00% | 97.23% | 0.00% | 1 |
| Supervisor request | 17.94% | 6.90% | 75.16% | 22.02% | 6.90% | 16 |
| Motor output | 17.66% | 5.70% | 76.64% | 20.94% | 5.70% | 19 |

The Nav2 request is continuously held during the active interval, whereas the
bridge and motor stream pulse between motion and zero. Collision state was
`CLEAR` for 6.792 seconds, `SLOW` for 0.365 seconds, and `SENSOR_STALE` for
0.136 seconds. The first three forward windows were mostly `SLOW`; that explains
their 0.0064–0.0744 m/s downstream values without contradicting the CLEAR-only
0.10 m/s floor.

## Forward windows and aligned odometry

| Window | Duration (s) | Mean linear (m/s) | Mean angular (rad/s) | Collision context | Aligned net odom (m) |
|---:|---:|---:|---:|---|---:|
| 1 | 0.184 | 0.0064 | -0.400 | 0.173 s SLOW | 0.0256 |
| 2 | 0.049 | 0.0743 | -0.109 | 0.047 s SLOW | 0.0020 |
| 3 | 0.148 | 0.0744 | -0.391 | 0.142 s SLOW | 0.0037 |
| 4 | 0.284 | 0.1000 | -0.327 | CLEAR | 0.0232 |
| 5 | 0.242 | 0.1000 | -0.305 | CLEAR | 0.0357 |
| 6 | 0.122 | 0.1000 | -0.400 | CLEAR | 0.0016 |
| 7 | 0.057 | 0.1000 | -0.400 | CLEAR | 0.0020 |
| 8 | 0.176 | 0.1000 | 0.000 | CLEAR | 0.0031 |
| 9 | 0.026 | 0.1000 | -0.400 | CLEAR | 0.0085 |

The active interval's overall odometry was 0.0311 m net, 0.1244 m path length,
and 2.239 rad absolute yaw. That is 71.89 rad of absolute yaw per net metre.
Only window 8 was straight, and it lasted 0.176 seconds. The short, mostly
turning windows and overlapping odometry response intervals do not support a
claim that 0.10 m/s either reliably breaks away or reliably stalls. They do
show that failure to apply the configured CLEAR floor was not the cause.

## Jitter metrics

Within active navigation, downstream angular sign reversals remained zero, so
that saturated metric does not describe the visible stop/start behavior.

- Motor angular nonzero duty: 20.94%; exact 0.35 rad/s floor duty: 5.70%.
- Motor motion starts: 19 in 7.293 seconds (2.61 starts/s).
- Motor angular command changes: 38; total variation: 12.673 rad/s.
- Motor discrete angular jerk: 455.47 rad/s³ RMS, 1451.66 rad/s³ maximum
  absolute value across 37 finite-difference samples.
- Absolute yaw per net metre: 71.89 rad/m, explicitly high-variance because net
  translation was only 0.0311 m.

These are the comparison metrics Phase 1 should improve while preserving zero
downstream sign reversals.

## Ranked causes

1. **The active first leg was terminated early by semantic completion.**
   Prefetch generation 2 started as the first goal was dispatched. At that point
   the goal state was `navigating`, ETA was 13.500 seconds, and 1.350 m remained.
   The provider returned `finish`/`partial` after 7.360 seconds; the decision was
   recorded 7.209 seconds after the first nonzero Nav2 request, and controller
   completion followed 0.084 seconds later. This left no opportunity for a
   normal traversal outcome and is the dominant cause of this run's lack of
   useful forward progress.
2. **Goal geometry made rotation the necessary first behavior.** The sole target
   at map `(-0.7805, 0.5466)` was 145.0 degrees behind localization `(0, 0, 0)`.
   Nav2 consequently requested pure rotation for 77.79% of its active time.
3. **Downstream motion was highly fragmented.** The motor saw 19 starts and only
   1.288 seconds of forward output, split into nine windows; just 0.907 seconds
   reached 0.10 m/s. Most windows also carried substantial angular command.
4. **The true loaded-surface forward breakaway remains unknown.** The trace has
   no sustained straight 0.10 m/s window, so physical load may still be a later
   limiter. It is not necessary to resolve that uncertainty before fixing the
   earlier deterministic termination cause.

## Phase 0B decision and reserved protocol

Recommendation: **do not run Phase 0B now**. Human review should select the
active-leg finish-eligibility fix for Phase 1. After that fix, use its attended
fixed-distance acceptance run to determine whether a load-limited problem
remains. Candidate-selection forward bias can be considered only as a separate
follow-up if safe forward candidates exist and the termination fix alone does
not provide useful traversal.

If the corrected mission still cannot sustain forward movement, Phase 0B must
be a separate exact-SHA, attended, default-off diagnostic with this fixed
protocol:

1. Use the existing runner-to-supervisor ownership chain; never add a motor
   publisher. Keep STOP, ESTOP, collision, scan-freshness, lease, and room gates
   unchanged. Use a level bounded room and an operator at the controls.
2. Test deterministic CLEAR-state plateaus of 0.04, 0.06, 0.08, and 0.10 m/s in
   ascending order. Never exceed 0.10 m/s under this authorization.
3. For every plateau, command zero for at least 1.0 second and require measured
   linear speed below 0.01 m/s for at least 0.5 second before the next trial.
   Then command straight forward for at most 2.0 seconds or 0.15 m of odometry,
   whichever comes first, followed immediately by zero and the same settle gate.
4. Run three valid trials per plateau. A collision state other than `CLEAR`,
   stale sensing, lease loss, operator intervention, or angular request above
   0.01 rad/s invalidates the trial without counting it as a stall.
5. Call a plateau repeatably translating only when all three valid trials show
   at least 0.02 m net forward odometry within the command window plus the fixed
   0.25-second response allowance, with no reverse displacement and no safety
   gate activation. Preserve the synchronized raw events and SHA-bind the
   derived result.

If no plateau through 0.10 m/s passes, the result is exactly “forward breakaway
is greater than 0.10 m/s under this surface/load.” Any higher-speed experiment
then requires separate approval and M7.3 collision stopping-distance/no-contact
re-validation.
