# Design note — the rangefinder replaces the camera in navigation

**Status: design only. No code.** Written 2026-08-13 after the SEN0628's first
characterisation. Scott: *"Let's go ahead and start planning to remove the camera from
the navigation and add the range finder sensor instead."*

Evidence base: `03_validation/sensor_2026-08-13_tof_characterisation/` in the vault —
12,869 recorded frames across two sessions, with its own README. Every number below is
cited to that data, to a line of code, or to deployed config. **Where a number is not
yet measured, it says so and names the measurement that would settle it.**

**This batch touches the SAFETY PATH deliberately.** Every previous batch has been
verifiable by empty diffstat over `collision_stop*.py`; this one cannot be. That
protection is replaced by: a staged transition where the ToF carries no authority
until it has been watched flying alongside the camera, per-clause revert-proofs on
every safety-path change, and Scott's explicit gate on the final swap.

---

## 0. What the camera actually does today — measured, not remembered

`grep` for the topic, then read every consumer. Five, and one of them turns out to be
inert:

| # | Consumer | Where | Authority | Fails how? |
|---|---|---|---|---|
| A | **Forward brake** | `collision_stop_node.py:493` `_apply_camera_brake` | **SAFETY.** Scales forward speed only; never reverse, never turn | OPEN — stale/absent cloud → lidar-only |
| B | **Pivot veto (D19)** | `collision_stop_node.py:464` `_camera_blocks_pivot` | **SAFETY.** Zeroes angular when a point is inside the swept corner circle | OPEN — same |
| C | **Steering law** | `decisive_controller_node.py` `_on_camera_cloud` → `corridor_blocker` | Advisory. Changes the TARGET HEADING before `compute_drive_command`; emits no twist | Degrades to lidar-only blockers |
| D | **Local costmap layer** | `lean_nav2.yaml:116` `camera_low` | **NONE IN PRACTICE** — see below | n/a |
| E | **Telemetry** | `/collision_stop/state`: `cam_nearest`, `cam_scale`, `cam_output_linear`, `pivot_veto`, `cam_cloud_age`; recorder columns | Observability | n/a |

**D is inert and the design must not pretend otherwise.** `camera_low` marks the LOCAL
costmap, and decisive mode removes the local costmap entirely (`explore.launch.py`
lines 174/185/214/225 — `controller_server` and its costmap are conditioned OUT when
`use_decisive_controller:=true`). Every mission we fly is decisive mode. So the camera
has had **no planning influence at all** in the flights that matter, and the design
gains nothing by replacing D. It is listed so that nobody later "restores" a capability
that was not there.

The camera's deployed brake constants, for comparison against what the ToF can offer:

```
camera_stop_distance_m   0.50     camera_min_range_m  0.40      camera_swept_path  true
camera_slow_distance_m   0.70     camera_max_range_m  1.20      camera_half_width_m 0.16
camera_min_forward_scale 0.60     camera_max_age_s    0.6
```

**Why 0.50 m and not the lidar's 0.175:** the up-tilted camera cannot see floor closer
than ~0.45 m, so it must stop while the obstacle is still visible or the obstacle
slips into the blind zone before the stop lands. That constraint is a property of the
camera. **The ToF has a different one, and this design's central question is whether it
is a better one.**

---

## 1. What the ToF actually does — measured 2026-08-13

From the characterisation, in the sensor's own terms:

* **Accuracy**: agrees with tape within the tape's uncertainty (~1-2 cm) at 0.30 m,
  0.885 m and 0.994 m. No offset or scale error detectable.
* **Rate**: 7.6 Hz session-wide (the camera cloud runs ~5 Hz).
* **Field**: 60° × 60°, 8×8 zones, 7.5° per zone — the FOV confirmed independently by
  a 25 cm box filling 6 of 8 columns at 0.30 m, not taken from the datasheet.
* **Axes, TESTED not assumed**: **column 0 = rover-LEFT, row 0 = UP**, no mirror on
  either axis. *(Write this next to the lidar's fact and never merge them: the lidar's
  `base_link→laser` yaw is ~179°, so raw `/scan` bearing 0 points BEHIND the robot, and
  rung 3 once steered into a mirror image of open space because of it — N1. The two
  devices are different; the ToF needs no rotation.)*
* **Floor visibility**: floor returns at 0.26 m and 0.39 m (rows 7, 6) and **returns
  nothing where the floor would be at 0.79 m** (row 5). Grazing-angle physics, not this
  carpet.
* **Detection tracks FILL FRACTION, monotonically**, across four independent
  target/range pairs: wall (fills a zone) 100%; 5 cm box at 0.74 m (~half a zone)
  14-23%; 5 cm rail at 0.46 m (~half) 15%; 5 cm rail at 0.98 m (a sliver) ~1%.
* **A zone reports the NEAREST surface in its cone**, and does so reliably: with a 5 cm
  box in front of a wall, the affected zone read 892 mm instead of 992 mm in **100% of
  frames**.

That last pair is the whole design. **A marginal obstacle is intermittent if you ask
"did anything return" (~20%) and continuous if you ask "is this zone nearer than it
should be" (100%).**

### 1.1 Two things that must be settled before thresholds are frozen

1. **z versus radial.** Wall ranges are FLAT across 30° of elevation (983-993 mm) where
   radial ranging demands +16% at the top row. That is the signature of perpendicular
   (z) distance. **If it is z, the morning's floor-geometry fit assumed the wrong
   quantity** and mount height/pitch must be re-derived. Settled by a 5-minute test:
   one wall, two distances, check whether the elevation profile stays flat. **Scott-gated.**
2. **Firmware v1.2 vs v1.3.** The version is not readable over I2C (DFRobot's drivers
   define commands 1-8, `CMD_END = 8`, no version query; undocumented IDs were NOT
   probed because command 7 is a configuration WRITE). The wall test killed the "can it
   range at 1 m" doubt, but cannot distinguish "23% is the true rate for a marginal
   target" from "40% minus a firmware that discards readings". **Every number in §4
   marked (F) is frozen only after the v1.3 update and a repeat of the two rail steps.**

---

## 2. The driver node

`sphero_rvr_driver/tof_node.py`, one job: turn I2C frames into two topics and never
decide anything.

```
/tof/points        sensor_msgs/PointCloud2   the raw 8x8, one point per FINITE zone,
                                             in frame tof_link, stamped at read time
/tof/obstacles     sensor_msgs/PointCloud2   the DERIVED obstacle signal (§3), base_link
/tof/state         std_msgs/String           health: rate, i2c errors, stale, zone
                                             detection counts (events AND zones, D35)
```

**Two topics, not one, and the split is the point.** `/tof/points` is what the sensor
said; `/tof/obstacles` is what we concluded. A consumer that wants to disagree with our
conclusion can, and a recording keeps both — the camera's equivalent split is exactly
what let yesterday's analysis re-examine the raw data after my first statistic was
wrong.

**Sentinel handling**: 4000 mm is treated as NO RETURN and such zones are omitted from
`/tof/points` entirely — never emitted as a 4 m point. *This is an inference from
behaviour, not documentation (the value appears in 45 of 64 zones and sits above the
rated 3.5 m range) and it should be confirmed with DFRobot.* If it is ever found to be
a real measurement, one line changes and the tests catch it.

**TF**: a static `base_link → tof_link` publisher carrying the measured mount pose,
alongside the existing `base_to_laser` and `base_to_camera`. Geometry is TF's job, not
arithmetic scattered through consumers — the same rule that fixed N1.

---

## 3. The detection principle: nearer than it should be

For each zone, compute the range the background WOULD produce if nothing were there,
and flag the zone when it reads meaningfully nearer.

### 3.1 Below the horizon — the floor model

Each row below horizontal intersects the floor at a known distance from the fitted
geometry. Two rules, and the second is the one the recorded data proves:

```
(i)  zone returns FINITE and value < expected_floor - margin      -> OBSTACLE
(ii) zone returns FINITE where the floor CANNOT be seen at all    -> OBSTACLE
     (rows whose floor intersection lies beyond the visibility horizon)
```

Rule (ii) is strong precisely because of the floor-visibility limit that first looked
like bad news. Row 5's floor lies at 0.79 m, past the horizon, so row 5 normally
returns **nothing**. In the recorded data, row 5 returned finite values exactly when a
rail or a box was placed in it — 4-6% baseline, rising to 14-23% with an obstacle
present. **In that band, any return at all is an obstacle**, because nothing else in
the room can produce one.

`margin` is derived, not chosen: the measured range noise (a few mm) plus the pose
uncertainty of the floor model itself. It is NOT a tuning knob for this room.

### 3.2 Above the horizon — the honest answer

There is no floor to model. Three candidates, and I am recommending the first for
SAFETY and the third for steering only:

* **(a) Absolute proximity — RECOMMENDED for the brake.** A zone reading nearer than a
  fixed distance is an obstacle, full stop. It cannot be fooled by a wrong background
  model, and its failure mode is over-caution (braking for a wall you were going to
  stop at anyway) rather than under-caution.
* **(b) Persistence / learned background — REJECTED for safety.** Learn what each zone
  usually reads and flag departures. It would give the best sensitivity, and it is the
  option I would most like to be talked out of rejecting — but **its failure mode is
  silent and dangerous**: a background learned while an obstacle was present makes that
  obstacle invisible thereafter, and a rover that has been sitting still for 30 s has
  learned exactly the scene it is about to drive into.
* **(c) Frame-to-frame novelty — steering only.** A zone that got suddenly nearer is
  interesting. Cheap, no memory of "usual", and its failure mode (missing a static
  obstacle) is tolerable for a steering hint and intolerable for a brake.

**Stated as a rule: the BRAKE may only use models whose errors make it stop sooner.
Anything cleverer belongs to steering, where being wrong costs a wasted curve.**

### 3.3 What breaks this

* **A wrong floor model is the central risk** — it makes real floor look like an
  obstacle (rover freezes, mission dies of phantom brakes) or hides one (contact). The
  model comes from a geometry fit that is itself unfinished (§1.1). **The fit must be
  re-derived and validated against recorded flight data before it holds authority**,
  which is what stage (ii) of the transition is for.
* **Pitch drift.** A mount that shifts by 2° moves every row's floor intersection.
  Detectable: the floor rows' readings ARE the calibration, so the driver can publish
  the residual against the model and the state topic can say "my floor model disagrees
  with the floor by X" — a self-check the camera never had.

---

## 4. The envelope, and which numbers are frozen

Derived from the sensor and the measurements, **not from the camera's constants**.

| Quantity | Value | Where it comes from |
|---|---|---|
| Floor visibility horizon | ~0.4-0.5 m | measured: floor at 0.39 m yes, 0.79 m no |
| Raw detection, 5 cm object | ~0.3-0.5 m at ~15-23% of frames **(F)** | measured, rail + box |
| Detection via §3 comparison | 0.74 m at **100%** of frames | measured, box against wall |
| Zone height at 0.5 m | 65 mm | 7.5° per zone |
| Zone height at 1.0 m | 131 mm | 7.5° per zone |
| Proposed ToF stop distance | **derive after the geometry re-fit** | must exceed braking distance at cruise |
| Proposed slow band | **derive after the re-fit** | as above |

**(F) = gated on the v1.3 firmware retest.** If the true rate is 40% rather than 15%,
detector windows shorten and the usable envelope grows; if it is genuinely 15%, the
comparison rule of §3 is doing nearly all the work and the design must say so.

**The honest comparison with the camera**: the camera stops at 0.50 m because it cannot
see closer. The ToF sees to 0.26 m and detects best between 0.3 and 0.5 m — so on RAW
detection it is a *shorter*-range sensor, and a naive swap would trade a 0.50 m stop for
a 0.4 m one. The design's bet is that §3's comparison rule more than repays that, since
it turned a 20% signal into a 100% one at 0.74 m in the recorded data. **That bet is
what stage (ii) exists to test, and it is the thing that should kill this batch if it
fails.**

---

## 5. Transition — four stages, each with its own revert

**(i) SHIP THE DRIVER, NO AUTHORITY.** `tof_node` publishes; nothing consumes. Recorder
gains ToF columns beside `cam_*`. Camera keeps every job. *Revert: don't launch the node.*

**(ii) SIDE-BY-SIDE, IN FLIGHT.** The gauntlet missions are the data source — they fly
anyway. Both sensors recorded, camera still authoritative. The question to answer with
numbers: **in the band where both can see, does the ToF flag what the camera flags?**
Plus the two the camera cannot answer: what does the ToF catch that the camera missed,
and how often does the floor model disagree with the floor? *Revert: nothing to revert;
this stage changes no behaviour.*

**(iii) AUTHORITY SWAP — the safety-path batch.** ToF into the brake, the pivot veto and
the steering law; camera consumers unwired. **Does not fly until: the gauntlet is
complete on the current binary (sequencing recommendation — Scott can override), AND
stage (ii) shows the ToF catching what the camera catches in the overlapping band.**
*Revert: `camera_brake_enable` and its ToF counterpart are both parameters; the revert
is a config change and a relaunch, not a rebuild.*

**(iv) RETIREMENT.** D27 (optical low-obstacle) closes. D22's navigation relevance ends.
The `camera_low` costmap layer is deleted rather than repointed (it is inert, §0).
**The camera node, its launch and Track 2's `observe` all stay** — the camera loses
motion authority, not its existence.

---

## 6. Failure modes

* **STALE OR DEAD SENSOR — the D22 lesson, and it is not optional.** A stale ToF must
  degrade to **lidar-only**, never to a phantom brake and never to "assume clear". The
  camera's brake and veto already fail open on `camera_max_age_s`; the ToF's must do the
  same, with the age published so the fail-open window is visible in a recording.
  D22 was exactly this: a starved subscription silently disabling the veto while clouds
  were still arriving.
* **I2C BUS HEALTH — a seam we have not had before.** The RVR is UART, the lidar is USB;
  I2C is new, and a hung bus reads as "no data", which is indistinguishable from "clear
  floor" unless the driver says so. The driver must publish read errors and rate on
  `/tof/state`, and the brake must treat "no frames" as stale, not as clear. **Assert,
  don't infer**: the driver knows it failed; the consumer must not guess from silence.
* **SUNLIGHT.** The camera's low-obstacle path died of sun (D27, hard-won). This is a
  940 nm active sensor and ambient IR competes with its own illuminator — the failure
  is plausible, the mechanism is different, and **it is untested**. One measurement in
  hard sun before the authority swap, or the swap ships an unknown.
* **LIDAR CROSS-TALK.** RPLIDAR is ~905 nm; this device ~940 nm; both are in the same
  plane-ish volume. Interference is unlikely and unmeasured. Cheap to check: both
  running, look for a rate or noise change vs lidar stopped.
* **A ZONE REPORTS THE NEAREST SURFACE.** Safe for braking, but it means a zone cannot
  see past a near object — a 5 cm rail at 0.4 m hides the floor behind it. Consumers
  must not treat "one zone, one distance" as "this whole cone is at that distance".
* **MULTI-RETURN / MIXED PIXELS.** The recorded data shows a strong near return can
  BLANK an adjacent grazing-floor zone (row 7 went silent with a box at 0.30 m, normal
  with it at 0.885 m). So a nearby obstacle can suppress a neighbouring zone's floor
  return. Clear-ray semantics must never assume a silent zone is clear.

---

## 7. Revert-proofs (per clause, all mutation-verified)

1. `stale_tof_degrades_to_lidar_only` — an aged frame removes the ToF limit entirely;
   the rover keeps driving on lidar alone. Fails against a version that holds the last
   limit.
2. `i2c_failure_is_stale_not_clear` — read errors produce staleness, never a
   clear-floor conclusion.
3. `floor_is_not_an_obstacle` — replayed clear-floor frames from the characterisation
   produce ZERO obstacle points. Fails against a floor model off by more than its margin.
4. `the_recorded_rail_is_detected` — the 0.46 m rail frames produce an obstacle in the
   right bearing and range. Fails against presence-thresholding at the recorded 15% rate.
5. `the_recorded_box_is_detected_continuously` — the 0.74 m box against a wall produces
   an obstacle in ~100% of frames via §3.1, not ~20%. This is the design's central claim,
   and it is falsifiable against recorded data before any code flies.
6. `the_tof_brake_only_slows_forward` — never reverses, never turns, exactly like the
   camera brake it replaces. Same clause, new sensor.
7. `axis_orientation_is_not_mirrored` — a synthetic obstacle at rover-left produces a
   left-bearing obstacle point. Fails against a mirrored mapping (the N1 lesson,
   pre-empted rather than repeated).
8. `sentinel_is_never_a_measurement` — 4000 mm zones never appear as 4 m points.

---

## 8. Explicitly NOT in this batch

* Removing the camera node, its launch, or Track 2's `observe`.
* Any change to the LIDAR core of the supervisor — the lidar's stop/slow/pivot gates are
  untouched; this batch only replaces the ADDITIVE camera layer.
* Re-tuning lidar constants to compensate for a new sensor.
* A learned/persistent background model (§3.2b), which is rejected for safety and would
  need its own design if anyone wants it for steering.
* The v1.3 firmware update itself (Scott's Windows machine) — this design consumes its
  result, it does not perform it.
