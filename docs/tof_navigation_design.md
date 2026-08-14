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

### 1.0 DECIDED MOUNT GEOMETRY, 2026-08-14

**Height as-is (~0.10-0.11 m), tilted DOWN 10°, board NOT flipped.** Scott's decision,
against the analysis below.

| option | verdict |
|---|---|
| flip the board ~3-4 cm lower | **declined.** Its hoped-for benefit does not exist: a 5 cm object at 0.5 m subtends 5.60° from h=0.10 and 5.68° from h=0.07 — 1% of fill — because angular subtense is set by distance, not sensor height, when the sensor is above the object. Meanwhile `d_max` falls 0.51 → 0.33 m and rows in the useful 0.20-0.50 m band drop from 2 to 1 |
| raise the mount to ~0.15 m | **declined for RIGIDITY**, and this is the one that hurt. It was the biggest lever available (`d_max` 0.51 → 0.77 m, 3 rows in the useful band) but Scott: *"I don't have the ability to raise it much without totally redesigning the mount and possibly losing rigidity."* A flexing mount corrupts the floor model continuously, which is worse than any static geometry — rule (i) compares against a model of where the floor should be, and a mount that moves makes that model wrong in a way no calibration can catch |
| **tilt down 10°** | **ADOPTED.** 3 floor-seeing rows instead of 2, near edge 0.24 → 0.16 m, 2 rows still inside 0.20-0.50 m, and 0.47 m of overhead reach at 1 m |

**What tilt can and cannot do, since the reasoning is easy to get backwards:**
`d_max = h / tan(θ_min)` contains **no tilt term**. Tilting cannot push rule (i)'s outer
edge past the height-determined limit; what it does is decide WHICH rows land inside
that limit, and it converts beyond-horizon rows into floor-seeing ones. That matters
because **rule (i) is the strong rule** (continuous, no confirmation window) and rule
(ii) is the weak one (adjacency plus N-of-M). Trading weak-rule rows for strong-rule
rows is the actual gain.

**The cost, stated:** overhead coverage. The cone's highest reach at 1 m falls from
0.68 m to 0.47 m. Past ~15° the rover would start driving under chair seats seeing only
legs, which is why 10° and not 20°.

**AMBIGUITY FOR SCOTT TO RESOLVE WHEN HE AIMS IT:** the current mount sits at a FITTED
+4° nose-UP, which was never deliberate. "Down 10°" from level gives -10°; ten degrees
of rotation from where it is now gives -6°. **He should aim for 10° below LEVEL**, and
it does not need to be precise — the actual angle is FITTED from the floor rows in the
bench session below, exactly as the +4° was. His aim is the target; the fit is the
number that ships.

### 1.05 The bench session that freezes the geometry

One sitting, no chassis motion, ~5 minutes of captures. **Stage (ii) does not begin
collecting until these numbers are in**, because side-by-side data taken at a geometry
we then change describes a robot that does not exist.

| # | capture | settles |
|---|---|---|
| 1 | clear floor, 30 s | re-fits mount height AND pitch; re-measures the grazing threshold (11° ± 3.75°, which every number in this note rests on) |
| 2 | wall at distance A, 30 s | z-versus-radial: does the elevation profile stay flat? |
| 3 | wall at distance B (≥ 0.3 m from A), 30 s | confirms 2 — one distance cannot separate a flat profile from a coincidence |
| 4 | 5 cm rail at 0.46 m, 30 s | rule (ii) adjacency rate at the FINAL geometry; the M/N table is recomputed from it |

Deliverables: refitted `mount_height_m`, `mount_pitch_deg`, `floor_horizon_m`,
`reports_z`, and a recomputed adjacency/N-of-M table. Those four constants then FREEZE
until something physical changes.

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

`sphero_rvr_driver/tof_node.py`, one job: turn I2C frames into two topics. It makes **no
motion decisions**; the derivation in §3 is deterministic, pure and testable against
recorded frames, which is a different thing from "decides nothing" — it decides what
counts as an obstacle, and that decision is auditable offline.

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

`margin` is derived, not chosen: the measured range noise (a few mm) plus the pose
uncertainty of the floor model itself, including the DYNAMIC pitch envelope of §3.3.
It is NOT a tuning knob for this room.

#### Rule (ii) needed fixing, and the review caught it against my own revert-proof

The first draft of this note said *"in that band, any return at all is an obstacle,
because nothing else in the room can produce one."* **That is false, and the same
section recorded the evidence against it**: a 4-6% baseline of finite returns with
nothing there. At 7.6 Hz that is a phantom obstacle every ~3 s on empty floor, and
revert-proof 3 (replayed clear-floor frames produce ZERO obstacle points) fails against
the design as written. The proof stands; the rule was wrong. What follows is measured
from the recorded segments rather than reasoned about.

**What the baseline returns actually are.** Two populations, and they need separating:

* **Physically impossible values** — `0`, and values above ~1.8 m in a row whose entire
  geometry is under 1 m. Examples from the clear-floor and wall-only segments: 0, 1824,
  2163, 2251, 2612, 2717, 3185, 3691, 3730. About 4 per 30 s segment. **These are the
  invalid-value signature the v1.3 firmware note warns about**, and they are removed by
  a plausibility filter (60-1500 mm for a beyond-horizon row) at zero cost.
* **Plausible-looking isolated returns** — 400-700 mm, scattered, no obvious cause;
  cone-edge clutter or glints off the floor are candidates and I cannot tell from this
  data which. **An unexplained few percent feeding a brake is exactly what the temporal
  and spatial structure below is for**, and it is a question for the v1.3 retest.

Filtering removes only the first population: baseline 3.16% → **2.86%** raw-to-filtered
on clear floor, 4.19% → 3.89% on wall-only. Not a fix by itself.

**Run lengths kill the obvious remedy.** Consecutive-frame debouncing does not work
here, because obstacle returns are ALSO mostly isolated: median run length 1 frame both
with and without an obstacle (max 5 with the rail, max 1-3 without). A rule requiring
consecutive returns would reject the obstacle along with the noise.

**What DOES separate them is SPATIAL COHERENCE.** An obstacle spans adjacent zones; the
baseline arrives as scattered singles. Measured, per frame, over row 5's eight zones —
"are there ≥2 ADJACENT zones returning at once":

| segment | ≥2 adjacent zones, per frame |
|---|---|
| clear floor, nothing ahead | **1.3%** |
| wall at 0.994 m, no obstacle | **1.8%** |
| 5 cm rail at 0.46 m | **45.2%** |
| 5 cm box at 0.74 m (against wall) | 14.0% |

That is a 25× separation on the rail case, from a single frame, with no memory at all.

**So rule (ii) becomes:** a beyond-horizon obstacle requires **≥2 adjacent zones
returning plausible values in the same frame, in N of the last M frames**. Derived from
the rates above (baseline 1.8%, rail 45.2%):

| M | N | window | P(false fire per window) | false fires/min | P(detect, rail) |
|---|---|---|---|---|---|
| 6 | 2 | 0.79 s | 0.0046 | 0.35 | 0.84 |
| **8** | **2** | **1.05 s** | **0.0084** | **0.48** | **0.94** |
| 8 | 3 | 1.05 s | 0.0003 | 0.02 | 0.78 |
| 12 | 3 | 1.58 s | 0.0011 | 0.04 | 0.96 |

**Recommendation: M=8, N=3** — one false fire per ~50 minutes, 78% detection of the
recorded rail per window, 1.05 s of window which is 0.21 m at cruise and 0.105 m at the
ladder's escape speed. M=12/N=3 buys 96% detection for 1.58 s (0.32 m at cruise), which
is too much travel to spend before braking. **(F)** — every number here moves with the
firmware retest, and the table is the thing to recompute rather than re-argue.

**And the honest limit:** the box at 0.74 m scores only 14% per frame on adjacency,
because at that range a 5 cm object spans barely one zone. Rule (ii) is therefore a
CLOSE-RANGE rule (~0.3-0.5 m, where an obstacle subtends 2+ zones) and rule (i) is what
covers the box case — which it does at 100%, because there a wall provides the
background. **The two rules cover different situations and neither generalises to the
other's**; a design that claimed one rule for everything would be claiming something
this data refutes.

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
* **Static pitch drift.** A mount that shifts by 2° moves every row's floor
  intersection. Detectable: the floor rows' readings ARE the calibration, so the driver
  publishes the residual against the model and the state topic can say "my floor model
  disagrees with the floor by X" — a self-check the camera never had.
* **DYNAMIC PITCH, and it is the one that bites.** The chassis pitches on carpet under
  acceleration and braking. A transient 2-3° nose-down moves every row's floor
  intersection NEARER, which fires rule (i) — *exactly when the rover starts braking*.
  That is a positive feedback loop into phantom stops: brake → pitch → phantom
  obstacle → brake harder. The static-drift check above does not catch it, because the
  residual is transient and correlated with the very command that caused it.

  Consequences, all required rather than optional:
  1. **The margin in rule (i) is derived from the MEASURED in-flight pitch envelope**,
     not from bench geometry. Stage (ii) supplies it: floor-row residual against
     commanded acceleration, over real missions.
  2. **The residual self-check becomes a GATE on stage (iii), not telemetry.** The
     authority swap does not happen until recorded flight shows the floor-model residual
     staying inside the margin through accel and brake transients.
  3. Until that envelope is measured, **any ToF number quoted for the brake is provisional**
     — a static bench fit cannot bound a dynamic error, and pretending otherwise is how a
     phantom-brake cascade gets discovered on carpet instead of in a recording.

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

### 4.1 The envelope is a function of OBJECT HEIGHT, not a single number

A zone spans `2·d·tan(3.75°)` vertically — 39 mm at 0.30 m, 66 mm at 0.50 m, 131 mm at
1.0 m, 262 mm at 2.0 m. An object taller than that FILLS the zone, and fill fraction is
what detection tracks (measured across four target/range pairs):

| class | height | fills a zone out to | measured |
|---|---|---|---|
| **rail-class** | ~5 cm | **~0.38 m** | 83% fill at 0.46 m → 15-45% of frames; 39% fill at 0.98 m → ~1% |
| **box-class** | ~15-25 cm | **~1.1-1.9 m** | 216% fill at 0.885 m → **100% of frames** |
| **wall-class** | fills always | full range | **100%**, 992 mm vs 994 tape |

**So "0.3-0.5 m" is the RAIL-class envelope, and quoting it as the sensor's is the
mistake this table exists to prevent.** Anything taller than ~13 cm fills a zone at a
metre and is seen as reliably as the wall was. The weak class is the smallest one —
and that is also the class the freeze/escape machinery already handles by touch, which
is what makes the trade survivable rather than merely accepted.

**THE GAP THIS EXPOSES, and it is not yet built.** Rules (i) and (ii) are FLOOR rules:
one compares against modelled floor, the other fires in the band where floor cannot
return. A box-class object at 1 m sits at or just ABOVE the horizon (a 13 cm object at
1.0 m subtends +1.7°, which is row 4) — where neither rule applies. §3.2 recommended
absolute proximity there and stage (i) did NOT implement it, deliberately, because the
obvious version does not work: **at 1 m a 13 cm object and a wall read the same
distance in the same zone.** Absolute proximity cannot tell them apart, and firing on
both means braking at every wall.

The promising answer is not a learned background but a LIVE one: **the lidar already
knows where the walls are.** A ToF return at 0.9 m in a bearing where the lidar reports
open space to 3 m is, by construction, something below the lidar plane — which is
exactly the definition of the obstacle class this sensor exists for. That is sensor
fusion rather than memory, its background comes from a sensor we trust, and its failure
mode is loss of sensitivity rather than a phantom.

**It is written here and NOT built**, because it is a new rule in a reviewed design and
belongs in the next review rather than smuggled into stage (i). Stage (ii)'s recordings
will show how often it would have mattered.

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

**AND THE STEERING LAW LOSES RANGE — say it plainly.** Consumer C reads camera points
across 0.40-1.20 m and uses them to lean the heading BEFORE the brake band, which is the
whole point of the gentle-turn design (engage at 0.90 m). The ToF's raw detection of a
5 cm object on OPEN floor at 0.5-1.2 m — no wall behind it, so no background to compare
against, so rule (i) cannot help — is **single-digit to ~20% of frames**, measured. In
that band the replacement input is materially weaker than what it replaces, and the
steering law was designed around information arriving at 0.90 m.

**RESOLVED BY SCOTT, 2026-08-14: full removal.** Verbatim — *"Let's just go without the
camera for navigation right now."* The ToF takes the brake, the pivot veto AND the
steering law's obstacle input; after stage (iii) the camera holds **zero** navigation
roles.

**The accepted cost, stated plainly so it is a decision and not a discovery:** reduced
early steer-around for small low obstacles at 0.5-1.2 m on open floor. The rover will
brake-and-escape where it would previously have curved around, and it will do so until
something else fills that band. That is a real loss of grace, knowingly taken.

**The hybrid — camera for steering, ToF for brake and veto — was considered and
declined**, not overlooked. It is kept here because Scott's *"right now"* leaves the
door open: if stage (ii)'s degradation metric shows the loss biting harder than this
paragraph implies, the number goes back to him as a fresh decision rather than being
relitigated quietly by whoever is next in this file.

**Stage (ii)'s steering-degradation measurement therefore stays**, and changes purpose:
it is no longer deciding whether to swap, it is the evidence for whether the accepted
cost is the cost we thought it was.

**CORRECTION AND RE-CONFIRMATION, 2026-08-14.** After the design was approved, stage
(i)'s revert-proof falsified the claim that the comparison rule would buy back the
0.5-1.2 m band (§7, proof 5 — 0 of 40 recorded frames, because that 100% figure came
from comparing against a WALL, which is the learned background §3.2(b) rejects). The
corrected cost went back to Scott, who re-confirmed: *"Go ahead with camera removal.
Do your best with the ToF. Note you should be able to use it for most objects."*

**AND THE LIVE BACKGROUND RECOVERS MOST OF WHAT THE FALSIFIED CLAIM PROMISED**
(measured 2026-08-13, after §9 was built). The 100% figure that proof 5 falsified came
from comparing against a remembered wall — §3.2(b), rejected for safety because a
background learned while an obstacle was present hides that obstacle forever. Rule B
makes the same comparison against a background that is MEASURED LIVE by a second
sensor: 38 of 40 recorded frames on the 5 cm object at 0.50 m. Same detection, no
memory, and its failure mode is a missed obstacle rather than an invisible one. The
capability the design over-claimed and then honestly retracted is largely back, by a
mechanism the retraction is what forced us to find.

**And his note is right, which the first correction over-stated in the other
direction.** The 0.3-0.5 m limit belongs to the 5 cm RAIL CLASS, not to the sensor.
See §4.1 — object height sets the envelope, and most things this rover meets are not
5 cm tall.

---

## 5. Transition — four stages, each with its own revert

**(i) SHIP THE DRIVER, NO AUTHORITY.** `tof_node` publishes; nothing consumes. Recorder
gains ToF columns beside `cam_*`. Camera keeps every job. *Revert: don't launch the node.*

**(ii) SIDE-BY-SIDE, IN FLIGHT.** The gauntlet missions are the data source — they fly
anyway. Both sensors recorded, camera still authoritative, no behaviour change.

**PASS/KILL CRITERIA, PRE-REGISTERED — these are written before the first side-by-side
flight and are not renegotiable afterwards.** Post-hoc judgement on side-by-side data is
how a bet quietly becomes a narrative, which is the same failure as reading a median
that hid an intermittent target.

| Metric | PASS | KILL |
|---|---|---|
| **Agreement**: camera brake events (`cam_scale < 1.0`) with a concurrent ToF obstacle flag, restricted to the overlap band where both can see (0.40-0.50 m) | ≥ 80% | < 50% |
| **Phantom rate**: replay the recording with ToF-as-authority and count brake events with no camera event and no lidar obstacle | ≤ 1 per mission | > 3 per mission |
| **Floor-model residual**: |measured floor row − model| through accel/brake transients (R2) | ≤ the derived margin in 99% of frames | > margin in > 5% of frames |
| **Steering-input degradation** (R4): ToF blocker availability vs camera blocker in the 0.5-1.2 m band | ToF supplies a blocker in ≥ 50% of frames where the camera did | < 25% |

An outcome between PASS and KILL is neither — it means another mission's data, or a
narrowed claim, not a judgement call in the direction we were already hoping for.
*Revert: nothing to revert; this stage changes no behaviour.*

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
  is plausible, the mechanism is different, and **it is untested**. **This is the THIRD
  Scott-gated hardware item** (with the v1.3 update and the z-vs-radial wall test): one
  measurement in hard sun, and it GATES stage (iii). Shipping the authority swap without
  it would put the rover's only sub-lidar sense on an untested failure mode that already
  killed its predecessor.
* **LIDAR CROSS-TALK.** RPLIDAR is ~905 nm; this device ~940 nm; both are in the same
  plane-ish volume. Interference is unlikely and unmeasured. **Scheduled into stage (i)
  bringup** rather than left floating: with the driver running, compare ToF rate and
  per-zone noise with the lidar spinning vs stopped. Five minutes, no chassis.
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

---

## 9. AMENDMENT, 2026-08-13 — the detection rules at the fitted geometry

**Status: HELD FOR REVIEW. No code in this section has been written.** §§1-8 above
describe the LEVEL mount and stay as the record of that; this section supersedes §3.1's
two rules for the tilted mount and adds the lidar-background rule §4.1 wrote down and
deliberately did not build.

### 9.0 The defect, measured

Running the real `ObstacleDetector` against 23,057 frames at the new tilt: **an obstacle
was reported in 99.3% of frames, rule (i) fired ZERO times, and the trigger was row 2
seeing the WALL.** The box the session existed to detect was never the reason. Both
rules broke, in opposite directions, from the same cause.

### 9.1 Diagnosis — a visibility constant was doing an authority job

`floor_horizon_m = 0.55` answers *"can the floor return from this far?"*. Both rules used
it to answer *"may this row conclude an obstacle?"*. Those are different questions, and
tilting the mount pulled them apart:

All figures below are computed THROUGH THE PRODUCTION MODEL at the shipped constants,
centre column, and "floor reads" is the quantity rule A actually compares against — the
reading a zone would report for flat floor, not a ground distance:

| row | world elevation | floor reads | height of a return at 0.50 m |
|---|---|---|---|
| 0 | +4.98° | none (above horizon) | +0.226 m |
| 1 | −0.92° | 8.37 m | +0.131 m |
| 2 | −6.82° | **1.16 m** | +0.079 m |
| 3 | −12.72° | 0.63 m | +0.026 m |
| 4 | −18.62° | 0.43 m | — (below floor) |
| 5 | −24.51° | 0.33 m | — |
| 6 | −30.41° | 0.27 m | — |
| 7 | −36.31° | 0.23 m | — |

Rule (i) requires `expected_floor_m`, which returns `None` past the horizon — so rows 2
and 3 lost their floor model entirely and rule (i) went silent on exactly the rows that
now point at the interesting band. Rule (ii) fires on *any* adjacent plausible returns
past the horizon, on the premise that **nothing else in the room produces one** — true
for a row whose floor is 0.30 m away, false for row 2, whose floor is 1.16 m away and
whose ray at 0.68 m is only 79 mm off the ground. A wall has material at 79 mm. The
premise was never a property of the rule; it was a property of the level geometry, and
nothing in the code said so.

### 9.2 Rule A (was rule (i)) — floor comparison, with a DERIVED applicability bound

The comparison is unchanged: a finite plausible return at `R < F − floor_margin` is
something standing between the sensor and the floor. What changes is **which rows may
use it**, and the bound is derived rather than tuned:

> **Rule A applies to a row only where its floor intersection F is inside the stop
> distance.**

The justification is §3.2's rule that the brake may only use models whose errors make it
stop sooner. Rule A cannot distinguish an object from the sub-lidar part of a wall — it
never could. Where `F ≤ stop_distance`, it does not need to: anything nearer than the
floor at that range is inside the brake band whatever it is, so both explanations
demand the same action and the ambiguity is harmless. Where `F > stop_distance`, the two
explanations demand opposite actions and rule A is not entitled to choose.

At a stop distance of 0.45 m this admits **rows 4-7**; at 0.35-0.40 m, rows 5-7. The row
set is therefore a FUNCTION of the stop distance, which §4 still lists as
*derive after the re-fit* and which §3.3 gates on the unmeasured in-flight pitch
envelope. **The row set is not frozen by this amendment and must not be hardcoded** — it
is computed from the geometry and the stop distance at config time, and published on
`~/state` so a recording says which rows held authority.

`floor_horizon_m` keeps its original job — it stays in `plausible_for_zone` and in the
"can this zone see floor at all" question — and loses its authority job entirely.

### 9.3 Rule B (replaces rule (ii)) — the lidar as a LIVE background

For rows outside rule A's bound, the ToF alone cannot separate an object from a wall.
The lidar can, and §4.1 already stated why: **a return the lidar CANNOT see at matching
range is sub-lidar by definition.** That is the class this sensor exists for.

```
For a ToF return, expressed in base_link as (x, y, z):
    r     = hypot(x, y)                 range in the ground plane
    beta  = atan2(y, x)                 bearing
    L_min = MINIMUM lidar range over the angular span of this zone's column
    RULE B fires when   r  <  L_min - disagreement_margin
```

**CORRECTION, made while building it: THE FLOOR IS A RETURN TOO.** As approved, the rule
above compares every ToF return against the lidar and nothing else — and the floor is
always nearer than the wall the lidar sees. Replayed against recorded frames the first
implementation fired in **38 of 40 clear-floor frames**. The rule needs a front half:

```
a return is a CANDIDATE only if it stands ABOVE the ground --
    reading < modelled floor - floor_margin      (or the row never meets the floor)
then rule A concludes directly inside its bound, and rule B asks the lidar outside it
```

That makes the two rules one pipeline rather than two overlapping tests: the same
"something is standing here" evidence, adjudicated by the floor model where that is
safe and by the lidar where it is not. It also re-introduces a floor-model dependency
into rule B, in the EXCLUDING direction only — a wrong floor model can make rule B miss
an obstacle, never invent one. That is the failure direction rule B already committed
to. And the margin cuts the right way: a return within `floor_margin` of modelled floor
is within a couple of centimetres of the ground at any of these geometries, so what it
excludes is objects thin enough that the freeze/escape machinery is what handles them.

Four further properties, each load-bearing:

* **The comparison happens in `base_link`, through TF, on both sides.** The lidar's
  `base_link->laser` yaw is ~179°, so its raw bearing 0 points BEHIND the robot, and the
  two sensors sit at different x offsets — at 0.68 m that parallax is not negligible.
  Comparing raw scan indices against ToF columns is the N1 defect with a new pair of
  sensors. This is the single most likely way to build this rule wrong.
* **Applicability: the ToF ray must be BELOW the lidar plane** (`z < laser_z`, measured
  0.1905 m). Rows 1-7 always satisfy this — the mount at 0.139 m sits 51 mm below the
  lidar plane and every downward row descends. Row 0 (+4.95°) crosses the lidar plane at
  **0.59 m**; beyond that the lidar can see what row 0 sees, disagreement means occlusion
  geometry rather than a sub-lidar object, and rule B must not fire. Below 0.59 m it may.
* **`L_min`, not `L_mean` or `L_max` — deliberately the insensitive choice.** A ToF
  column spans 7.5° against the lidar's ~1°, so a column straddling a doorway or a corner
  contains both near and far lidar returns. Taking the minimum makes disagreement HARDER
  to claim, so the rule's error is a missed obstacle rather than a phantom brake at every
  corner. That is the failure mode §4.1 committed to (*"loss of sensitivity rather than a
  phantom"*), and it is the opposite of rule A's — which is correct, because rule A
  operates where over-caution is free and rule B operates where it costs the mission.
* **Adjacency and N-of-M survive unchanged.** The isolated-return baseline (1.3-1.8% of
  frames) is a property of the sensor, not of the geometry, and nothing in the tilt
  data contradicts the 25× adjacency separation §3.1 measured. `min_adjacent_zones=2`,
  `M=8, N=3` carry over, still **(F)**-gated on the firmware retest.

**No lidar return at that bearing is NOT the same as no lidar.** A bearing where the
scan reports infinity, NaN, or beyond max range means *open to max range* — so any ToF
return there is sub-lidar and rule B fires at full strength. That is the rule's strongest
case, and conflating it with a missing scan would silently invert it. The two are
distinguished at the source and reported separately on `~/state`.

### 9.4 The gap between A and B is real, and today's own object sat on its edge

> **RETRACTION, 2026-08-13 (the frame batch). The "0.178 m" figure below is wrong and
> every conclusion drawn from its SIZE is withdrawn.** It is `0.678 − 0.500`: a lidar
> range measured from `base_link` minus a **ToF sensor reading**, subtracted across two
> frames that differ by the sensor's 0.10 m forward offset. The object's real
> disagreement is **0.089 m**. So "nearly twenty times the margin rule A had to spare"
> becomes *under one times the margin* — at the shipped `disagreement_margin_m` of
> 0.10 m, rule B does not fire on this object at all (0 of 40 recorded frames).
>
> **What survives, and it is the load-bearing part:** rule B does not consult the floor
> model, so a floor-fit error cannot move it, while rule A's verdict here rests on
> 1.5–9 mm of margin against a model that took a 15 mm correction. The ARGUMENT FOR
> LAYERING is unaffected — it was never about the size of the number. What is withdrawn
> is the claim that rule B holds a comfortable margin on this object at the current
> threshold. It holds 0.089 m, and the threshold has to come in under that.
>
> This is the same defect class as the code the section describes, committed in prose
> while describing it: `range` meaning two things. See `docs/frame_fix_handover.md` §9.6.

Row 3's floor reads 0.63 m, so it is outside rule A's bound at any plausible stop
distance — and it is the row that saw the 5 cm box at 0.50 m in 100% of frames. Testing
that object against rule A as if row 3 did qualify:

```
reading 0.500 m ,  floor reads 0.629 m ,  floor_margin 0.120
0.500 < 0.629 - 0.120 = 0.509 ?   yes -- by 9 mm
```

**CORRECTION.** The first draft of this section computed 0.614 m by hand, using the
pre-9.10 geometry, and reported that rule A MISSED this object by 6 mm. Recomputed
through the corrected model it catches it, by 9 mm. The conclusion survives the sign
flip and is in fact strengthened by it: **rule A's verdict on the best-detected object
of the session moves from "miss" to "catch" on a 15 mm change in the floor model.** A
detection that thin is not a capability. Rule B does not depend on the floor model at
all — the lidar measured the wall behind the box at 0.678 m, a disagreement of 0.178 m,
nearly twenty times the margin rule A had to spare.

**THE ARITHMETIC ABOVE IS A PREDICTION. HERE IS THE MEASUREMENT, AND IT SUPERSEDES IT.**
Replayed through the built rules over 40 consecutive recorded frames, **rule A alone
reaches this object in 0 of 40** — and the reason is NOT the 9 mm. It is the
applicability bound: row 3's floor reads 0.63 m, outside the 0.45 m stop distance, so
rule A never evaluates the row at all. The bound working as designed is the whole
finding. Anyone reading the 9 mm as "rule A nearly had it" has the causality backwards.

Lift the bound artificially — `stop_distance_m = 0.70`, admitting row 3 — and the
prediction is roughly borne out, but raggedly, which is the fragility argument in a
sharper form than the single-column sum could give:

| row 3 column | median reading | rule A threshold | fires |
|---|---|---|---|
| 1 | 515 mm | 517 mm | **27/40** |
| 2 | 502 mm | 512 mm | 39/40 |
| 3 | 500 mm | 509 mm | 38/40 |
| 0, 4–7 | 539–704 mm (the wall) | 509–526 mm | 0/40 |

The object spans three columns, and rule A's confidence across them runs from 98% to
**68%** on 1.5 to 9 mm of margin. The 9 mm in the hand calculation was the BEST column,
quoted as if it were the object's. Rule B fires 38/40 on the same frames with 178 mm of
margin and no floor model in the path at all.

So rule B is load-bearing rather than an enhancement: without it there is a band,
starting right at the stop distance, where the sensor sees an obstacle perfectly and the
only rule available to report it is balanced on its own calibration error.

**STATED AS THE ARGUMENT FOR THE LAYERING, because that is what it is.** Rule A is a
*differencing* rule: its verdict is the gap between a reading and a MODEL, so near its
applicability edge that verdict inherits the model's whole error budget. The numbers
above are that inheritance made visible — 9 mm of decisive margin against a 15 mm
correction to the model itself. Rule B is a *comparison between two sensors*: it holds
178 mm on the same object and it does not consult the floor model at all, so a floor-fit
error cannot move it.

**Rule A is therefore not load-bearing alone near its boundary, and must not be treated
as though it were.** That is not a reason to distrust it — inside its bound, where the
floor is nearer than the stop distance, it is the strong rule: continuous, no
confirmation window, and correct whichever explanation is true. It is a reason the two
rules degrade INDEPENDENTLY, which is the entire point of running both. A floor-model
error blinds rule A and leaves rule B intact; a lidar outage removes rule B and leaves
rule A intact. Neither failure is silent, because `~/state` names which rule concluded
what. A single rule covering both bands would have one error budget and one failure
mode, and the 9 mm above is what that would be resting on.

### 9.5 WHERE the background check lives — the seam

**Recommendation: `tof_node` subscribes to `/scan` and applies rule B before publishing
`~/obstacles`.** This is new coupling and it needs the justification the review asked for.

The deciding argument is the fan-out. `~/obstacles` has TWO consumers — the supervisor
brakes on it and the decisive controller steers around it, exactly as
`/camera/low_obstacles` is consumed today. If the background check lived in the
supervisor, the steering law would receive the UNFILTERED set and the two consumers would
be acting on different definitions of "obstacle". Two consumers disagreeing about the
contents of one topic is a defect this project has already paid for. The check must sit
upstream of the fan-out, and upstream of the fan-out is the ToF node.

The secondary argument: rule B is a PERCEPTION judgement ("is this return sub-lidar?"),
not a braking judgement, and it needs the ToF's zone geometry to compute a column's
angular span. That geometry lives in `tof_frame`. Moving the rule into `collision_stop`
would move ToF geometry into the safety path, which is under a standing empty-diffstat
discipline.

Alternatives considered:

| option | why not |
|---|---|
| **Supervisor does it** (it already has `/scan`) | Splits the obstacle definition between two consumers; grows the safety path with perception logic |
| **A separate fusion node** | Adds a third process between two that both already exist, whose silence reads as clear floor; the ToF node would still need `/scan` for its own `~/state` honesty |
| **Both consumers do it** | Two implementations of one rule, guaranteed to drift |

The coupling is made observable rather than hidden: `~/points` continues to publish the
raw per-zone returns with NO background filtering, so any consumer that distrusts our
conclusion can re-derive its own and a recording keeps both. That separation is already
the node's stated design and it is what makes this coupling acceptable.

### 9.6 When the lidar is down — spelled out, three distinct cases

Per the standing rule, degrade toward lidar-only braking. The cases are not the same and
must not share a code path:

1. **Scan MISSING or STALE.** Rule B is UNAVAILABLE, not "false". Its candidate zones are
   DROPPED — never published as obstacles, never published as clear. Rule A continues on
   its own rows. `~/state` reports `background=DEGRADED` with the reason token.
   Note what the supervisor is doing meanwhile: a missing or stale scan is already
   `SENSOR_STALE` there (`max_scan_age_s = 0.30`), which stops the robot. So in the
   dangerous direction this case is largely moot — the rover is not moving on a degraded
   ToF. The requirement is that the ToF node not INVENT a conclusion in the window before
   the supervisor notices.
2. **Scan present, no return at this bearing.** Not a failure — see §9.3. Rule B fires.
3. **Scan present and disagreeing with itself** (stamp not advancing, etc.). Treated as
   case 1. The supervisor's existing `evaluate_scan` taxonomy is the model; the ToF node
   should not invent a second one.

**The net effect of losing the lidar is that the ToF becomes strictly less sensitive,
never more.** Rule B can only ADD obstacles, so its unavailability can only remove them —
it can never produce a phantom brake by being absent.

### 9.7 The fitted constants are CALIBRATION, not a description of the mount

`h = 0.139 m`, `pitch = −15.7°`, `5.9°/zone vertical` reproduce 8 independent floor-row
measurements to **±4 mm RMS**. They are the numbers that make the model predict the
readings. They are **not** a statement about where the sensor physically is: the fitted
height sits ~30 mm above Scott's tape measure, and the most likely mechanism is that each
zone reports the NEAREST returning part of its cone rather than its centre ray — which
biases every downward row short, and a fit absorbs that bias into height and pitch.

Consequences that follow from this and are not optional:

* Re-mounting invalidates these numbers even if the tape says the height is unchanged.
* They must not be quoted as physical facts in the provisioning repo or the run protocol.
* The residual self-check of §3.3 measures the model against the floor, which is exactly
  what these constants were fitted to — so **a small residual is not evidence the fit is
  physically right**, only that it is still self-consistent.

The horizontal zone pitch stays at **7.5°** (independently confirmed: a 25 cm box at
0.30 m filled 6 of 8 columns). The FOV is therefore **asymmetric, ~60° × ~47°**, and
`zone_deg` must split into `zone_deg_h` / `zone_deg_v` — forcing 7.5° on both axes is six
times worse on the fit with a systematic pattern, i.e. a real model error rather than
noise.

### 9.8 What this amendment does NOT have data for — the gates

Stated plainly, because the last version of this design was falsified by its own
revert-proof:

* **`disagreement_margin` is unpinned.** The tilt session recorded 23,057 ToF frames and
  NO synchronised `/scan` — the 0.678 m lidar figure is a single spot probe from the
  crosstalk check, not a time series. So rule B's threshold, its false-fire rate on a bare
  wall, and its detection rate on the box are all UNMEASURED. A capture recording
  `/scan` and ToF frames together is a precondition for rule B holding any authority, and
  the arithmetic in §9.4 is a worked example, not a measurement.

  **BENCH ITEM J — JOINT `/scan` + ToF CAPTURE.** Named so it can ride along rather than
  wait for a session of its own.

  | | |
  |---|---|
  | **effort** | ~5 minutes, chassis OFF, rover stationary — it is a tag-along on any staged session, not a trip of its own |
  | **needs** | lidar up and spinning, ToF capture running, both recorded with timestamps that can be aligned |
  | **scene** | (a) bare flat wall at ~0.7 m, 60 s — the FALSE-FIRE case: the lidar and the ToF must AGREE, and every rule B fire here is a phantom brake at a wall; (b) the same 5 cm object at 0.50 m in front of that wall, 60 s — the DETECTION case, where they must disagree by ~0.18 m |
  | **pins** | `disagreement_margin`, rule B's false-fire rate on (a), its detection rate on (b) |
  | **watch for** | the two clocks. §9.3 compares in `base_link` through TF, so the capture must carry TF or at minimum unambiguous stamps; a 53 s alignment error has already cost this project once |

  **ACCEPTANCE CRITERION, quantitative — added 2026-08-13 after an attempt to settle
  this from existing data failed.** J passes when, over its bare-wall segment, the
  measured per-column disagreement between `/scan` and the ToF stays below the chosen
  `disagreement_margin_m` in at least 99% of frames, and the margin is then set from
  THAT distribution rather than from the current 0.10 m guess. Rule B's authority
  follows the measurement; it is not a number anyone picks.

  **AND A TRAP THAT ALREADY CAUGHT ME, so J's analysis does not repeat it: the background
  is PER COLUMN and a uniform number is not a stand-in for it.** Replaying recorded
  clear-floor frames against a flat 1.90 m background produced 188 "phantoms" which were
  nothing of the kind — that segment's columns 0-4 face a wall at ~2.0 m while columns
  5-7 see a real object at ~1.05 m, sitting 0.03-0.14 m up and therefore BELOW the
  0.19 m lidar plane. Rule B was very likely right and the synthetic background was
  wrong. `scan_min_by_column` exists because the background varies across the frame;
  feeding it a constant asks a question no lidar would ever pose. **Nothing in the
  existing recordings can settle rule B's false-fire rate at all** — which is a stronger
  argument for J than any claim derived from them.

  **(a) is the more important half and is the one that will be skipped if the session
  runs long.** Detection rate is the number everyone wants; the false-fire rate on a bare
  wall is the number that decides whether this rule can drive a brake at all, and it is
  the failure this whole amendment exists to fix. If only one scene gets captured,
  capture the WALL.
* **The stop distance is still underived**, so rule A's row set is still parametric
  (§9.2), and §3.3's in-flight pitch envelope still gates `floor_margin_m`.
* **The 6 mm miss in §9.4 is one object at one range.** It shows the gap exists; it does
  not measure its width.

### 9.9 Revert-proofs for this amendment (to be mutation-verified before code lands)

Numbered on from §7. Each must FAIL against the code it indicts:

9. `row_two_does_not_brake_for_a_wall` — replay the tilt session's wall frames: the
   99.3% obstacle rate becomes ZERO from row 2. BUILT, mutation-verified. It asserts
   BOTH independent reasons (row 2 holds no rule A authority; rule B agrees with the
   lidar) and asserts that the REMOVED rule still fires on the same frames — without
   that last line the proof passes against code that detects nothing at all.
10. `rule_a_row_set_follows_the_stop_distance` — BUILT, mutation-verified, anchored on
    rows 2 and 3 by name.
10b. `rule_a_authority_is_independent_of_the_floor_horizon` — **added because proof 10
    was INERT.** A mutation deleting rule A's authority bound outright changed no test:
    at a 0.45 m stop distance the 0.55 m visibility horizon was already stricter, so
    `expected_floor_m` returning None past the horizon was still making the authority
    call — the very confusion 9.1 diagnosed, surviving inside its own fix. Same shape as
    the pivot controller below an unreachable branch. `nearer_than_floor` now uses the
    UNCAPPED floor reading and the bound is the only gate.
10. `rule_a_row_set_follows_the_stop_distance` — changing the stop distance changes which
    rows hold rule A, computed not hardcoded. Fails against a frozen row list.
11. `rule_b_needs_the_lidar_to_disagree` — a ToF return at the same range the lidar
    reports produces NO obstacle; the same return with the lidar reporting open space
    DOES. Fails against absolute proximity thresholding.
12. `missing_scan_drops_rule_b_zones` — with no scan, rule B's zones appear in NEITHER
    the obstacle set nor a clear conclusion, and `~/state` says DEGRADED. Fails against
    a version that treats a missing scan as open space (which would brake on everything)
    or as agreement (which would brake on nothing).
13. `row_zero_stops_above_the_lidar_plane` — a row 0 return beyond 0.59 m produces no
    rule B obstacle however much the lidar disagrees. Fails against a version that
    applies rule B to every row.
14. `bearings_are_compared_in_base_link` — a synthetic scan built with the real ~179°
    laser yaw must not mirror the comparison. The N1 lesson, pre-empted for a second time.
15. `asymmetric_fov_is_pinned_by_measurement` — the vertical zone pitch is fixed against
    today's capture, and forcing it equal to the horizontal one fails the fit. BUILT and
    mutation-verified: symmetric 7.5 deg gives 31 mm RMS against 6 mm, with residuals
    that are ALL ONE-SIGNED. The sign pattern is the assertion; RMS alone could be noise.
16. `the_sensor_reports_along_its_own_boresight` — the 0.60 m wall's row gradient is
    predicted within 6 mm from ONE solved parameter. Fails against the base_link-x
    projection that shipped (defect 1, 9.10). BUILT and mutation-verified.
17. `the_mount_pitch_is_a_rigid_rotation` — the angle between any two zone rays is
    unchanged by pitch. Fails against adding pitch to elevation (defect 2, 9.10). BUILT
    and mutation-verified. WEAKEST proof here: an invariant, not a measurement — see 9.10.

### 9.10 TWO GEOMETRY DEFECTS IN SHIPPED STAGE-(i) CODE, found while building 9.2

Neither was in the review that approved this amendment's scope. Both were found by
trying to pin the vertical zone pitch against today's data and discovering the fit
would not reproduce — a fixture doing the job a fixture is for.

**Defect 1 — the reading was projected onto the wrong axis.** Every projection in
`tof_frame` used the ray's WORLD elevation, i.e. it assumed the sensor reports distance
along `base_link` x. It reports along its own boresight, which §1 of this note settled
by measurement. Against eight recorded clear-floor medians the floor model
under-predicted by 19–42 mm, **all negative, 29.7 mm RMS**; with the boresight
projection the same constants land at **5.6 mm RMS with mixed signs**.

**Defect 2 — the mount pitch was applied as an addition, not a rotation.** Adding the
pitch to each zone's elevation is exact on the centre column and wrong by **1.8° at the
corner zone** at −15.7°, moving that zone's floor intersection by ~5%.

**Why both shipped:** at the old +4° mount, defect 1 was worth a fraction of a percent
and defect 2 was worth 0.4°. Both sat far inside `floor_margin_m` and no test could see
them. The mount angle chosen to see low obstacles is also what converted two harmless
roundings into systematic errors — *a level bench could not have found either*, which is
the same lesson as §1's reporting convention, arriving twice in one session.

**The clean-up this forced, and its cost.** The vertical zone pitch, the mount height
and the pitch are strongly correlated in a floor-only fit — three parameters, few
constraints. The 0.60 m WALL is what breaks the degeneracy: a plane's row-to-row
gradient depends on pitch and zone pitch but NOT on mount height. Solving the wall
distance from row 0 alone and then predicting rows 1-3 lands within 6 mm, and the solved
distance (0.578 m) sits sensibly behind the 0.60 m tape. That is an independent check,
and it is why the wall segment is now a fixture in its own right.

**The level-mount geometry could NOT be repaired.** Its constants were fitted under both
defects AND a 7.5° vertical assumption, and its clear-floor segment has every row from 1
to 7 reading ~300 mm — the side clutter Scott flagged at the time — so there is no floor
gradient left to re-fit against. The level fixtures are therefore frozen with the
geometry they were recorded under, explicitly labelled as known-wrong-and-unrepairable,
and **demoted**: they remain evidence about the sensor and the scene (validity,
sentinels, run lengths, adjacency separation, detection rates) and are no longer evidence
about geometry. Geometry is pinned by the tilt fixtures, which come from clean data.

**One proof is weaker than the rest and is marked as such.** The rigid-rotation property
is a geometric invariant, not a measurement: today's edge columns cannot settle it,
because across nominally flat floor row 6 reads 247/267/282/271/267/261/252/243 mm — a
~35 mm spread against the ~10 mm the two conventions differ by. A cleaner edge-column
capture would replace it with evidence.

---

## 10. REQUIREMENT — traversable terrain is not an obstacle (Scott, 2026-08-13)

Verbatim: *"I think the rangefinder may be seeing the ridge (about 3/5 of an inch) high
that is the mat my chair sits on. Anything smaller than say 0.7" should be ignored."*

**Returns less than `min_obstacle_height_m` (0.018 m) above the fitted floor are
TERRAIN and are dropped before either detection rule.**

**Which number wins, and why it is not derived.** 0.7 inch is 17.8 mm; 18 mm ships.
This is an OPERATOR SPEC, not a derivation — no documented RVR climb capability exists
in this repo or in Sphero's published material, so there is nothing to derive from, and
a threshold reverse-engineered from wheel radius would be a guess wearing a derivation's
clothes. If a real climb spec surfaces and is LOWER, it wins; if HIGHER, 18 mm stays,
because being *able* to climb something is not a reason to drive into it.

**It is resolvable, not wishful.** At the shipped geometry an 18 mm height difference is
~44 mm of range in row 5 — more than ten times the ~3 mm range noise.

**THE CAMERA COULD NOT HAVE IMPLEMENTED THIS AT ALL, and that is a capability argument
rather than a reliability one.** A monocular floor-boundary reports WHERE the floor stops,
never HOW HIGH the thing stopping it is. Height is precisely the quantity it does not
measure, so "ignore anything under 18 mm" is not a threshold the camera layer could have
been given. The ToF measures height directly.

### 10.1 The gate was INERT when first wired, and the reason generalises

Deleting it from either rule changed no test. `floor_margin_m = 0.12` already refuses
anything shorter than **27 mm (row 3), 51 mm (row 5), 76 mm (row 7)** — up to 4× the
spec. The margin was doing terrain classification by accident.

Same shape as `floor_horizon_m` making authority decisions (§9.1) and as proof 10's
inertness (§9.9): two different questions answered by one constant, agreeing today and
diverging later. **Here the divergence is already scheduled** — §3.3 requires
`floor_margin_m` to be RE-DERIVED once the in-flight pitch envelope is measured. If that
shrinks it, a stack leaning on the margin for terrain would silently begin braking for
mats, and the symptom would be *a rover that got more timid after its floor model got
better*. The proof therefore asserts the gate where the two constants come apart, at a
tightened margin, and includes a vacuity check so it cannot pass by testing nothing.

### 10.2 Stage (ii)'s comparison basis has changed — the camera cannot be the reference

Scott, 2026-08-13: *"I thought we removed the camera as a guide. It isn't mounted
properly so any feedback would be terrible."* The mount moved, most likely during the
ToF mount work, and nobody noticed — **§1.0's "the camera mount is fixed" assumption died
silently**, which is worth more than the incident: an assumption held by physical
convention and never re-measured is not a constraint, it is a habit.

Consequences:

* **Stage (ii) is no longer ToF-versus-camera.** The camera cannot serve as the reference
  sensor while its aim is unknown. The comparison becomes ToF versus GROUND TRUTH —
  Scott's eyes, the lidar where it overlaps, and freeze/touch events.
* The 2026-08-13 mission is **contaminated and excluded** — archived under
  `03_validation/CONTAMINATED_mission_2026-08-13_mismounted_camera/` with his verdict
  (*"Don't make changes based on that run - it was rubbish"*). No threshold, no
  mat-ridge finding and no escape observation is taken from it.
* **Bench item J and the sun measurement are unaffected** — neither involves the camera.
* **The sun rule transfers pro tem**: missions avoid hard direct sun until the ToF sun
  capture happens. It is a scheduling rule for the operator, not a code gate.
* **Do not re-aim the mount.** Scott owns physical setup; if it is ever re-mounted for
  Track 2, it gets measured then rather than assumed.

---

## 11. DERIVING THE ToF BRAKE — and the three wrong operands on the way

Building the authority batch required deriving the brake's stop distance. The first
attempt concluded the batch was blocked. **That conclusion was wrong** and 11.2 records
why, because how it went wrong is more useful than the number it produced.

### 11.1 Rule A's reach is not its floor distance

A zone fires only below `floor − floor_margin`, so the obstacle must be that much nearer
than the floor. The **margin sits between the floor and the detection range**, and
reading the floor distance as the reach overstates it by the whole margin:

| row | floor reads | fires below | → GROUND RANGE | obstacle height there |
|---|---|---|---|---|
| 3 | 0.629 m | 0.509 m | **0.498 m** | 0.027 m |
| 4 | 0.434 m | 0.314 m | **0.298 m** | 0.039 m |
| 5 | 0.330 m | 0.210 m | 0.194 m | 0.051 m |
| 6 | 0.265 m | 0.145 m | 0.129 m | 0.063 m |
| 7 | 0.219 m | 0.099 m | 0.085 m | 0.076 m |

At the shipped 0.45 m authority bound, rule A's rows are 4–7, so **its reach is 0.298 m**.

> **SUPERSEDED 2026-08-13 by the frame batch — the table above is in the BROKEN frame.**
> Every "GROUND RANGE" in it omits the sensor's 0.10 m forward offset. Corrected, and
> with the authority bound now compared in `base_link` as it must be:
>
> | row | floor reads | floor FROM base_link | fires below | → GROUND RANGE |
> |---|---|---|---|---|
> | 3 | 0.629 m | 0.716 m | 0.509 m | **0.598 m** |
> | 4 | 0.434 m | **0.512 m** | 0.314 m | **0.397 m** |
> | 5 | 0.330 m | 0.405 m | 0.210 m | **0.294 m** |
> | 6 | 0.265 m | 0.337 m | 0.145 m | 0.229 m |
> | 7 | 0.219 m | 0.289 m | 0.099 m | 0.185 m |
>
> **Row 4 leaves the row set.** Its floor is 0.512 m from `base_link` — outside the
> 0.45 m bound — although it READS 0.434 m, which is inside. Rule A's rows are now
> **5–7 and its reach is 0.297 m** (zone (5,0); the centre column gives 0.294 m).
>
> Note what did NOT happen: the reach did not move by the offset. Every row's reach
> gained exactly +0.100 m, and the reach still fell from 0.298 to 0.297 because the row
> supplying it changed. A check that the number "still looks right" would have passed.
> The floor comparison moves by 0.070–0.108 m instead, because it crosses the boresight
> projection as well as the offset — which is why the constants were recomputed rather
> than shifted.

### 11.2 CORRECTED — the comparison in the first draft was wrong twice

The first version of this section compared rule A's 0.298 m against the lidar's 0.30 m
`stop_distance_m` and concluded the braking room was negative. **Both halves of that were
wrong, and they compounded.**

**Error 1: it is not redundancy, because the lidar cannot see this object class at all.**
The lidar's 0.30 m stop applies to things the lidar SEES. A 5 cm rail is invisible to it
at every distance — that is the entire reason this layer exists. Without it the rover
reaches that rail at full cruise and learns about it by contact. Rule A at 0.298 m is not
2 mm inside an existing stop; **it is the only detection that class gets.**

**Error 2: 0.30 m is a THRESHOLD, not a stopping distance.** It is a number chosen to
exceed the physical requirement with margin, so comparing a detection range against it
asks the wrong question. The requirement is in the config's own arithmetic:

    footprint_front 0.11 + payload_margin 0.02 + braking_margin 0.02 = 0.150 m
    (the config comment cites a 0.045 braking term giving 0.175 m; deployed says 0.02)

**Rule A's 0.298 m clears that by 0.12–0.15 m.** Taking the more conservative 0.175 m,
rule A has 0.123 m of margin over the distance in which the rover must physically stop.

### 11.3 So rule A alone IS a viable brake — a short-range one

For sub-lidar obstacles, rule A detects at 0.298 m and needs ~0.175 m. It can deliver a
clean stop. It is not inert, it is not touch-softening, and it is not redundant with the
lidar. It is a **short-range brake for a class that currently has no brake at all.**

**Rule B changes the quality of that stop, not its existence.** Row 3 reaches 0.498 m —
0.20 m of extra warning, which at 0.20 m/s is a full second. That buys an earlier, gentler
stop and room to steer around rather than halt. A real improvement, and **not a
precondition.**

**The corrected sequencing:** bench item J remains the right thing to do first and it is
still five chassis-off minutes, but it is a RANGE upgrade rather than a gate on the batch
existing. If J fails to pin rule B — bad margins, unusable false-fire rate — the fallback
is not a dead batch, it is rule-A-only short-range braking, which is already worth more
than the nothing the rover has today. Recorded now, while nobody is invested in the answer.

### 11.4 What the episode is actually about

Three wrong conclusions in one derivation, each from comparing against the nearest
available number instead of the one the question needed: the floor distance instead of
the floor-minus-margin reach, the lidar's threshold instead of the physical stop
requirement, and a sensor that cannot see the object class at all instead of one that
can. **The arithmetic was right every time; the operands were wrong.**

That is the week's theme once more — one constant answering two questions — in its most
expensive form yet, because here it nearly cancelled a batch that was sound.

### 11.45 The state token — configuration, never capability

`~/state` reports `rules=rule_a_only` or `rules=rule_a+b`, and the choice of vocabulary
is the point. A capability word like `short_range` or `full` **drifts the moment a
constant moves** — `short_range` would have been written during the retracted draft of
11.2, when rule A's reach was believed redundant with the lidar, and it would still read
`short_range` today after the correction that made it a real brake. The word would have
outlived the reasoning that produced it, in a recording nobody could re-derive.

A configuration word stays true by construction: `rule_a_only` describes which rules ran,
which is a fact about the run rather than a claim about the robot. Report what IS and let
the reader judge what it means (D35).

**The gate lives on the DETECTOR, not on the consumer.** Rule B decides what it
publishes; a consumer-side gate would leave the detector claiming obstacles nobody was
permitted to act on, which is two components disagreeing about what is true — the seam
class this project keeps paying for.

### 11.5 What the guard now guards

`tests/test_tof_brake_derivation.py` no longer blocks the batch — the batch is not
blocked. It pins the three operands so a future reader inherits the derivation rather
than the conclusion: rule A's reach must be measured as floor MINUS MARGIN, the stop
requirement must be the footprint-plus-braking sum and not the lidar's threshold, and the
lidar must never be treated as covering the sub-lidar class.

It also keeps the `# RULE B PINNED BY:` citation requirement, but scoped to what it
honestly gates: **rule B's own authority**, not the brake as a whole. Rule A may fly
without bench item J; rule B may not fly without it. Mutation-verified in both
directions.

**Lidar core constants stay untouched, and that is now a decision rather than an
omission.** `stop_distance_m`, `slow_distance_m` and the cruise speed all have their own
flight history and nothing in this derivation argues for moving any of them.
