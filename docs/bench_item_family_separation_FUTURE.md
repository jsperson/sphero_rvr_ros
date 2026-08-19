# FUTURE bench item — the family-separation experiment

*Approved for a FUTURE sitting's card (consensus 2026-08-19); deliberately NOT
on the 2026-08-19 card, whose scope Scott fixed. Filed following the
`run_card_arc_rate_FUTURE.md` precedent so the wording survives until a sitting
wants it.*

## The question it separates

The 2026-08-19 flight's pivot stalls happened UNDER firmware closed-loop
velocity control (`drive_tank_normalized` carries velocity targets, per the
vendor SDK; the firmware's stall protection tripped rather than powering a
from-rest pivot through this floor's scrub). Two hypotheses survive that fact:

- **H-scale**: the normalized command's authority/scaling is the limit — the
  same firmware family, asked in SI units at a healthy tread target, powers
  through where normalized-45 stalled.
- **H-family**: the velocity controller family itself protect-trips on
  from-rest pivots here regardless of command flavor — only a DIFFERENT
  control system (the heading loop, already bench-passed at 21/21) has the
  authority.

One minute on the floor separates them.

## The measurement (feedback only — NO duty constant anywhere, per Scott's
fixed-duty veto)

Rover at rest on the open floor (the flight's own regime). Via a diagnostic
that speaks `drive_tank_si_units` directly (supervisor estop reachable, same
staging discipline as every motor-capable bench item):

1. Command a pure-pivot differential at tread targets ±0.445 m/s — the SI
   equivalent of the flight's commanded 3.55 rad/s (±wz·track/2, track
   0.2507 m) — for 3 s, from rest.
2. Record: achieved yaw rate (odom/IMU), `motor_stall_events` delta, and
   whether the firmware's stall protection trips.
3. Repeat ×3; once each direction at minimum.

**Readout:** clean rotation ≥ ~80% of commanded with zero stall events →
H-scale (the SI lane has authority the normalized lane lacked; D58's parked
route gains a fact, though its clamp story still stands). Stall-protect trips
from rest → H-family confirmed (the heading loop is the only pivot authority;
D58 closes as moot when the heading direction covers in-path turns).

Either answer is one more measured wall for the feedback-owns-correctness
architecture; neither produces a tunable.
