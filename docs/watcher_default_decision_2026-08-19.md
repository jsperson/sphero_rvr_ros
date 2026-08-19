# Decision memo: refusal_watcher launch default — OFF → ON for stock flights

**Asks Scott for one word on one line:** flip `start_refusal_watcher`'s launch
default from `false` to `true` for the stock middle. Nothing here moves without
that word; the flag itself stays (OFF remains one launch arg away).

## What the mechanism is

Option D: when navigation livelocks against an obstacle no sensor can see (the
RPP refusal loop — recoveries mounting, zero progress), the watcher REQUESTS a
mark at the refusal centroid; contact_marker (sole author of `/contact_marks`)
plants it mission-permanent; the planner routes around it thereafter. The
watcher has zero motion authority — it only asks, and only contact_marker
grants.

## The evidence ledger, in certification order

1. **Rig-certified 2026-08-18** (four-arm campaign, falsifier-first): fires on
   the livelock signature, self-quenches when the mark takes ("no delta
   cells"), cooldown suppresses repeats, evidence-freshness gate (3.0 s,
   derived from the 15% flicker class) refuses stale snapshots.
2. **Field ride-along, same evening** (flight d45bd24, `--ride-along-watcher`
   deviation logged at bringup, Scott at the switch):
   - Run 1, staged sub-lidar boot: 15 s pin, 9 recoveries — then **one
     promotion, centroid (0.586, 0.003), centimetres from the boot**, self-
     quench on the next window, same goal SUCCEEDED via the detour with no
     resend. Scott, verbatim: *"That was excellent navigation!"* (the jerkiness
     he noted was entirely the pre-promotion pin — the disease, not the cure).
   - Run 2, the persistence capstone: **the very first plan curved around the
     remembered boot** (0.423 m clearance read before wheels), zero pin, zero
     new promotions, goal in 20 s vs run 1's 40. Learning once, remembering
     after — the product behavior this project has chased since D25.
   - Zero false promotions across both goals; zero stalls; zero contact.
3. **Mission-scale negative test, 2026-08-19** (explore-on-stock campaign,
   watcher ON in every arm): across a full certified 482 s autonomous mission
   plus five falsifier/cert arms — **zero promotions, zero non-empty marks**,
   formally scored as bar B5. In a room with nothing invisible, the watcher
   said nothing, all night. (Claimed as exactly that and no more: the rig has
   no contact physics, so this is the no-false-fire half only — the positive
   half is the ride-along's.)

## Residual risks, named

- **A bad mark is permanent** (the camera-2026-08-08 class: a false mark in the
  global costmap can close a doorway for the mission). Bounded by four
  independent gates (sustained window 12 s + stall bar 0.15 m + recoveries ≥ 2
  + freshness 3.0 s), plus cooldown and self-quench; the field record is one
  promotion, centimetre-accurate. Rollback is one launch arg.
- **D52 adjacency**: every mark adds refusal surface for behavior_server's
  recoveries (D36-on-stock, open landmine row). Watcher marks appear only
  where a livelock already was — the exposure increment is small and sits on
  an already-filed row.
- **Sample size**: the positive lane has one field promotion. The negative
  lane (no false fires) carries far more coverage — two field goals plus the
  entire rig campaign.
- Adjacent-but-separate: the touch-port v1 open rows (tof_layer blind < 0.27 m;
  /contact_marks QoS silence) are unchanged by this decision.

## Recommendation

**Flip the default to ON.** Without the watcher, every sub-lidar obstacle costs
each mission its own 15 s pin and 9 recoveries — every time, with no memory,
and the livelock can outlast the goal. With it, the cost is paid once and the
room is learned. The false-fire risk is the one that matters, and it is now
bounded by four gates, a rig campaign, a clean field flight, and a full
certified mission of silence.

**Condition attached**: first default-ON flight is a watch item — every
promotion inspected post-flight against the room's ground truth; one false
promotion reverts the default the same day (one launch arg, no code).

*Worker memo 2026-08-19; PM-reviewed before reaching Scott. The default does
not move on this document — it moves on Scott's word.*
