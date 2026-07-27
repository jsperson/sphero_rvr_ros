# M7.3 directional-veto addendum

## Verdict

PASS on reviewed and deployed source
`8f020c84ffbbcd0f3eb7ad642e938794cfe0c39f`.

This addendum closes the directional-veto evidence residual identified during
review of the accepted M7.3 collision gate. It preserves the original M7.3
evidence digest unchanged:

- parent M7.3 evidence SHA-256:
  `7e2636f100ffad724477f1e6287458d0708057c3ee93f26d5dd6f52432281f55`;
- directional addendum SHA-256:
  `638abb8f293781adcf3827a486cf700b96693f1172d6fb058c40b79a8b8f4130`;
- read-only observation SHA-256:
  `7efe0f1c4d0f90db5c8b3b2bf114ac5feccd0c72e6adb1d0c95ededcd3c8e0ba`.

The operator explicitly confirmed that the rover made no contact with the box
or any other object.

## Paired physical result

The rover remained in one pose beside the same rear-left box return. The first
reverse state measured `0.1505 m` rear and `0.2005 m` left clearance; the first
forward state measured `0.15025 m` rear and `0.2015 m` left clearance.

| Direction | Requested behavior | Downstream result |
| --- | --- | --- |
| Toward obstacle | Three bounded `-0.05 m/s` `/cmd_vel` samples | Thirteen supervisor states reported `SLOW reason=rear_hold`, `requested=(-0.050,0.000)`, and `output=(0.000,0.000)`. All 106 downstream motor samples in the paired interval were zero. |
| Away from obstacle | Three bounded `+0.05 m/s` `/cmd_vel` samples | Eleven supervisor states reported positive requested and output velocity while excluding `89-99` already-overlapping points. Eleven downstream motor samples were `+0.05 m/s`, followed by motor zero after the command lease expired. |

This proves the physical policy exercised by PR #49: motion toward the known
rear obstacle remains blocked, while straight translation away is permitted
only when the sampled clearance is non-decreasing. No threshold or tolerance
was changed.

## Authority disclosure

A bounded `ros2 topic pub --times 3` process was used for each direction so the
test could address the collision supervisor directly. It published only to
`/cmd_vel`; it never published to `/cmd_vel_motor`.

The idle `live_route_runner` publisher endpoint remained present, so
`/cmd_vel` temporarily had two publisher endpoints during this targeted
addendum. The independent collision supervisor remained the sole
`/cmd_vel_motor` publisher. This does not replace or alter the exclusive
authority graph accepted in the parent M7.3 evidence.

The read-only observer captured all three reverse `/cmd_vel` samples. For the
second short-lived CLI publisher, it captured the supervisor's positive
requested/output state and the downstream positive motor samples but did not
capture the upstream samples. The forward claim is therefore bound to the
supervisor input state and motor output, not to an inferred command.

## Evidence inventory

- `report.json` contains the passing checks, measurements, disclosures, parent
  digest, and addendum digest.
- `relevant_events.jsonl` contains the ten observation events referenced by the
  report.
- `raw_artifact_sha256.txt` binds the full observation and the committed report
  and event subset.

The full `7,116,817`-byte observation remains on `sphero-pi-2` at:

```text
/home/jsperson/rvr_runs/m7-phase3-directional-addendum-20260727/
  supervisor-paired-observation.json
```

It is checksum-bound rather than committed to Git.

## Cleanup and scope

The adaptive hardware unit, driver, route runner, collision supervisor, and
lidar were stopped after capture. The lidar is powered down.

M7.4 remains locked. Its separate approval must bind both the parent M7.3
evidence digest and the directional addendum digest above. Camera-pitch and
far-band floor-projection re-verification remain independent entry conditions;
the `0.050 m` error bound must not be widened.
