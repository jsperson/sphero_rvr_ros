# Direct drivetrain bench

`scripts/rvr_drivetrain_bench.py` is an attended diagnostic that talks directly
to the Sphero RVR serial protocol. It does not start or import ROS, Nav2, the
mission service, the LLM/provider, the route bridge, the collision supervisor,
or an approval/evidence path. Stop every process that might own the rover UART
before running it.

This is intentionally not a mission or a deployed-stack speed change. Its
purpose is to measure the unloaded and loaded command thresholds that later
stack tuning must respect.

## Safety

- Keep an operator beside the rover with its power cut reachable.
- The default is wheels-up `bench` mode. Support the chassis securely with all
  wheels clear of the ground.
- Floor mode has no collision or drop-off software protection. Use only a level,
  bounded, clear area with no stairs, ledges, pets, or people in its path.
- Every command is followed automatically by both raw-motor zero and tank-SI
  zero. Pulse duration is hard-capped at 0.75 second, raw duty at 96/255, and
  tank-SI track velocity at 0.30 m/s.
- SIGINT, SIGTERM, exceptions, normal exit, and interpreter exit all attempt
  zero. If serial communication is lost, the script prints `CUT RVR POWER NOW`;
  software cannot confirm a stop through a failed connection.

## Run unloaded first

```bash
python3 scripts/rvr_drivetrain_bench.py \
  --bench \
  --i-am-present \
  --output-prefix ~/rvr-bench-unloaded
```

The default sweep tests forward and pure-turn commands in both representations:

- raw motor duty: `0..96` in steps of `4`;
- native tank SI: `0.00..0.30 m/s` in steps of `0.01 m/s`.

Run only one representation or axis with `--representation raw_duty`,
`--representation tank_si`, `--axis forward`, or `--axis turn`.

## Run loaded on the floor

Move the rover fully off ridges and mats first. Floor mode requires both the
operator acknowledgement and the separate clear-area acknowledgement:

```bash
python3 scripts/rvr_drivetrain_bench.py \
  --floor \
  --floor-area-clear \
  --i-am-present \
  --output-prefix ~/rvr-bench-loaded
```

Floor mode pauses before every nonzero pulse. Reposition the stopped rover when
needed and recheck the clear path and drop-off boundary before pressing Enter.

Start with one axis or a lower ceiling if the available clear distance is
limited. CLI ceilings may be lowered but cannot exceed the hard-coded limits.

## Output and interpretation

The script rewrites `<prefix>.csv` and `<prefix>.json` after every completed
pulse using atomic temporary files, so an interrupted run retains completed
measurements. It refuses to overwrite an existing prefix. Each row includes
mode, representation, axis, commanded value, left/right encoder deltas,
estimated displacement and yaw, and `moved`, `sustained`, and `smooth` flags.
The JSON also retains every in-pulse encoder sample. In wheels-up mode,
displacement means encoder-estimated tread travel rather than chassis travel;
turn yaw uses the existing rough effective track-width calibration.

- `moved` means total directional progress crossed the small measurement floor.
- `sustained` requires progress in at least three intervals, at least 60% active
  intervals, and at least 80% direction consistency. A one-time twitch is not
  breakaway.
- `smooth` additionally requires at least 80% active intervals, completely
  consistent direction, and coefficient of variation no greater than 0.45 for
  active progress rates.

The printed/JSON summary reports the first sustained command as breakaway and
the first later smooth command as the smooth-above-breakaway threshold, for each
representation and axis. Bench results are unloaded thresholds; only floor mode
measures the loaded thresholds relevant to later mission-stack tuning.
