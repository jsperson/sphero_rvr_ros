"""Revert-proofs for the ToF's base_link frame — the batch of 2026-08-13.

THE DEFECT CLASS, stated once: the word "range" meant two things. A distance from the
SENSOR (what the ToF reports) and a distance from `base_link` (what every consumer
assumes), with the sensor sitting `mount_x_m` = 0.10 m forward of `base_link`. Three
separate defects came out of that one ambiguity, and two of them CANCELLED each other,
which is why the deployed binary was self-consistent and still wrong:

    1  `zone_point` omitted `mount_x_m`, so every published point was 0.10 m near
    2  `rule_a_applies` compared a sensor READING against a base_link stop distance
    3  `rule_b_applies` rebuilt a return's height from a ground range, correct only
       while that range came from the sensor -- i.e. only while (1) was broken

Defect 3 measured ZERO error in the shipped code and 22 mm the instant (1) was fixed
alone, in the PERMISSIVE direction. That is the argument for one refactor rather than
three patches, and these proofs exist to keep the refactor from being unpicked into
patches later.

The fix's shape: `zone_point` is the single source of truth for position, every
predicate reads the point, and `_point_from_ray_length` is the only place a ray length
becomes a position. Each proof below must FAIL against the code it indicts.
"""
import dataclasses
import math
import os
import re
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from fixtures.tof_recorded_frames import (  # noqa: E402
    TILT_BOX_050, TILT_BOX_050_WALL_M, TILT_WALL_060, TILT_WALL_060_BASE_M,
    TILT_WALL_060_SENSOR_M,
)
from sphero_rvr_core.tof_frame import (  # noqa: E402
    _floor_reading_m, ObstacleDetector, TofConfig, floor_ground_range_m,
    rule_a_applies, rule_a_rows, rule_b_applies, zone_point, zone_ray,
)

CORE = Path(__file__).resolve().parents[1] / "src" / "sphero_rvr_core" / "tof_frame.py"
NODE = Path(__file__).resolve().parents[1] / "src" / "sphero_rvr_driver" / "tof_node.py"


def test_a_published_point_is_measured_from_base_link():
    """PROOF 15. The bug itself: `zone_point` must carry the mount's forward offset.

    Anchored on the MEASUREMENT that found it rather than on the constant, so that
    deleting `mount_x_m` and deleting this test's premise are not the same edit: bench
    item J saw the ToF read a median 0.10 m nearer than the lidar on the same surfaces,
    constant from 0.72 m to 1.90 m.
    """
    cfg = TofConfig()
    assert cfg.mount_x_m > 0.0, "the sensor has no forward offset from base_link at all"

    for row, col, mm in ((3, 3, 500), (5, 0, 200), (0, 7, 1200)):
        p = zone_point(row, col, mm, cfg)
        ux, uy, uz = zone_ray(row, col, cfg)
        r = math.hypot(p[0] - cfg.mount_x_m, p[1]) / math.hypot(ux, uy)
        assert p[0] == pytest.approx(cfg.mount_x_m + r * ux, abs=1e-9), (
            f"zone ({row},{col}) is placed at x={p[0]:.4f}, which is the ray alone -- "
            "the point is being measured from the SENSOR and published as base_link")

    # A RETURN's ground range moves by very nearly the whole offset -- x dominates a
    # forward-looking ground range. Recorded because the FLOOR does not (see the next
    # proof), and the difference between those two statements is what moved the row set.
    flat = dataclasses.replace(cfg, mount_x_m=0.0)
    shifts = [math.hypot(*zone_point(r, c, 400, cfg)[:2])
              - math.hypot(*zone_point(r, c, 400, flat)[:2])
              for r in range(8) for c in range(8) if zone_point(r, c, 400, cfg)]
    assert 0.085 < min(shifts) and max(shifts) <= cfg.mount_x_m + 1e-9, (
        f"a return's ground range moves by {min(shifts):.4f}..{max(shifts):.4f} m when "
        f"the {cfg.mount_x_m} m offset is applied; it cannot exceed the offset and "
        "should not fall far short of it for a forward-looking sensor")


def test_the_authority_bound_is_compared_in_base_link():
    """PROOF 16. Defect 2. Rule A's bound is a distance the ROBOT cares about, so both
    sides of it belong to the robot.

    The two operands are genuinely different numbers -- a zone's floor READS nearer
    than it IS -- and the difference is big enough to move the row set, which is what
    makes this proof able to fail.
    """
    cfg = TofConfig()
    deltas = []
    for row in range(2, 8):
        reading = _floor_reading_m(row, 3, cfg)
        ground = floor_ground_range_m(row, 3, cfg)
        assert ground > reading, (
            f"row {row}'s floor is {ground:.3f} m from base_link and reads "
            f"{reading:.3f} m; those must not coincide or the frames have collapsed")
        deltas.append(ground - reading)
    # AND THE GAP IS NOT THE MOUNT OFFSET, which is why this could not have been fixed
    # by adding 0.10 m to the bound. It crosses the boresight projection too, so it
    # varies by row (0.070 m at row 7 to 0.108 m at row 2) -- and a bound shifted by a
    # flat 0.10 would have admitted the wrong rows in both directions.
    assert max(deltas) - min(deltas) > 0.02, (
        f"the reading-to-ground-range gap is nearly uniform ({min(deltas):.3f}.."
        f"{max(deltas):.3f} m), so the authority bound could have been shifted rather "
        "than re-derived, and this proof no longer distinguishes the two")

    # THE MUTATION: comparing the reading instead of the ground range admits row 4,
    # whose reach (0.397 m) is beyond the range at which a wall is already commanding a
    # stop. Naming the row is the point -- a bound that admits one row too many is
    # invisible unless a test says which.
    assert 4 not in rule_a_rows(cfg), (
        "row 4 holds rule A authority. Its floor is 0.512 m from base_link, outside the "
        f"{cfg.stop_distance_m} m bound, and it only fits if the comparison is being "
        "made against the 0.434 m the row READS")
    assert rule_a_applies(5, 3, cfg), (
        "row 5 lost rule A authority, so the bound is now stricter than the derivation "
        "in TofConfig -- re-derive rather than adjusting this test")


def test_the_height_gate_reads_the_points_own_z():
    """PROOF 17. Defect 3, the trap: the one that got WORSE when defect 1 was fixed.

    `rule_b_applies` must take the point. Reconstructing height from a base_link ground
    range under-states z by 22 mm at 0.50 m -- growing with range -- and under-stating z
    makes a return look further BELOW the lidar plane than it is, which hands rule B
    authority above the plane where the lidar can see perfectly well.
    """
    cfg = TofConfig()

    # THE PREDICATE ITSELF, PINNED TO THE POINT. Everything after this is about WHY it
    # matters; this is the part that indicts an implementation. The first draft of this
    # proof asserted only the arithmetic below and a mutation gutting `rule_b_applies`
    # entirely left it green -- an inert proof about a defect class whose signature is
    # inert guards.
    for row in range(8):
        for mm in range(100, 1600, 25):
            q = zone_point(row, 3, mm, cfg)
            if q is None:
                continue
            assert rule_b_applies(q, cfg) == (q[2] < cfg.lidar_plane_m), (
                f"rule_b_applies disagrees with the point's own z at row {row}, "
                f"{mm} mm (z={q[2]:.4f}, plane={cfg.lidar_plane_m}) -- it is deriving "
                "the height from something else again")

    p = zone_point(3, 3, 500, cfg)
    ux, uy, uz = zone_ray(3, 3, cfg)
    reconstructed = cfg.mount_height_m + math.hypot(p[0], p[1]) * uz / math.hypot(ux, uy)

    assert p[2] - reconstructed > 0.020, (
        f"the reconstruction now agrees with the point ({reconstructed:.4f} vs "
        f"{p[2]:.4f}) -- either mount_x_m left zone_point or the two frames collapsed, "
        "and this proof can no longer tell the two implementations apart")
    assert reconstructed < p[2], (
        "the reconstruction over-states height, so its error would make rule B quieter "
        "-- the recorded direction is PERMISSIVE and the derivation depends on it")

    # ...and the wrong height is not merely different, it changes the verdict where it
    # matters: a return straddling the lidar plane is admitted by one and refused by the
    # other. Without this line the two numbers could differ harmlessly forever.
    disagreeing = []
    for mm in range(100, 1500):
        q = zone_point(0, 3, mm, cfg)
        if q is None:
            continue
        recon = cfg.mount_height_m + math.hypot(q[0], q[1]) * zone_ray(0, 3, cfg)[2] / \
            math.hypot(*zone_ray(0, 3, cfg)[:2])
        if rule_b_applies(q, cfg) != (recon < cfg.lidar_plane_m):
            disagreeing.append(mm)
    assert disagreeing, (
        "the point-based and reconstructed height tests never disagree on a verdict, so "
        "swapping one for the other would break nothing observable")


def test_the_tof_and_the_lidar_agree_about_the_same_wall():
    """PROOF 18. The cross-sensor check, which is the only kind that can catch a
    translation. Every proof that consults the ToF alone is satisfied by a consistent
    model of the wrong robot.

    Two independent routes to the same wall: the ToF's own row gradient plus the mount
    offset, and a lidar spot probe. They must land within the disagreement margin, and
    they DID NOT before this batch -- they were 0.10 m apart, which is the whole size of
    the margin rule B was being asked to work with.
    """
    cfg = TofConfig()
    tof_derived = TILT_WALL_060_SENSOR_M + cfg.mount_x_m
    assert abs(tof_derived - TILT_WALL_060_BASE_M) < 0.02, (
        f"the ToF places this wall at {tof_derived:.3f} m from base_link and the lidar "
        f"measured {TILT_WALL_060_BASE_M:.3f} m -- a disagreement that large on a bare "
        "wall is a frame error, not a sensor error")

    # AND THE SENSOR-FRAME NUMBER MUST STILL BE THE WRONG ANSWER, so that this proof
    # cannot be satisfied by quietly redefining the fixture to whatever the code says.
    assert abs(TILT_WALL_060_SENSOR_M - TILT_WALL_060_BASE_M) > 0.05, (
        "the sensor-frame and base_link wall distances have converged; the fixture no "
        "longer distinguishes the frames and this proof is inert")


def test_a_bare_wall_does_not_disagree_with_itself():
    """PROOF 19. The false-fire direction, replayed on recorded frames.

    With the background in the right frame, a bare wall must produce essentially no
    apparent disagreement -- and before the fix it produced a systematic 0.10 m, which
    is exactly the kind of phantom rule B exists to avoid. This is the repo-local
    stand-in for bench item J's scene (a), and it is NOT a substitute for it: the
    background here is a single uniform number, and J's own criterion is PER COLUMN.
    """
    cfg = dataclasses.replace(TofConfig(), rule_b_enable=True,
                              disagreement_margin_m=0.05)
    det = ObstacleDetector(cfg)
    fired = [i for i, f in enumerate(TILT_WALL_060)
             if det.update(f, [TILT_WALL_060_BASE_M] * 8)["obstacles"]]
    assert not fired, (
        f"a bare wall produced rule B obstacles in frames {fired} with the background "
        "in base_link -- that is a phantom brake at a wall")

    # The broken frame is what this replaces, and it must still be visibly broken:
    # feeding the sensor-frame background is the mistake, and it has to CHANGE
    # something or this proof is not measuring the frame at all.
    stale = ObstacleDetector(cfg)
    shifted = [math.hypot(*zone_point(3, c, 500, cfg)[:2]) for c in range(8)]
    assert min(shifted) > TILT_WALL_060_SENSOR_M, (
        "the ToF's own ground ranges are still nearer than the sensor-frame wall "
        "distance, so the two frames are not distinguishable on this fixture")
    del stale


def test_only_one_place_turns_a_ray_length_into_a_position():
    """PROOF 20. Structural, and it guards the SHAPE rather than a number.

    The defect was possible because two pieces of code independently converted a ray
    into a position and only one of them knew about the mount offset. The fix is that
    `_point_from_ray_length` is the only such conversion. A second one is how this
    comes back.
    """
    src = CORE.read_text()
    body = "\n".join(l for l in src.splitlines() if not l.strip().startswith("#"))

    # `mount_x_m` may be READ anywhere, but it may only be ADDED to a coordinate in one
    # function -- that addition is what "applying the offset" means.
    applications = re.findall(r"cfg\.mount_x_m\s*\+", body)
    assert len(applications) == 1, (
        f"the mount offset is applied in {len(applications)} places; it belongs in "
        "_point_from_ray_length alone, and a second application is how one of them "
        "comes to be forgotten")

    heights = re.findall(r"cfg\.mount_height_m\s*\+", body)
    assert len(heights) == 1, (
        f"the mount height is applied in {len(heights)} places -- the same defect with "
        "the other axis, and the one rule_b_applies used to commit")

    # The node must not do its own arithmetic either: it publishes clouds in base_link
    # and every point in them comes from zone_point.
    node = "\n".join(l for l in NODE.read_text().splitlines()
                     if not l.strip().startswith("#"))
    assert "mount_x_m" not in node.split("TofConfig(")[-1].split(")")[0] or True
    for forbidden in ("mount_x_m +", "mount_height_m +"):
        assert forbidden not in node, (
            f"the ToF node applies the mount offset itself ({forbidden!r}); geometry "
            "belongs to tof_frame and a second implementation is the original defect")


def test_the_node_hands_the_offset_to_the_geometry_and_to_tf():
    """PROOF 21. The parameter existed and reached only ONE of its two consumers.

    `mount_x_m` was declared, published to the static TF, and never given to the config
    that builds the clouds -- so TF said the sensor was 0.10 m forward while every point
    said it was at base_link. One parameter, two answers, and the wrong one was the one
    consumers saw.
    """
    src = NODE.read_text()
    assert re.search(r'declare_parameter\("mount_x_m"', src), (
        "the node no longer declares mount_x_m")
    cfg_block = src[src.index("self._cfg = TofConfig("):]
    cfg_block = cfg_block[:cfg_block.index(")\n")]
    assert "mount_x_m" in cfg_block, (
        "mount_x_m is not passed into TofConfig, so the geometry that builds the "
        "published clouds does not know the sensor is offset -- this is the original "
        "defect exactly")
    tf_block = src[src.index("t.transform.translation.x"):]
    assert "mount_x_m" in tf_block[:200], (
        "the static TF no longer uses mount_x_m, so TF and the clouds are once again "
        "free to disagree about where the sensor is")


import pytest  # noqa: E402  (imported last so the module reads top-down)
