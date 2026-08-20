"""The recognition primitive's brain: prompt, strict parse, bearing, provenance.

Pure logic, no ROS — the node (`recognition_node`) adapts topics and the camera
lifecycle to these functions; everything decidable without hardware is decided
here and unit-tested on any machine. Design:
docs/design_recognition_primitive_2026-08-19.md (consensus 2026-08-19).

The contract this module enforces, because a search layer will stand on it:
a reply either parses into the EXACT schema or raises loudly with the reason —
an empty/near-miss answer silently treated as "not seen" is a searcher that
finds nothing and says nothing, the exact failure the design forbids.
"""

from __future__ import annotations

import math

from sphero_rvr_core.vlm_client import extract_json

#: Horizontal field of view. VENDOR SPEC for the Raspberry Pi Camera Module 3
#: (standard, not wide): 66 degrees. UNMEASURED ON THIS MOUNT — the bench cert
#: measures it (where_in_frame vs true bearing on staged objects) and this
#: constant gains the measured value + receipt then. Until that sitting,
#: bearing precision claims inherit the vendor number's uncertainty.
HORIZONTAL_FOV_DEG = 66.0

#: The frame is judged in thirds; a sector answer is honest about the camera's
#: actual granularity (a VLM asked for degrees invents them).
WHERE_VALUES = ("left", "center", "right")

#: Camera axis vs body axis, degrees LEFT (positive yaw). MEASURED on the
#: 2026-08-20 bench card (R4: 10° from the 9-mark walk + 4° lidar-measured
#: deliberate staging yaw), and confirmed live the same day (a body-centered
#: bottle reads sector RIGHT at −22°, exactly as this offset predicts).
CAMERA_MOUNT_OFFSET_DEG = 14.0

#: Sector bands in the CAMERA frame, degrees (left edge positive per REP-103):
#: thirds of the validated 66° HFOV.
_SECTOR_BANDS_DEG = {
    "left": (11.0, 33.0),
    "center": (-11.0, 11.0),
    "right": (-33.0, -11.0),
}


#: Floor-clutter cutoff for sighting ranges, metres. DERIVED from the range
#: sitting's own measurement (2026-08-20, placement 1, 114 in-band returns
#: over 6 clouds): the tof's floor-grazing clutter — present in EVERY sector,
#: always — ended at 0.54 m and the first standing object began at 1.09 m, so
#: 0.8 splits the gap with margin both ways. DELIBERATELY THE SAME NUMBER as
#: the search stanza's minimum confirm range ("never closer than about
#: 0.8 m"): one constant, one meaning — nearer than this the camera loses
#: floor objects AND the tof sees mostly floor. If the two ever need to
#: diverge, that is a design round, not a quiet edit.
SIGHTING_FLOOR_CLUTTER_M = 0.8

#: Cluster gap, metres: consecutive in-band returns further apart than this
#: belong to different standing objects. DERIVED same sitting: the observed
#: intra-cluster spreads were <=0.13 m and the inter-cluster gaps 0.55 and
#: 0.38 — 0.25 separates with margin both ways.
SIGHTING_CLUSTER_GAP_M = 0.25

#: Near-band floor, metres: in-band returns in [SIGHTING_NEAR_BAND_M,
#: SIGHTING_FLOOR_CLUTTER_M) are DISCARDED from the range but FLAG AMBIGUITY —
#: placement 2 (2026-08-20): a bottle at tape 0.742 returned 0.72-0.73, below
#: the cutoff, and the range confidently reported the BACKGROUND at 1.67
#: unambiguous — "confidently wrong about which object it measured", the worst
#: class an instrument hands a navigator. Endpoints derived: 0.6 sits above
#: the measured floor-clutter ceiling (0.54 in BOTH placements' distributions)
#: with margin, and below it only floor lives; the 0.8 top is the cutoff.
#: Residual documented: an object hiding INSIDE the clutter band (<0.6) still
#: escapes the flag — the stanza's never-confirm-below-0.8 rule is the
#: argument that a compliant searcher rarely meets one.
SIGHTING_NEAR_BAND_M = 0.6


def range_from_tof_points(points_xy, where_in_frame,
                          mount_offset_deg=CAMERA_MOUNT_OFFSET_DEG):
    """(range_m, ambiguous) for the sighting's sector, or None — never a guess.

    Consensus ruling C (2026-08-20, after placement 1 FAILED the ±0.15 bar):
    a plain median drowned the bottle (1.61 m, in-bar) under 60 floor-grazing
    returns (median 0.50). So: drop the floor-clutter band, CLUSTER what
    stands, return the NEAREST cluster's median — the safe error direction
    (an intervening object shortens the leg; the re-look recovers) — with
    `ambiguous=True` whenever other standing clusters exist, so the model
    treats the number as a minimum. No standing returns at all -> None.

    `points_xy` are (x, y) in the BASE frame. GEOMETRY: a point's camera-frame
    azimuth is its body azimuth MINUS the mount offset (the camera points 14°
    LEFT, so a point dead ahead of the CAMERA sits at body +14°); the sector
    bands are thirds of the measured 66° FOV in the camera frame.
    """
    if where_in_frame not in _SECTOR_BANDS_DEG:
        raise ValueError(f"where_in_frame {where_in_frame!r} has no sector band")
    lo, hi = _SECTOR_BANDS_DEG[where_in_frame]
    ranges, near_band = [], 0
    for x, y in points_xy:
        r = math.hypot(x, y)
        cam_az = math.degrees(math.atan2(y, x)) - float(mount_offset_deg)
        if not (lo <= cam_az <= hi):
            continue
        if r < SIGHTING_NEAR_BAND_M:
            continue                      # floor clutter: lives in every band
        if r < SIGHTING_FLOOR_CLUTTER_M:
            near_band += 1                # something STANDS too near to rank
            continue
        ranges.append(r)
    if not ranges:
        return None
    ranges.sort()
    clusters = [[ranges[0]]]
    for r in ranges[1:]:
        if r - clusters[-1][-1] > SIGHTING_CLUSTER_GAP_M:
            clusters.append([])
        clusters[-1].append(r)
    nearest = clusters[0]
    return nearest[len(nearest) // 2], (len(clusters) > 1 or near_band > 0)

#: Sector-center yaw offsets, radians. Image LEFT is the robot's LEFT, which is
#: POSITIVE yaw (REP-103). Thirds have centers at +/- FOV/3 and 0.
_WHERE_OFFSET_RAD = {
    "left": math.radians(HORIZONTAL_FOV_DEG) / 3.0,
    "center": 0.0,
    "right": -math.radians(HORIZONTAL_FOV_DEG) / 3.0,
}


#: The identity verdicts a match can carry. Order matters to no one; the WORDS
#: matter to the searcher: `unverified` is the approach-candidate answer (right
#: kind of object, identity not resolvable from here) and `mismatch` is the
#: pruning signal (visible evidence CONTRADICTS the identity). Design +
#: consensus: docs/design_recognition_schema_redesign_2026-08-20.md.
IDENTITY_VALUES = ("confirmed", "unverified", "mismatch")


def build_prompt(target: str) -> str:
    """Two verdicts about one target: object-match and identity, split so that
    honest doubt and blindness stop sharing a word (the 2026-08-20 bench card's
    core exhibit: every bottle frame was detected, but brand-doubt had nowhere
    to go except seen=false)."""
    target = str(target).strip()
    if not target:
        raise ValueError("empty target — the verb needs a thing to look for")
    return (
        f"You are the camera of a small ground robot. Look at this photo and "
        f"answer about ONE target: a {target}.\n"
        'Reply with ONLY a JSON object, exactly this shape:\n'
        '{"match": true/false, '
        '"identity": "confirmed"|"unverified"|"mismatch"|null, '
        '"where_in_frame": "left"|"center"|"right"|null, '
        '"confidence": 0.0-1.0, "description": "one short sentence"}\n'
        "Rules:\n"
        f"- match is true when an object of the target's basic kind is visible "
        f"(any bottle counts as a match when asked for a specific bottle). "
        f"Name the supporting evidence you see in description. Never report "
        f"match true for something you cannot describe.\n"
        "- identity, when match is true: 'confirmed' means the distinguishing "
        "features of the specific thing asked for are visible and match; "
        "'unverified' means right kind of object but you cannot verify the "
        "specific identity from this photo — this is the honest answer when "
        "unsure; 'mismatch' means visible evidence CONTRADICTS the identity — "
        "then name that contradicting evidence in description. If the target "
        "has no distinguishing identity beyond its kind, use 'confirmed'. "
        "When match is false, identity must be null.\n"
        "- where_in_frame must be null when match is false, and one of "
        "left/center/right when match is true.\n"
        "- confidence is YOUR confidence in the match answer.\n"
        "Do not mention anything except the answer object."
    )


def parse_recognition_reply(text: str) -> dict:
    """The reply, held to the schema — or a loud ValueError naming the breach.

    Returns exactly {match, identity, where_in_frame, confidence, description}
    and nothing else; extra fields are dropped, missing/invalid ones refuse.
    """
    data = extract_json(text)
    if data is None:
        raise ValueError(f"no JSON object in VLM reply: {text[:120]!r}")
    if not isinstance(data.get("match"), bool):
        raise ValueError(f"'match' missing or not a bool: {data.get('match')!r}")
    match = data["match"]
    identity = data.get("identity")
    where = data.get("where_in_frame")
    if match:
        if identity not in IDENTITY_VALUES:
            raise ValueError(
                f"match=true but identity is {identity!r} (must be one of "
                f"{IDENTITY_VALUES} — a match that won't say whether it is the "
                f"thing asked for is unusable)")
        if where not in WHERE_VALUES:
            raise ValueError(
                f"match=true but where_in_frame is {where!r} (must be one of "
                f"{WHERE_VALUES} — a sighting with no place is unusable)")
    else:
        if identity not in (None, "null"):
            raise ValueError(
                f"match=false but identity is {identity!r} (must be null — "
                f"an identity verdict with no object is a contradiction)")
        identity = None
        if where not in (None, "null"):
            raise ValueError(
                f"match=false but where_in_frame is {where!r} (must be null — "
                f"a place with no sighting is a contradiction)")
        where = None
    try:
        confidence = float(data["confidence"])
    except (KeyError, TypeError, ValueError):
        raise ValueError(f"'confidence' missing or not a number: "
                         f"{data.get('confidence')!r}")
    if not 0.0 <= confidence <= 1.0:
        raise ValueError(f"confidence {confidence} outside [0, 1]")
    description = data.get("description")
    if not isinstance(description, str) or not description.strip():
        raise ValueError("'description' missing or empty")
    return {"match": match, "identity": identity, "where_in_frame": where,
            "confidence": confidence, "description": description.strip()}


def bearing_from_frame_position(capture_yaw_rad: float, where_in_frame: str) -> float:
    """Map-frame bearing to the sighting: capture yaw + the sector's center
    offset. Honest to a third of the FOV — the bench cert measures how honest."""
    if where_in_frame not in _WHERE_OFFSET_RAD:
        raise ValueError(f"where_in_frame {where_in_frame!r} has no bearing")
    yaw = float(capture_yaw_rad) + _WHERE_OFFSET_RAD[where_in_frame]
    while yaw > math.pi:
        yaw -= 2.0 * math.pi
    while yaw < -math.pi:
        yaw += 2.0 * math.pi
    return yaw


#: FRAME QUALITY GATE (the warm-up fix, bench card R2's convicted mechanism):
#: every invocation cold-starts the camera, and frames captured before auto-
#: exposure/white-balance settle are near-black — the model then honestly
#: reports absence on an unusable frame (6 dark-frame opportunities on the
#: 2026-08-20 card produced 6 honest absences and 0 hallucinations; the
#: RECOGNIZER was exonerated, the CAPTURE was convicted). Constants MEASURED
#: from that card's own frames: dark class luma 9.4 (recognition_20260820_
#: 092438), bright class 97.2-100.9 (092132/092454/092520/092904) — the floor
#: 40 sits 4.3x above the measured dark and 2.4x below the measured bright.
#: The cast bound is stated INSURANCE: the card's global cast ratios were all
#: ~1.06-1.08 (the observed magenta was local, not a global mean shift), so
#: today's fixtures do not exercise it; it exists to catch a grossly unsettled
#: AWB frame that is bright enough to pass luma.
LUMA_FLOOR = 40.0
CAST_RATIO_MAX = 1.5


def frame_quality_ok(bgr) -> tuple:
    """(ok, reason) for one BGR frame: bright enough, and no gross global color
    cast. numpy-only (this module stays cv2-free)."""
    import numpy as np
    img = np.asarray(bgr, dtype=float)
    b, g, r = (float(img[:, :, i].mean()) for i in range(3))
    luma = 0.114 * b + 0.587 * g + 0.299 * r
    if luma < LUMA_FLOOR:
        return False, f"underexposed (luma {luma:.1f} < {LUMA_FLOOR:g} — warm-up)"
    ratio = max(b, g, r) / max(min(b, g, r), 1.0)
    if ratio > CAST_RATIO_MAX:
        return False, f"color cast (channel ratio {ratio:.2f} > {CAST_RATIO_MAX:g})"
    return True, ""


def sharpness(gray) -> float:
    """Focus score: variance of the discrete Laplacian, numpy-only (this module
    stays cv2-free). Higher = sharper. `gray` is a 2-D array."""
    import numpy as np
    g = np.asarray(gray, dtype=float)
    lap = (g[1:-1, :-2] + g[1:-1, 2:] + g[:-2, 1:-1] + g[2:, 1:-1]
           - 4.0 * g[1:-1, 1:-1])
    return float(lap.var())


def pick_sharpest(grays) -> int:
    """Index of the sharpest frame. Refuses an empty list loudly."""
    if not grays:
        raise ValueError("no frames captured — nothing to pick from")
    scores = [sharpness(g) for g in grays]
    return max(range(len(scores)), key=scores.__getitem__)


def build_result(*, target: str, parsed: dict, photo_path: str,
                 map_x: float, map_y: float, capture_yaw_rad: float,
                 stamp: str, model: str,
                 range_m=None, range_source=None, range_ambiguous=None) -> dict:
    """The verb's answer WITH PROVENANCE — photo, pose, bearing, time, model.

    BY CONSTRUCTION this function cannot leak a credential: its signature has
    no key argument and the result carries only what is listed here (a "found
    it" without provenance is D24's lying-telemetry class wearing a camera; a
    result carrying secrets is worse).
    """
    bearing = (bearing_from_frame_position(capture_yaw_rad, parsed["where_in_frame"])
               if parsed["match"] else None)
    return {
        "target": str(target),
        "match": parsed["match"],
        "identity": parsed["identity"],
        "where_in_frame": parsed["where_in_frame"],
        "confidence": parsed["confidence"],
        "description": parsed["description"],
        "photo_path": str(photo_path),
        "map_pose": {"x": round(float(map_x), 3), "y": round(float(map_y), 3),
                     "yaw_deg": round(math.degrees(float(capture_yaw_rad)), 1)},
        "bearing_deg": (None if bearing is None
                        else round(math.degrees(bearing), 1)),
        # Round 2 §1: a measured range turns the approach leg from a guess
        # into a placement. null is honest (no tof return in the sector) and
        # a match=false sighting carries no range by construction.
        "range_m": (None if (range_m is None or not parsed["match"])
                    else round(float(range_m), 3)),
        "range_source": (None if (range_m is None or not parsed["match"])
                         else str(range_source or "tof")),
        # ambiguous=True: other standing clusters share the sector — the range
        # is a MINIMUM, not the sighted object's certified distance.
        "range_ambiguous": (None if (range_m is None or not parsed["match"])
                            else bool(range_ambiguous)),
        "stamp": str(stamp),
        "model": str(model),
    }
