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

#: Sector-center yaw offsets, radians. Image LEFT is the robot's LEFT, which is
#: POSITIVE yaw (REP-103). Thirds have centers at +/- FOV/3 and 0.
_WHERE_OFFSET_RAD = {
    "left": math.radians(HORIZONTAL_FOV_DEG) / 3.0,
    "center": 0.0,
    "right": -math.radians(HORIZONTAL_FOV_DEG) / 3.0,
}


def build_prompt(target: str) -> str:
    """One question about one target, answer shape stated exactly."""
    target = str(target).strip()
    if not target:
        raise ValueError("empty target — the verb needs a thing to look for")
    return (
        f"You are the camera of a small ground robot. Look at this photo and "
        f"answer ONE question: is there a {target} visible?\n"
        'Reply with ONLY a JSON object, exactly this shape:\n'
        '{"seen": true/false, "where_in_frame": "left"|"center"|"right"|null, '
        '"confidence": 0.0-1.0, "description": "one short sentence"}\n'
        "Rules: where_in_frame must be null when seen is false, and one of "
        "left/center/right when seen is true. confidence is YOUR confidence in "
        "the seen answer. Do not mention anything except the answer object."
    )


def parse_recognition_reply(text: str) -> dict:
    """The reply, held to the schema — or a loud ValueError naming the breach.

    Returns exactly {seen, where_in_frame, confidence, description} and nothing
    else; extra fields are dropped, missing/invalid ones refuse.
    """
    data = extract_json(text)
    if data is None:
        raise ValueError(f"no JSON object in VLM reply: {text[:120]!r}")
    if not isinstance(data.get("seen"), bool):
        raise ValueError(f"'seen' missing or not a bool: {data.get('seen')!r}")
    seen = data["seen"]
    where = data.get("where_in_frame")
    if seen:
        if where not in WHERE_VALUES:
            raise ValueError(
                f"seen=true but where_in_frame is {where!r} (must be one of "
                f"{WHERE_VALUES} — a sighting with no place is unusable)")
    else:
        if where not in (None, "null"):
            raise ValueError(
                f"seen=false but where_in_frame is {where!r} (must be null — "
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
    return {"seen": seen, "where_in_frame": where,
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
                 stamp: str, model: str) -> dict:
    """The verb's answer WITH PROVENANCE — photo, pose, bearing, time, model.

    BY CONSTRUCTION this function cannot leak a credential: its signature has
    no key argument and the result carries only what is listed here (a "found
    it" without provenance is D24's lying-telemetry class wearing a camera; a
    result carrying secrets is worse).
    """
    bearing = (bearing_from_frame_position(capture_yaw_rad, parsed["where_in_frame"])
               if parsed["seen"] else None)
    return {
        "target": str(target),
        "seen": parsed["seen"],
        "where_in_frame": parsed["where_in_frame"],
        "confidence": parsed["confidence"],
        "description": parsed["description"],
        "photo_path": str(photo_path),
        "map_pose": {"x": round(float(map_x), 3), "y": round(float(map_y), 3),
                     "yaw_deg": round(math.degrees(float(capture_yaw_rad)), 1)},
        "bearing_deg": (None if bearing is None
                        else round(math.degrees(bearing), 1)),
        "stamp": str(stamp),
        "model": str(model),
    }
