"""The supervisor's low-obstacle layer must be named for its ROLE, not its occupant.

That layer was built for the camera, is being handed to the ToF, and will outlive both.
Naming it after whichever sensor currently feeds it guarantees the names lie again at
the next handover -- this would have been the second time in one transition.

It is the same defect class as the recorder scraping `rule_i_zones=` while the node
published `rule_a_zones=`, one layer down: a name that describes what something USED to
be, believed by a reader who has no way to check.
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NODE = ROOT / "src" / "sphero_rvr_driver" / "collision_stop_node.py"
CONFIG = ROOT / "config" / "collision_stop.yaml"

#: Field names on `/collision_stop/state`. DELIBERATELY still `cam_*` and NOT part of
#: this rename -- see `test_the_telemetry_field_names_are_a_known_deferral`.
TELEMETRY = {"cam_nearest", "cam_scale", "cam_cloud_age", "cam_output_linear"}

#: Fields ADDED since the deferral was recorded. Additions are a different act from
#: renames and are held to a different bar -- see the test below.
TELEMETRY_ADDITIONS = {"cam_considered", "cam_hold_active", "cam_hold_reason"}
"""`cam_hold_active` / `cam_hold_reason` added 2026-08-15 with the D39 hold.

Declared here rather than renaming anything, but note what they exist to disambiguate:
`cam_nearest` keeps its meaning -- the value the brake ACTED on -- and that value can
now be a HELD BELIEF rather than a live sighting. Its distribution is therefore not a
pure sighting distribution across this commit, and the owed longitudinal comparison
must filter on `cam_hold_active=false` to compare like with like. That is exactly the
population trap this file exists to keep visible, arriving this time through a change
in what a column MEANS rather than in what it is called.
"""


def test_no_camera_named_parameter_survives_in_the_supervisor():
    """A HALF-RENAME IS WORSE THAN NONE. Half the parameters describing the sensor and
    half describing the role leaves a reader unable to tell which convention any given
    name follows, so every one of them has to be traced."""
    params = set(re.findall(r'"(camera_[a-z_]+)"', NODE.read_text()))
    assert not params, (
        f"the supervisor still declares sensor-named parameters {sorted(params)}; the "
        "layer is generically a low-obstacle cloud layer and the names must say so")

    cfg = set(re.findall(r"^\s*(camera_[a-z_]+):", CONFIG.read_text(), re.M))
    assert not cfg, (
        f"config/collision_stop.yaml still sets {sorted(cfg)} -- parameters renamed in "
        "code but not in the deployed config are parameters that silently take their "
        "defaults, which is the worst of both")


def test_no_camera_named_internal_state_survives():
    attrs = set(re.findall(r"(_cam_[a-z_]+)", NODE.read_text()))
    assert not attrs, f"sensor-named internals remain: {sorted(attrs)}"
    for method in ("_apply_camera_brake", "_camera_blocks_pivot", "_on_camera_cloud"):
        assert method not in NODE.read_text(), f"{method} still names the sensor"


def test_the_telemetry_field_names_are_a_known_deferral():
    """`cam_nearest` and friends on /collision_stop/state are NOT renamed, on purpose.

    They are a published interface with six consumers -- the recorder, two diagnostics,
    three docs -- and, decisively, they are COLUMN NAMES in every mission CSV ever
    recorded. Renaming them now breaks longitudinal comparison against those runs, and
    one such comparison is currently owed: `cam_nearest`'s distribution against an
    earlier mission's is what will timestamp when the camera mount moved.

    So this asserts the deferral is INTACT rather than that it is correct. If the fields
    are ever renamed, this test is the place that records what it costs.

    RENAMING AND ADDING ARE DIFFERENT ACTS, and this test used to treat them as one.
    A rename breaks every consumer and silently ends the longitudinal comparison above,
    because old CSVs keep the old column name and nothing says the two are the same
    quantity. An APPENDED `key=value` field breaks no parser -- all of them look fields
    up by name -- and costs the forensics nothing, since old recordings simply lack the
    column. Conflating the two would have meant either blocking a cheap addition or, far
    worse, being tempted to relax the check that guards the expensive one.

    So: the four deferred names must ALL still be published, and any additional
    `cam_*` field must be declared in `TELEMETRY_ADDITIONS` -- which keeps an addition
    deliberate and reviewed without letting it look like a rename. (`cam_considered`
    was added 2026-08-15 by autopsy #2: nothing reported how many points the brake
    actually considered after its own filters, so `/tof/state`'s whole-reach zone count
    stood in as a proxy and produced a confident wrong diagnosis for two sessions.)
    """
    published = set(re.findall(r"(cam_[a-z_]+)=", NODE.read_text()))

    missing = TELEMETRY - published
    assert not missing, (
        f"a DEFERRED /collision_stop/state field disappeared: {sorted(missing)}. These "
        "are column names in every mission CSV ever recorded, and one comparison is "
        "still owed against them -- `cam_nearest`'s distribution is what will timestamp "
        "when the camera mount moved. Renaming or removing one ends that silently.")

    undeclared = published - TELEMETRY - TELEMETRY_ADDITIONS
    assert not undeclared, (
        f"new /collision_stop/state camera fields {sorted(undeclared)} are published but "
        "not declared. Adding one is fine and cheap -- consumers look fields up by name "
        "-- but declare it in TELEMETRY_ADDITIONS and update the consumers that should "
        "USE it (run_recorder, gap_run_capture, swept_brake_ab, and the telemetry row "
        "in docs/tof_navigation_design.md) in the same commit.")
