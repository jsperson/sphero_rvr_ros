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
    """
    published = set(re.findall(r"(cam_[a-z_]+)=", NODE.read_text()))
    assert published == TELEMETRY, (
        f"the /collision_stop/state camera fields changed: expected {sorted(TELEMETRY)}, "
        f"found {sorted(published)}. If that was deliberate, update every consumer "
        "(run_recorder, gap_run_capture, swept_brake_ab, and three docs) in the same "
        "commit and say in the message what it costs the mount-move forensics")
