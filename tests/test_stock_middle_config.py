"""The stock-middle prototype config, checked offline against this robot's own facts.

NOT A CLAIM THAT THE PROTOTYPE WORKS. It has never been flown and the chassis was off
when it was written. These tests assert only that the config does not contain the
defects we already paid for -- which is the most a config can be held to without a
robot, and is exactly what "an exhibit beside the decision" should survive.

Everything asserted here traces to a recorded incident, not to taste.
"""

import re
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
STOCK = ROOT / "config" / "lean_nav2_stock.yaml"
FLOWN = ROOT / "config" / "collision_stop.yaml"

#: Measured on gauntlet mission 1: 41 consecutive commanded pure rotations at exactly
#: this rate produced 0-1 mm of motion. Breakaway is therefore ABOVE it.
KNOWN_INEFFECTIVE_RAD_S = 0.4
#: Chosen by the decisive controller to be above breakaway, per its own documentation.
KNOWN_EFFECTIVE_RAD_S = 0.9


def cfg():
    """The config's DIRECTIVES ONLY, with comment lines stripped.

    THIS HELPER IS THE POINT OF A LESSON LEARNED THREE TIMES IN ONE NIGHT. A guard
    that greps raw text fails on the file's own explanation of the defect it forbids
    -- the costmap-window 253 guard, the camera launch guard, and three of the guards
    below all did exactly that on first writing. The pressure such a guard creates is
    to DELETE THE EXPLANATION to get green, which destroys the most valuable comment
    in the file. Assert on what the machine reads, not on what the human wrote.
    """
    return "\n".join(line for line in STOCK.read_text().splitlines()
                      if not line.lstrip().startswith("#"))


def raw():
    """Full text INCLUDING comments -- for assertions that are about the prose."""
    return STOCK.read_text()


def scalar(text, name):
    m = re.search(rf"^\s*{name}:\s*([-\d.]+)", text, re.M)
    assert m, f"{name} not found in the stock config"
    return float(m.group(1))


# --- D45: no angular constant may sit in the dead band ------------------------------

@pytest.mark.parametrize("name", [
    "rotate_to_heading_angular_vel",
    "min_rotational_vel",
])
def test_no_angular_constant_is_below_the_known_ineffective_rate(name):
    """THE DEFECT THAT KILLED MISSION 1. The supervisor clamped pivots to 0.4 rad/s,
    the motors could not execute it, the robot did not move, and the freeze classifier
    blamed an obstacle that did not exist -- then planted a mark that buried the robot.

    Nav2's own Spin defaults `min_rotational_vel` to 0.4, so this is not a trap unique
    to us; it is the trap the ecosystem parameterised for bases exactly like this one.
    """
    value = scalar(cfg(), name)
    assert value > KNOWN_INEFFECTIVE_RAD_S, (
        f"{name}={value} is at or below the rate measured to produce NO MOTION on this "
        f"drivetrain (0.400 rad/s, 41 consecutive commands, 0-1 mm)")
    assert value >= KNOWN_EFFECTIVE_RAD_S, (
        f"{name}={value} is below the only rate documented as above breakaway (0.9)")


def test_the_rotation_shim_is_not_used():
    """The shim's purpose is to stop and rotate in place to face the path, and it is
    what ground the motors. RPP's own `use_rotate_to_heading` gives the same behaviour
    with ONE authority over the rotation rate instead of two."""
    text = cfg()
    assert "RotationShimController" not in text
    assert "use_rotate_to_heading: true" in text


def test_the_angular_constants_are_measured_and_cite_the_measurement():
    """This guard was written when every angular constant was provisional, and it
    demanded four MEASURE-FIRST markers. The sweep has since run
    (03_validation/breakaway_2026-08-16), so the invariant it protects has INVERTED: the
    angular constants must now be derived, and must say where from. A guard that still
    demanded 'MEASURE-FIRST' would be enforcing a state we deliberately left."""
    text = raw()

    assert "MEASURE-FIRST" not in text, (
        "an angular constant is still marked MEASURE-FIRST, but the measurement exists"
    )
    assert "breakaway_2026-08-16" in text, "the config must cite the run it derives from"
    assert "pivot_curve" in text, "and the module that owns the curve"


def test_no_angular_constant_asks_for_a_rate_the_drivetrain_cannot_produce():
    """THE WHOLE POINT. 0.4 was stock's old rotation rate and Nav2's Spin default; 0.9
    was the 'above breakaway' replacement. Measured: the rate curve jumps from exactly
    zero to ~0.8-1.5 rad/s, so NEITHER is producible by any duty, and the slowest clean
    in-place rotation is 3.55 rad/s at the deployed pivot_min_duty of 28.

    A config that asks for an unproducible rate is not merely mis-tuned -- it is asking
    for something the driver will silently substitute, which is how three layers came to
    hold opinions none of them executed."""
    import re

    from sphero_rvr_core import pivot_curve as pc

    floor = pc.minimum_clean_rate(28)
    text = raw()

    rotational_keys = (
        "rotate_to_heading_angular_vel",
        "min_rotational_vel",
        "max_rotational_vel",
    )
    found = {}
    for key in rotational_keys:
        match = re.search(rf"^\s*{key}:\s*([0-9.]+)", text, re.M)
        assert match, f"{key} is missing from the stock config"
        found[key] = float(match.group(1))

    for key, value in found.items():
        assert value >= floor, (
            f"{key} = {value} is below the slowest clean pivot this drivetrain can make "
            f"({floor:.2f} rad/s). The driver would raise it silently; the config would "
            "be fiction."
        )
    assert found["max_rotational_vel"] <= pc.maximum_clean_rate(45) + 1e-6, (
        "max_rotational_vel exceeds what the deployed duty band can deliver"
    )


def test_constants_the_curve_does_not_cover_are_marked_UNMEASURED():
    """The curve measured IN-PLACE PIVOTS. Regimes it does not cover must be labelled,
    not quietly inherited.

    This guard used to COUNT the word UNMEASURED and require three. That broke the moment
    acceleration became genuinely derived (it is now fixed by the floor/dt arithmetic),
    which is the guard punishing us for doing the right thing. It now names the regimes
    that are actually still unmeasured, so deriving one correctly removes it from the list
    instead of failing the build.
    """
    text = raw()

    assert "UNMEASURED" in text
    # Linear breakaway: both treads driving the same way is a different regime, and the
    # pivot path turned out to have a hard dead zone -- assuming linear has none would be
    # the same mistake in a new costume.
    assert "regulated_linear_scaling_min_speed" in text
    assert "STILL UNMEASURED" in text, "linear breakaway must stay labelled"
    # Arc rates, with a close path rather than just a label.
    assert "run_card_arc_rate_FUTURE" in text


# --- D42: marks must be points, and the lidar must not erase them -------------------

def _costmap(text, which):
    """Slice one costmap's whole block out of the config text.

    ADDED when the global costmap arrived, because `_layer` below silently assumed there
    was only ever one costmap: with `touch_layer:` appearing in both, its first-index
    slicing spanned from the GLOBAL touch layer all the way into the LOCAL one, and three
    guard tests failed against a config that was correct. A helper that finds "the" layer
    is fine until there are two of everything -- and the whole point of these tests is to
    guard the layer whose rationale is being asserted, so the scope has to be explicit.
    """
    starts = {"global": "\nglobal_costmap:", "local": "\nlocal_costmap:"}
    begin = text.index(starts[which])
    rest = text[begin + 1:]
    # The next top-level key ends the block: a line starting in column zero.
    for marker in ("\nplanner_server:", "\ncontroller_server:", "\nbehavior_server:",
                   "\nbt_navigator:", "\nlifecycle_manager_explore:", "\nlocal_costmap:",
                   "\nglobal_costmap:"):
        if marker[1:] == starts[which][1:]:
            continue
        idx = rest.find(marker)
        if idx > 0:
            rest = rest[:idx]
    return rest


def _layer(text, name, which="local"):
    """Slice one layer block out of ONE costmap, whatever order the layers sit in."""
    block = _costmap(text, which)
    order = ["scan_layer:", "touch_layer:", "tof_layer:", "inflation_layer:"]
    present = [o for o in order if o in block]
    i = present.index(name + ":")
    start = block.index(present[i])
    end = block.index(present[i + 1]) if i + 1 < len(present) else len(block)
    return block[start:end]


def test_touch_marks_are_cleared_by_no_sensor_at_all():
    """RAYTRACE CLEARING IS 2D PER LAYER, and the touch layer grants that authority to
    NOBODY.

    The lidar is excluded because a ray at the 0.19 m scan plane passes straight over a
    chair leg and would erase the one obstacle class this robot cannot see.

    The ToF was excluded on 2026-08-18, reversing the earlier design: IT CANNOT BE
    TRUSTED TO CLEAR WHAT IT CANNOT RELIABLY SEE. Measured ~15% fill on thin targets
    (08-13 bench, 5 cm rail; 15.6% of frames on the leg that stopped the 08-18 retest)
    means raytrace-through would erase a real mark within seconds of planting it.
    Observation authority and clearing authority have different reliability
    requirements, and the measured envelope meets only the first."""
    text = cfg()
    touch = _layer(text, "touch_layer")
    assert "clearing: false" in touch, "touch marks must be cleared by no sensor"
    assert "clearing: true" not in touch, "no source in this layer may clear"
    assert "/scan" not in touch, "the lidar has no observation role here"
    assert "/tof/points" not in touch, (
        "the ToF lost clearing authority over touch marks on 2026-08-18 -- if it is "
        "back in this layer, that reversal has been undone")


def test_footprint_clearing_is_the_touch_layer_escape_hatch():
    """D43 was a robot buried by its own marks. With no sensor clearing, footprint
    clearing is the ONLY way a cell under the robot returns to free -- so the robot can
    never be permanently trapped by its own planting.

    It is a second, independent defence rather than the primary one: freeze_mark_pose
    places marks at the LEADING EDGE (footprint_front + margin), so a mark is born
    outside the footprint and this path should never be needed."""
    touch = _layer(cfg(), "touch_layer")
    assert re.search(r"footprint_clearing_enabled:\s*true", touch)


def test_the_touch_range_gate_is_not_a_sensor_range():
    """MEASURED 2026-08-18, and it was silently discarding marks in the field.

    `obstacle_max_range` is measured from the OBSERVATION ORIGIN -- the origin of the
    cloud's own frame in the global frame. Every other source here publishes in a sensor
    frame, so that origin is the sensor and the gate means what its name says.
    `/contact_marks` is not a sensor: contact_marker publishes absolute-frame beliefs
    stamped in `map`, so the origin is the MAP ORIGIN and the gate reads "within N metres
    of wherever the rover started".

    Held robot-relative geometry constant and varied only distance from the map origin:
    0.5/1.0/1.5/1.9 m all marked at 253; 2.1/2.5/3.0 m did not mark at all. A clean
    cutoff at the configured 2.0.

    The guard is deliberately loose about the exact value and strict about the CLASS: any
    sensor-scale number here reintroduces a touch port that works in the first room and
    nowhere else."""
    touch = _layer(cfg(), "touch_layer")
    m = re.search(r"obstacle_max_range:\s*([0-9.]+)", touch)
    assert m, "the touch source must state its range gate rather than inherit a default"
    assert float(m.group(1)) >= 10.0, (
        f"obstacle_max_range={m.group(1)} on /contact_marks is a SENSOR range applied to "
        "a map-frame belief -- it gates on distance from the START POSE, not from the "
        "robot, and silently drops every mark planted beyond it")


def test_the_tof_has_its_own_layer_and_may_clear_its_own_returns():
    """The split (2026-08-18) exists because one shared layer forced ONE clearing policy
    onto two sources with different trust profiles. Removing ToF clearing to protect
    touch marks would also have made every ToF mark -- including phantoms, since D27's
    class is parked rather than extinct -- mission-permanent. Separated, each rationale
    lives where it is true: the ToF is self-consistent, touch marks are erased by
    nothing."""
    text = cfg()
    tof = _layer(text, "tof_layer")
    assert "/tof/points" in tof
    assert "clearing: true" in tof, "the ToF may erase its OWN returns"
    assert "/scan" not in tof and "/contact_marks" not in tof, (
        "the ToF layer clears only what the ToF itself marks")
    # Parsed, not grepped: a text search for `plugins:` finds the PLANNER's
    # ["GridBased"] first and passes an assertion about the costmap on the wrong list.
    # (It did, on the first run of this very test.)
    local = yaml.safe_load(text)["local_costmap"]["local_costmap"]["ros__parameters"]
    assert "touch_layer" in local["plugins"] and "tof_layer" in local["plugins"], (
        "a layer absent from the plugins list is configuration that never loads")


def test_the_tof_clears_only_within_its_honest_envelope():
    """A sensor may only clear where it can see. The ToF's structural blind band ends
    at 0.167 m (blind_band_outer_range_m) and rule B reaches ~0.60 m; clearing outside
    that would be clearing on evidence it cannot supply -- the D39 lesson applied to
    the costmap instead of the brake."""
    tof = _layer(cfg(), "tof_layer")
    assert re.search(r"raytrace_min_range:\s*0\.17", tof)
    assert re.search(r"raytrace_max_range:\s*0\.6", tof)


def test_the_planner_can_see_live_lidar():
    """THE BIGGER HALF OF THE GLOBAL COSTMAP CHANGE, and the one most likely to be
    silently undone.

    Before 2026-08-18 this config had no `global_costmap` section, so planner_server came
    up on nav2's defaults: an obstacle layer subscribed to NOTHING, logged as
    `Subscribed to Topics:` with an empty list. The flown `lean_nav2.yaml` has the same
    shape. So the planner has never in this project's history seen live lidar -- it
    planned against the SLAM map and was blind to every change since the map was drawn.

    Deleting this section does not fail loudly; nav2 just quietly reverts to defaults and
    plans blind again. Hence a test."""
    scan = _layer(cfg(), "scan_layer", "global")
    assert "/scan" in scan, "the planner is blind to live lidar again"
    assert re.search(r"marking:\s*true", scan)
    assert re.search(r"clearing:\s*true", scan), (
        "a global costmap that only accumulates is D42 at map scale -- the planner must "
        "be able to forget an obstacle that moved")


def test_the_planner_can_see_contact_marks():
    """Measured 2026-08-18: with marks in the LOCAL costmap only, a contact mark 0.204 m
    wide with open floor on both sides stopped the robot 0.151 m short and then ABORTED
    the goal -- RPP refused the approach while the planner, blind to the mark, replanned
    the identical straight path every cycle. Protection without progress."""
    touch = _layer(cfg(), "touch_layer", "global")
    assert "/contact_marks" in touch
    assert re.search(r"clearing:\s*false", touch), (
        "the lidar sees straight THROUGH the obstacle class this layer exists for")


def test_the_global_touch_range_gate_is_not_a_sensor_range():
    """Same defect as the local layer and STRICTLY WORSE here: `obstacle_max_range` is
    measured from the observation origin, which for a `map`-frame belief cloud is the map
    origin -- so a sensor-scale value gates on distance from the rover's START POSE. A
    global costmap exists precisely to hold marks far from the start pose."""
    touch = _layer(cfg(), "touch_layer", "global")
    m = re.search(r"obstacle_max_range:\s*([0-9.]+)", touch)
    assert m and float(m.group(1)) >= 10.0, (
        "a sensor-scale range gate on a map-frame belief silently drops every mark "
        "planted beyond it from the start pose")


def test_the_least_trusted_source_stays_out_of_the_global_costmap():
    """The 2026-08-08 camera precedent, applied consistently: the global costmap is where
    a bad mark does the most damage (it can close a doorway to the planner outright), so
    the ToF -- short range, and D27's phantom class parked rather than extinct -- is not
    given a vote on global geometry. It keeps its local layer."""
    glob = _costmap(cfg(), "global")
    assert "tof_layer" not in glob and "/tof/points" not in glob
    assert "tof_layer" in _costmap(cfg(), "local"), (
        "the ToF must keep its LOCAL layer -- this test guards its scope, not its life")


def test_both_costmaps_declare_the_footprint_padding():
    """M1: nav2 pads the footprint by `footprint_padding` and derives the inscribed
    radius -- and the polygon footprint clearing erases from -- out of the PADDED shape.
    An undeclared default participating in safety geometry is the defect; it has to be
    visible in BOTH costmaps or the next derivation reads 0.145 again."""
    for which in ("local", "global"):
        block = _costmap(cfg(), which)
        assert re.search(r"footprint_padding:\s*0\.01", block), (
            f"{which}_costmap does not declare footprint_padding")


def test_no_denoise_layer():
    """DenoiseLayer removes small obstacle groups. A touch mark IS a small obstacle
    group."""
    assert "DenoiseLayer" not in cfg()


def test_inflation_is_the_only_place_the_robot_radius_is_applied():
    """D42's double-booking: we painted marks as 0.14 m robot-radius discs and then let
    the costmap inflate them AGAIN, sterilising ~0.56 m per touch. Marks are points
    now; the robot's extent enters once.

    THE LITERAL MOVED ON 2026-08-31, THE OBLIGATION DID NOT. This asserted
    `robot_radius: 0.14` because that was where the robot's size was declared. The
    costmaps now declare the measured POLYGON instead, so the assertion follows the
    declaration rather than the string -- what is being guarded is that the extent is
    stated in exactly one place per costmap and inflation applies it once, not that a
    particular key exists. `tests/test_footprint_derivation.py` pins the polygon's
    VALUES against the supervisor's extents; this pins its SINGULARITY.
    """
    text = cfg()
    assert re.search(r"inflation_radius:\s*[0-9.]+", text)
    assert "robot_radius:" not in text, (
        "a costmap declares both a footprint and a robot_radius -- nav2 takes the "
        "footprint, so the radius is a silent lie rather than an error")
    declarations = re.findall(r"^\s*footprint:", text, re.M)
    assert len(declarations) == 2, (
        f"the robot's extent must be declared once per costmap, found "
        f"{len(declarations)} declarations (a substring match here would also count "
        f"footprint_padding and published_footprint -- anchor on the key)")


def test_inflation_leaves_a_gradient_the_planner_can_use():
    """THE 8 MM DEFECT, guarded as a relationship rather than as a literal.

    This test used to pin `inflation_radius: 0.16`. Measured 2026-08-18, that value sat
    ON the deployed circumscribed radius (0.1591), so the cost profile went 253 out to
    the inscribed 0.1519 and reached zero 8 mm later. An inflation radius equal to the
    circumscribed radius means NO GRADIENT EXISTS, and a planner with no gradient cannot
    prefer clearance: SmacPlanner2D hugged the boundary at 0.161 m and RPP -- which
    checks the FOOTPRINT, 0.1591 -- refused the path the planner had just produced.

    So the invariant is not a number, it is a margin: inflation must reach meaningfully
    beyond the circumscribed radius, or the planner and the controller disagree about
    what is drivable. Pinning the literal is what let the defect sit here unnoticed while
    a test claimed to be guarding it.

    AND THEN IT HAPPENED AGAIN, ONE LEVEL UP (D76, 2026-08-31). The relationship was
    right; the radius fed into it was a TRANSCRIBED circle-era constant. When the costmaps
    switched to a polygon, this guard went on measuring the gradient against a
    circumscribed radius the costmap had stopped building -- 0.1591 where the padded
    polygon reaches 0.1702 -- and stayed green throughout, because nothing compared its
    input to the declaration. The margin survives the correction (141 mm -> 130 mm against
    a 100 mm bar), so no verdict flips; the lesson does. It now derives the radius from the
    deployed footprint, so the input cannot go stale without the declaration changing."""
    from tests.test_footprint_derivation import costmap_radii
    _, circumscribed = costmap_radii()
    value = scalar(cfg(), "inflation_radius")
    margin = value - circumscribed
    assert margin >= 0.10, (
        f"inflation_radius={value} leaves only {margin * 1000:.0f} mm beyond the "
        f"circumscribed radius {circumscribed:.4f}. The planner needs a "
        f"gradient to prefer clearance; without one it plans paths the controller "
        f"refuses, which is exactly what 0.16 did.")


# --- D36 / D40: the recoveries must actually be able to run -------------------------

def test_a_local_costmap_exists_for_the_recoveries_to_check_against():
    """D36 measured stock recoveries refusing in 2 ms. They collision-check against the
    local costmap, and the flown stack has none -- explore.launch.py drops
    controller_server "and with it Nav2's local costmap". The recoveries were present,
    wired, and structurally unable to succeed."""
    text = cfg()
    assert "local_costmap:" in text
    behavior = text[text.index("behavior_server:"):]
    assert "local_costmap_topic" in behavior


def test_progress_checker_asks_about_translation_not_rotation():
    """PoseProgressChecker credits in-place rotation as progress. On mission 1 the rover
    rotated 1.8-2.2 degrees over 2-3 s while going nowhere; that reads as progress, and
    nothing trips for the whole time allowance. The honest question for a robot that is
    supposed to be going somewhere is whether it translated."""
    text = cfg()
    assert "SimpleProgressChecker" in text
    assert "PoseProgressChecker" not in text


def test_yaw_tolerance_is_not_tightened_below_one_control_cycle():
    """Bounded from BELOW, which surprises people. The tightest achievable yaw tolerance
    is about one control cycle of rotation (pivot_rate / controller_frequency).
    Tightening past it makes the goal unreachable: overshoot, correct, overshoot --
    which on this drivetrain is grinding."""
    text = cfg()
    yaw = scalar(text, "yaw_goal_tolerance")
    rate = scalar(text, "rotate_to_heading_angular_vel")
    freq = scalar(text, "controller_frequency")
    assert yaw >= rate / freq, (
        f"yaw_goal_tolerance {yaw} is tighter than one control cycle of rotation "
        f"({rate}/{freq} = {rate/freq:.3f} rad)")
    assert yaw >= 0.25


# --- the seam that will kill this prototype if it is forgotten ----------------------

def test_the_reverse_seam_is_documented_as_an_open_risk():
    """D40 WEARING STOCK CLOTHES, and the prototype's most likely failure.

    Nav2's BackUp recovery commands reverse. Our collision supervisor holds reverse
    whenever the rear sector is inside `reverse_stop_distance_m` (0.25) -- and mission 1
    sat at rear 0.243 m, seven millimetres inside it, refusing reverse 61 times in one
    minute. If the supervisor is not taught to permit BackUp, every stock reverse
    recovery dies exactly the way our bespoke ones did, and we will conclude stock is no
    better.

    This is asserted against the RECKONING DOC rather than the config, because the fix
    is in the supervisor and is NOT part of this prototype. It must not be discovered
    on carpet.
    """
    doc = (ROOT / "docs" / "navigation_reckoning.md").read_text()
    assert "BackUp" in doc and "reverse_stop_distance_m" in doc, (
        "the reverse-seam risk must be written down before this prototype is flown")


# --- the config must be able to START ------------------------------------------------

def test_the_lifecycle_manager_section_exists_and_manages_the_controller():
    """A config file is a claim about a system that runs.

    2026-08-17, during 3a pre-flight: this config had no `lifecycle_manager_explore`
    section, so `node_names` was uninitialized and nav2_lifecycle_manager THREW at
    startup (ParameterUninitializedException, exit -6). The four servers then sat in
    `unconfigured` forever -- present in `ros2 node list`, doing nothing. It had never
    been flown, so nothing had ever needed the manager to exist.
    """
    import yaml

    parsed = yaml.safe_load(raw())
    assert "lifecycle_manager_explore" in parsed, (
        "no lifecycle manager section: the servers will never leave `unconfigured`"
    )
    params = parsed["lifecycle_manager_explore"]["ros__parameters"]
    assert params.get("autostart") is True
    names = params.get("node_names")
    assert names, "node_names must be set, or the manager throws and dies at startup"
    assert "controller_server" in names, (
        "controller_server MUST be managed here -- it is the whole point of the stock "
        "middle, and the decisive path omits it precisely because it does not start it"
    )
    for required in ("planner_server", "behavior_server", "bt_navigator"):
        assert required in names


def test_the_angular_acceleration_limit_clears_the_floor_in_ONE_control_cycle():
    """The fix's arithmetic, pinned so it cannot silently regress.

    RPP's rotate command is clamped to an acceleration ramp from the measured speed. If
    accel * dt is below the slowest rate this drivetrain can produce, the FIRST command
    from rest is already sub-floor -- and every sub-floor command is raised to the floor
    duty and executed at full rate, which is the reversal limit cycle's coupling.
    """
    import re

    import yaml

    from sphero_rvr_core import pivot_curve as pc

    parsed = yaml.safe_load(raw())
    freq = parsed["controller_server"]["ros__parameters"]["controller_frequency"]
    accel = parsed["controller_server"]["ros__parameters"]["FollowPath"]["max_angular_accel"]
    spin = parsed["behavior_server"]["ros__parameters"]["rotational_acc_lim"]
    floor = pc.minimum_clean_rate(28)

    assert accel / freq >= floor, (
        f"max_angular_accel {accel} at {freq} Hz gives a first command of "
        f"{accel / freq:.3f} rad/s, below the {floor:.2f} rad/s floor this drivetrain "
        "can actually produce"
    )
    assert spin / freq >= floor, "behavior_server's Spin has the identical defect"


# --- the no-section family, closed as a CLASS (2026-08-18 night shift) ---------------

#: Run 3c under load ~8: controller_server's loop sagged from 20 Hz to 8.2 Hz, so one
#: starved cycle is ~122 ms and goal-acknowledge delays run to several hundred ms
#: exactly when the system is least healthy. The BT's ack budget must cover several
#: such cycles, or an accepted-but-unacknowledged goal becomes an ownerless drive --
#: which is not a hypothesis: goal 3's orphaned follow_path drove 0.135 m and pivoted
#: 82 degrees AFTER bt_navigator aborted the mission.
MEASURED_STARVED_CONTROLLER_PERIOD_S = 1.0 / 8.2


def test_bt_navigators_ack_budget_covers_the_measured_starvation():
    """No-section member three: bt_navigator ran nav2's default_server_timeout of
    20 ms because it had no section at all. The guard asserts the DERIVATION, not
    just presence: the budget must cover at least four starved controller cycles,
    because the field's ack delay arrived in exactly that regime."""
    import yaml

    parsed = yaml.safe_load(raw())
    assert "bt_navigator" in parsed, (
        "bt_navigator has no config section -- nav2 defaults include the 20 ms ack "
        "budget that manufactured the 2026-08-18 ownerless drive"
    )
    timeout_ms = parsed["bt_navigator"]["ros__parameters"]["default_server_timeout"]
    floor_ms = 4 * MEASURED_STARVED_CONTROLLER_PERIOD_S * 1000.0
    assert timeout_ms >= floor_ms, (
        f"default_server_timeout {timeout_ms} ms is under {floor_ms:.0f} ms -- four "
        f"starved controller cycles at the measured 8.2 Hz -- so a load spike can "
        f"again abort a goal the controller then runs with no owner"
    )


def test_every_nav2_node_the_stock_launch_starts_has_an_explicit_section():
    """The class guard, so member FOUR fails a test instead of a flight.

    planner_server (blind planner), global_costmap (same hole), bt_navigator (the
    ownerless drive) each shipped as a MISSING SECTION running silent nav2 defaults.
    This walks the launch file's actual nav2 nodes and requires an explicit top-level
    section for every one -- defaults-by-decision: a node that genuinely wants pure
    defaults gets an empty section saying so, never an omission."""
    import re

    import yaml

    launch_text = (ROOT / "launch" / "explore.launch.py").read_text()
    launched = set()
    for block in launch_text.split("Node(")[1:]:
        pkg = re.search(r'package="([^"]+)"', block)
        name = re.search(r'name="([^"]+)"', block)
        if pkg and name and pkg.group(1).startswith("nav2_"):
            launched.add(name.group(1))
    # The costmaps ride inside planner/controller but configure as their own
    # top-level sections; hold them to the same rule explicitly.
    launched |= {"global_costmap", "local_costmap"}
    assert launched >= {"planner_server", "controller_server", "bt_navigator",
                        "behavior_server"}, (
        f"launch parse broke -- found only {sorted(launched)}; fix the parser, do "
        f"not weaken the guard"
    )

    parsed = yaml.safe_load(raw())
    sections = set(parsed)
    missing = sorted(launched - sections)
    assert not missing, (
        f"nav2 node(s) launched with NO config section: {missing}. Every silent "
        f"default is a decision nobody made -- add a section, even an empty one "
        f"with the reason, per the no-section family's three receipts"
    )


def test_the_touch_ports_producer_is_launched_not_remembered():
    """The never-launched-node family's tripwire: both costmaps above subscribe
    /contact_marks, and on 2026-08-18 the node that publishes it appeared in NO
    launch file -- it flew twice as an operator memory item. The launch must
    reference the contact_marker executable AND declare its start argument."""
    launch_text = (ROOT / "launch" / "explore.launch.py").read_text()
    assert 'executable="contact_marker"' in launch_text, (
        "explore.launch.py no longer starts contact_marker -- a flight without it "
        "has no touch response at all (contacts plant no marks)"
    )
    assert "start_contact_marker" in launch_text
