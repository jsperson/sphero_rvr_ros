"""SCENARIO 5 of Scott's nine, 2026-08-31.

  REQUIREMENT, his words: "Lidar and ToF should prevent collisions on things they
  can see."

  BAR, PRE-REGISTERED: two obstacles, one per sensor -- a 40 cm box (the lidar
  sees it, the ToF's rules correctly stay quiet) and a 6 cm box (the lidar is blind,
  the ToF catches it). Contacts = 0 for BOTH.

  PREDICTION FILED BEFORE EXECUTION: PARTIAL -- lidar PASS (its stop tier fired in
  the field on 2026-08-25 at 0.290 m from base_link), ToF UNKNOWN.

The row is kept split on purpose. A single verdict here would hide which sensor is
untested, and "the safety stack works" is exactly the sentence that should never be
carried by one number.

WHAT THIS COMPOSES: the REAL `CollisionStopSupervisor` -- the same pure-core
arbiter the node wraps -- driven with scans raycast from `tof_sim` against the same
world the ToF half uses. Config is loaded from the DEPLOYED `collision_stop.yaml`,
so a bar tested here is tested against the robot's numbers.

WHAT IT DOES NOT TEST: the laser mount. Scans are synthesised directly in ROBOT
bearings with an identity laser->base transform, because the ~179 deg mount is
already certified elsewhere (`tof_frame`, the deployed static TF) and re-deriving
it here would put a second author on geometry this project has been burned by. So
this scenario tests the BRAKE's decision, not the frame maths beneath it -- and, as
with scenario 1, not the ROS plumbing.
"""

from __future__ import annotations

import math
import os

import pytest
import yaml

from sphero_rvr_driver.collision_stop import (CollisionStopConfig,
                                              CollisionStopSupervisor, ScanInput,
                                              Transform2D, TwistCommand)
from sphero_rvr_core.tof_frame import TofConfig
from sphero_rvr_core.tof_sim import Box, World, simulate_scan

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEPLOYED = yaml.safe_load(
    open(os.path.join(REPO, "config", "collision_stop.yaml"))
)["lidar_collision_stop_supervisor"]["ros__parameters"]

TOF_CFG = TofConfig()
DT = 0.1
BEARING_STEP_DEG = 1.0
BEARINGS = [math.radians(d) for d in
            [i * BEARING_STEP_DEG - 180.0 for i in range(int(360 / BEARING_STEP_DEG))]]
RANGE_MAX = float(DEPLOYED["max_range_m"])

TALL_BOX = Box(x_min=1.00, x_max=1.10, y_min=-0.15, y_max=0.15, z_min=0.0, z_max=0.40)


def _config():
    """The supervisor's own config, from the deployed YAML -- only the fields it
    declares, so an unknown key here would be a real disagreement rather than noise."""
    fields = {f.name for f in CollisionStopConfig.__dataclass_fields__.values()}
    return CollisionStopConfig(**{k: v for k, v in DEPLOYED.items() if k in fields})


def _scan(world, now):
    return ScanInput(
        ranges=simulate_scan(world, BEARINGS, TOF_CFG, range_max_m=RANGE_MAX),
        angle_min=BEARINGS[0],
        angle_increment=math.radians(BEARING_STEP_DEG),
        range_min=float(DEPLOYED["min_range_m"]),
        range_max=RANGE_MAX,
        stamp=now,
        received_at=now,
        transform_to_base=Transform2D(0.0, 0.0, 0.0),
    )


def _approach_lidar(box, cruise, steps=400):
    cfg = _config()
    supervisor = CollisionStopSupervisor(cfg, now=0.0)
    travelled, now = 0.0, 0.0
    outs = []
    for _ in range(steps):
        now += DT
        moved = Box(box.x_min - travelled, box.x_max - travelled,
                    box.y_min, box.y_max, box.z_min, box.z_max)
        world = World(boxes=[moved])
        supervisor.update_scan(_scan(world, now), now=now)
        decision = supervisor.apply_command(TwistCommand(cruise, 0.0), now=now)
        out = decision.output.linear_x
        outs.append(out)
        if out == 0.0 and len(outs) > 3:
            break
        travelled += out * DT
    return travelled, outs


def test_scenario_5_lidar_half_stops_before_a_box_it_can_see():
    cruise = float(DEPLOYED["max_forward_mps"])
    travelled, outs = _approach_lidar(TALL_BOX, cruise)
    assert outs[-1] == 0.0, f"the supervisor never zeroed: last {outs[-1]:.3f} m/s"
    gap = (TALL_BOX.x_min - travelled) - float(DEPLOYED["footprint_front_m"])
    assert gap > 0.0, (
        f"CONTACT: travelled {travelled:.3f} m, box face now at "
        f"{TALL_BOX.x_min - travelled:.3f} m, footprint front "
        f"{DEPLOYED['footprint_front_m']} m")
    print(f"\nSCENARIO 5 (lidar half): stopped with {gap * 100:.1f} cm between "
          f"footprint and a 40 cm box, after {travelled:.3f} m")
    # CROSS-VALIDATION AGAINST REALITY, and it is the reason to believe this loop at
    # all: the FIELD run of 2026-08-25 stopped 19.4 cm short of a real obstacle
    # (0.290 m from base_link at the STOPPED tick, minus footprint_front 0.0965).
    # This pure-core loop reproduces that to within a centimetre without having been
    # tuned to. Asserted loosely on purpose -- a tight bound here would be fitting the
    # sim to one field sample, which is how a rig starts certifying itself.
    assert 0.15 < gap < 0.25, (
        f"stand-off {gap * 100:.1f} cm disagrees with the 19.4 cm measured in the "
        f"field on 2026-08-25 -- one of the two is wrong and it matters which")


def test_scenario_5_lidar_half_control_it_does_not_stop_for_open_floor():
    """Calibrate the NO with a YES. A supervisor that zeroed unconditionally would
    pass the test above; on an empty world it must keep driving."""
    cruise = float(DEPLOYED["max_forward_mps"])
    cfg = _config()
    supervisor = CollisionStopSupervisor(cfg, now=0.0)
    now, outs = 0.0, []
    for _ in range(40):
        now += DT
        supervisor.update_scan(_scan(World(boxes=[]), now), now=now)
        outs.append(supervisor.apply_command(TwistCommand(cruise, 0.0), now=now).output.linear_x)
    assert outs[-1] > 0.0, (
        f"the supervisor stopped on open floor (last {outs[-1]:.3f} m/s) -- it would "
        f"pass the obstacle test for the wrong reason")


def test_scenario_5_the_tof_half_is_scenario_1_and_is_not_re_asserted_here():
    """The ToF half of this requirement IS scenario 1, which passes at 8.3 cm. It is
    referenced rather than duplicated: two copies of one claim drift, and the row is
    reported as two verdicts precisely so neither can be hidden behind the other."""
    import tests.test_scenario_1_never_touch_what_it_can_see as scenario_1
    assert hasattr(scenario_1, "test_scenario_1_the_brake_stops_before_the_footprint_reaches_the_box")
