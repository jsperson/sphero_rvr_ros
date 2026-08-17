"""Publish an ALL-CLEAR LaserScan so the reflex supervisor will grant motion.

NOT A SENSOR. This exists only for the chassis-off closed-loop rig
(`launch/sim_closed_loop.launch.py`).

WHY IT IS NEEDED: the collision supervisor refuses every command when no scan is present
(`SENSOR_STALE reason=missing_scan`) -- correctly, because moving blind is exactly what it
exists to prevent. With the lidar absent from the sim rig, that refusal zeroes
`/cmd_vel_motor` and the robot cannot move at all, so the loop never closes and the
falsifier proves nothing. The first falsifier attempt died precisely this way: RPP
commanding at 37 Hz, `/cmd_vel_motor` flat zero, odom never leaving the origin.

WHAT IT MEANS FOR THE CLAIM: the simulated world is **permanently empty**. That is
consistent with this rig's declared scope -- rotation behaviour only, no collision or
inflation behaviour -- but it must never be read as "the supervisor was exercised". The
supervisor's *clamp* is exercised (it still limits angular rate, which is the point); its
*obstacle logic* is not, because it is being shown a world with nothing in it.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

CLEAR_RANGE_M = 6.0
BEAMS = 720
RATE_HZ = 10.0


#: This node must be asked for explicitly. A fake all-clear scan on a real robot would
#: blind the reflex supervisor into granting every command -- the single most dangerous
#: thing in this repository if started by accident. Refusing without an explicit
#: acknowledgement is the same un-startable-by-construction pattern as the simulated
#: chassis and the stationary-test TF.
CONSENT_PARAMETER = "i_understand_this_publishes_a_fake_clear_scan"


class SimClearScanRefused(RuntimeError):
    """The consent parameter was absent or false."""


class ClearScanPublisher(Node):
    def __init__(self) -> None:
        super().__init__("sim_clear_scan")
        self.declare_parameter(CONSENT_PARAMETER, False)
        if not bool(self.get_parameter(CONSENT_PARAMETER).value):
            raise SimClearScanRefused(
                "REFUSED: this node publishes a FAKE ALL-CLEAR /scan, which would blind "
                "the collision supervisor into granting every command on a real robot. "
                f"Set {CONSENT_PARAMETER}:=true to run it, and only in the chassis-off "
                "simulation rig."
            )
        self._pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.create_timer(1.0 / RATE_HZ, self._tick)
        self.get_logger().warning(
            "SIM CLEAR SCAN -- NOT A SENSOR. The simulated world is empty; the "
            "supervisor's clamp is exercised, its obstacle logic is not."
        )

    def _tick(self) -> None:
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "laser"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2.0 * math.pi) / BEAMS
        scan.range_min = 0.15
        scan.range_max = 12.0
        scan.ranges = [CLEAR_RANGE_M] * BEAMS
        self._pub.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ClearScanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
