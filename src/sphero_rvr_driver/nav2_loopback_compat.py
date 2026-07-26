"""Jazzy loopback adapter used only by the Phase 1 replay launch.

nav2_loopback_sim 1.0.0 initializes the odom-to-base quaternion to
``(0, 0, 0, 0)`` after the first ``/initialpose`` message.  That quaternion
is invalid and becomes NaN as soon as the simulator integrates a command.
Keep the upstream simulator and narrowly repair that initial value.
"""

from __future__ import annotations


def main(args=None) -> None:
    import rclpy
    from nav2_loopback_sim.loopback_simulator import LoopbackSimulator
    from rclpy.executors import ExternalShutdownException

    class RvrLoopbackSimulator(LoopbackSimulator):
        def getMap(self) -> None:
            # The Jazzy 1.0.0 simulator requests /map during construction,
            # before Nav2's lifecycle manager activates map_server.  The
            # service then returns an empty map (resolution 0), which crashes
            # the simulator's ray caster.  Global planning still consumes the
            # recorded map directly; the loopback scan intentionally falls
            # back to deterministic max-range samples.
            self.map = None
            self.get_logger().info(
                "Loopback scan uses deterministic max range; "
                "recorded map remains owned by Nav2 map_server"
            )

        def initialPoseCallback(self, msg) -> None:
            first_pose = self.initial_pose is None
            super().initialPoseCallback(msg)
            if first_pose:
                rotation = self.t_odom_to_base_link.transform.rotation
                if (
                    rotation.x == 0.0
                    and rotation.y == 0.0
                    and rotation.z == 0.0
                    and rotation.w == 0.0
                ):
                    rotation.w = 1.0

    rclpy.init(args=args)
    node = RvrLoopbackSimulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
