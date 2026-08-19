"""map->odom for the rig, with SLAM's lag — because a static TF cannot fail.

WHY THIS NODE EXISTS, named after it cost three field marks: the closed-loop rig
pinned map->odom with a `static_transform_publisher`, and tf2 treats a static
transform as TIMELESS — valid at every stamp ever asked for. So an exact-stamp
lookup against the rig is UNFALSIFIABLE: it cannot throw ExtrapolationException no
matter how the code under test behaves in time. That is how contact_marker v1's
lookup-at-stamp shipped rig-green and then lost 3 of 3 real contacts on 2026-08-18,
where the real SLAM's map->odom ran 69–87 ms behind the contact stamps.

This node publishes the same identity map->odom the static publisher did, but as a
NON-static transform whose stamp trails wall clock by `lag_ms`, at `period_ms`
cadence, with a deterministic longer gap every `gap_every_n` cycles — the measured
shape of the real feed (run 3d: p50 ~0, p99 103 ms, max 396 ms between stamps).
Under it, code that looks up "now" or a fresh message stamp fails exactly as it
does in flight, and the falsifier gate can be run: the known-bad code must go
0/N here BEFORE the fixed code's N/N means anything.

Deterministic by construction (a cycle counter, no clocks-as-randomness): the same
run replays the same gaps.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class SimLaggyMapTf(Node):
    def __init__(self) -> None:
        super().__init__("sim_laggy_map_tf")
        self.declare_parameter("lag_ms", 90.0)          # run-3d contact staleness class
        self.declare_parameter("period_ms", 50.0)       # ~SLAM transform_publish_period
        self.declare_parameter("gap_every_n", 40)       # one long gap every ~2 s
        self.declare_parameter("gap_extra_ms", 300.0)   # → worst stamp gap ~350 ms
        # slam_toolbox FUTURE-DATES map->odom by its transform_timeout (deployed:
        # 0.2 s) so consumers looking up "now" normally find a stamp that LEADS
        # wall clock. This node originally omitted that, which is the right shape
        # for falsifying PAST-stamp lookups (contact_marker) but a world reality
        # does not present for now-lookups: cert attempt 1 (2026-08-18) had RPP
        # abort 5/5 goals in 13 s on 2 ms extrapolation-into-the-future errors
        # that field SLAM never produces. DEFAULT 0 keeps every existing
        # falsifier use byte-identical; the mission arms pass 200.0 to mirror
        # the deployed slam transform_timeout.
        self.declare_parameter("future_date_ms", 0.0)
        self._future_s = float(self.get_parameter("future_date_ms").value) / 1000.0
        self._lag_s = float(self.get_parameter("lag_ms").value) / 1000.0
        self._gap_every_n = int(self.get_parameter("gap_every_n").value)
        self._gap_extra = float(self.get_parameter("gap_extra_ms").value) / 1000.0
        self._period_s = float(self.get_parameter("period_ms").value) / 1000.0
        self._broadcaster = TransformBroadcaster(self)
        self._cycle = 0
        self._skip_until: float | None = None
        self.create_timer(self._period_s, self._tick)
        self.get_logger().info(
            f"laggy map->odom up: stamp trails now by {self._lag_s * 1000:.0f} ms, "
            f"period {self._period_s * 1000:.0f} ms, +{self._gap_extra * 1000:.0f} ms "
            f"gap every {self._gap_every_n} cycles. A rig on the static publisher "
            f"cannot falsify TF-timing code; this one can."
        )

    def _tick(self) -> None:
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        if self._skip_until is not None:
            if now_s < self._skip_until:
                return
            self._skip_until = None
        self._cycle += 1
        if self._gap_every_n > 0 and self._cycle % self._gap_every_n == 0:
            self._skip_until = now_s + self._gap_extra
        msg = TransformStamped()
        stamped = now - rclpy.duration.Duration(seconds=self._lag_s) \
            + rclpy.duration.Duration(seconds=self._future_s)
        msg.header.stamp = stamped.to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "odom"
        msg.transform.rotation.w = 1.0
        self._broadcaster.sendTransform(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimLaggyMapTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
