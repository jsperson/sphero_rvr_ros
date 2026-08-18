"""Persistent map->odom->base_link publisher. Robot x is read from /tmp/robot_x each tick.

Separate from the probe on purpose: local_costmap will not activate without this TF and
STOPS UPDATING the moment it disappears, so a probe that owns the TF starves the thing it
is measuring the instant it exits.
"""
import rclpy, os
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

PATH = "/tmp/robot_x"

class TF(Node):
    def __init__(self):
        super().__init__("probe_tf_publisher")
        self.b = TransformBroadcaster(self)
        self.create_timer(0.05, self.tick)
    def tick(self):
        try:
            x = float(open(PATH).read().strip())
        except Exception:
            x = 0.0
        now = self.get_clock().now().to_msg()
        out = []
        for parent, child, tx in (("map","odom",0.0), ("odom","base_link",x)):
            t = TransformStamped()
            t.header.stamp = now; t.header.frame_id = parent; t.child_frame_id = child
            t.transform.translation.x = tx; t.transform.rotation.w = 1.0
            out.append(t)
        self.b.sendTransform(out)

rclpy.init(); n = TF()
try: rclpy.spin(n)
except KeyboardInterrupt: pass
