"""Monocular low-obstacle detector (Stage C, safety track).

Subscribes to the camera image + camera_info, finds each column's floor-boundary
(where an obstacle rises off the floor), projects those ground-contact pixels to
metric points in the base frame via inverse perspective mapping, and publishes them
as a PointCloud2 on `/camera/low_obstacles`. That cloud is meant to be an
observation source for the costmap obstacle layer and/or an input to the collision
brake, covering the sub-lidar blind spot (e.g. chair legs).

First cut: flat-floor + colour-threshold detection (fragile; a depth camera is the
robust path — see docs/camera_low_obstacle_design.md). Decision core is pure/tested
(`sphero_rvr_core/floor_obstacle_detection.py`, `ground_projection.py`).
"""

import math

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

from sphero_rvr_core.floor_obstacle_detection import detect_obstacle_spans
from sphero_rvr_core.ground_projection import object_height_m, pixel_to_ground
from sphero_rvr_core.image_decode import imgmsg_to_array


class LowObstacleNode(Node):
    def __init__(self):
        super().__init__("low_obstacle")
        self.declare_parameter("image_topic", "/camera_node/image_raw")
        self.declare_parameter("camera_info_topic", "/camera_node/camera_info")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_height_m", 0.1143)
        self.declare_parameter("camera_tilt_down_rad", -0.0524)  # calibrated 3 deg UP
        self.declare_parameter("proc_width", 200)
        self.declare_parameter("color_thresh", 40.0)
        self.declare_parameter("adaptive_thresh", True)  # derive threshold from the floor band
        self.declare_parameter("min_run", 4)
        self.declare_parameter("floor_band_frac", 0.12)
        self.declare_parameter("min_range_m", 0.05)
        self.declare_parameter("max_range_m", 2.0)
        self.declare_parameter("max_obstacle_height_m", 0.20)  # keep only sub-lidar-plane obstacles
        self.declare_parameter("process_every_n", 6)  # 30 Hz camera -> ~5 Hz

        self._base_frame = str(self.get_parameter("base_frame").value)
        self._cam_h = float(self.get_parameter("camera_height_m").value)
        self._tilt = float(self.get_parameter("camera_tilt_down_rad").value)
        self._proc_w = int(self.get_parameter("proc_width").value)
        self._color_thresh = float(self.get_parameter("color_thresh").value)
        self._adaptive = bool(self.get_parameter("adaptive_thresh").value)
        self._min_run = int(self.get_parameter("min_run").value)
        self._band = float(self.get_parameter("floor_band_frac").value)
        self._min_r = float(self.get_parameter("min_range_m").value)
        self._max_r = float(self.get_parameter("max_range_m").value)
        self._max_h = float(self.get_parameter("max_obstacle_height_m").value)
        self._every_n = max(1, int(self.get_parameter("process_every_n").value))

        self._K = None  # (fx, fy, cx, cy) at full camera resolution
        self._count = 0
        self.create_subscription(CameraInfo, str(self.get_parameter("camera_info_topic").value), self._on_info, 1)
        self.create_subscription(Image, str(self.get_parameter("image_topic").value), self._on_image, 1)
        self._pub = self.create_publisher(PointCloud2, "/camera/low_obstacles", 5)
        self.get_logger().info("low_obstacle ready (monocular floor-boundary -> /camera/low_obstacles)")

    def _on_info(self, msg):
        k = msg.k
        if len(k) >= 6 and k[0] > 0:
            self._K = (float(k[0]), float(k[4]), float(k[2]), float(k[5]))  # fx, fy, cx, cy

    def _on_image(self, msg):
        self._count += 1
        if self._count % self._every_n != 0 or self._K is None:
            return
        img = imgmsg_to_array(msg, order="rgb")  # honor step; cv_bridge ignores it -> shear
        h0, w0 = img.shape[:2]
        s = self._proc_w / w0
        small = cv2.resize(img, (self._proc_w, max(1, int(h0 * s))))
        fx, fy, cx, cy = (v * s for v in self._K)

        spans = detect_obstacle_spans(
            small,
            floor_band_frac=self._band,
            color_thresh=None if self._adaptive else self._color_thresh,
            min_run=self._min_run,
        )
        points = []
        for u, span in enumerate(spans):
            if span is None:
                continue
            contact, top = span
            g = pixel_to_ground(u, contact, fx, fy, cx, cy, self._cam_h, self._tilt)
            if g is None:
                continue
            fwd, left = g
            if not (self._min_r <= fwd <= self._max_r):
                continue
            rng = math.hypot(fwd, left)
            # keep only obstacles below the lidar plane; taller ones are the lidar's
            # job (and dropping them keeps the camera brake from fighting gap crossing).
            if object_height_m(contact - top, rng, fy) > self._max_h:
                continue
            points.append((float(fwd), float(left), 0.0))

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self._base_frame
        self._pub.publish(pc2.create_cloud_xyz32(header, points))


def main(args=None):
    rclpy.init(args=args)
    node = LowObstacleNode()
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
