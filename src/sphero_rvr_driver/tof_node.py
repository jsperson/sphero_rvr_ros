"""The SEN0628 8x8 ToF rangefinder as a ROS node. Stage (i): PUBLISHES ONLY.

Design: docs/tof_navigation_design.md. Evidence: the 12,869 recorded frames in the
vault at 03_validation/sensor_2026-08-13_tof_characterisation/.

**NOTHING CONSUMES THIS YET, BY DESIGN.** Stage (i) puts the sensor on the graph and
in the recordings beside the camera it will eventually replace, with zero motion
authority, so that stage (ii) can compare the two on real missions before anything
swaps. A node that publishes and is ignored is the entire deliverable.

Three topics, and the split between the first two is the point:

    ~/points      what the SENSOR SAID  -- one point per valid zone, no interpretation
    ~/obstacles   what WE CONCLUDED     -- the detection rules of sphero_rvr_core.tof_frame
    ~/state       whether to believe either -- rate, I2C errors, staleness, counts

A consumer that disagrees with our conclusion can re-derive its own from `points`, and
a recording keeps both. That separation is what let the characterisation be re-analysed
after the first statistic turned out to hide an intermittent target.

I2C, not USB or UART: the RVR already owns the Pi's header UART (`/dev/ttyAMA0`) and
the lidar owns the only USB port, so the bus was chosen by what was free. It is also a
NEW seam for this stack, which is why `~/state` reports read errors rather than letting
silence be mistaken for clear floor.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

from sphero_rvr_core.tof_frame import (
    ObstacleDetector, TofConfig, ZONES, N_ZONES, valid_mm, zone_point,
)


def _cloud(frame_id, stamp, points):
    """XYZ PointCloud2 from (x, y, z) tuples."""
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    import struct
    msg.data = b"".join(struct.pack("<fff", *p) for p in points)
    return msg


class TofNode(Node):
    def __init__(self):
        super().__init__("tof")
        self.declare_parameter("i2c_address", 0x33)
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("frame_id", "tof_link")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 10.0)   # poll faster than the ~7.6 Hz sensor
        # Geometry. PROVISIONAL until the two-distance wall test settles z-vs-radial
        # and the mount is re-fitted -- see the design note 1.1. Exposed as parameters
        # precisely so that re-fit is a config change, not a code change.
        self.declare_parameter("mount_height_m", 0.10)
        # PROVISIONAL. Decided geometry is 10 deg DOWN from level (2026-08-14), but
        # the value that ships is FITTED from the floor rows in the bench session, the
        # same way the original +4 deg was -- aiming a mount by eye and then trusting
        # the intended number is how a floor model ends up describing a robot nobody
        # built. Positive = nose UP, so the intent is negative here.
        self.declare_parameter("mount_pitch_deg", -10.0)
        self.declare_parameter("mount_x_m", 0.10)
        self.declare_parameter("reports_z", True)
        self.declare_parameter("floor_margin_m", 0.12)
        self.declare_parameter("floor_horizon_m", 0.55)

        _p = self.get_parameter
        self._frame_id = str(_p("frame_id").value)
        self._base_frame = str(_p("base_frame").value)
        self._cfg = TofConfig(
            mount_height_m=float(_p("mount_height_m").value),
            mount_pitch_deg=float(_p("mount_pitch_deg").value),
            reports_z=bool(_p("reports_z").value),
            floor_margin_m=float(_p("floor_margin_m").value),
            floor_horizon_m=float(_p("floor_horizon_m").value),
        )
        self._detector = ObstacleDetector(self._cfg)

        self._points_pub = self.create_publisher(PointCloud2, "~/points", 5)
        self._obstacles_pub = self.create_publisher(PointCloud2, "~/obstacles", 5)
        self._state_pub = self.create_publisher(String, "~/state", 10)

        # STATIC TF rather than arithmetic in every consumer. The same rule that fixed
        # N1: geometry belongs to TF, and a consumer that hardcodes a mounting angle is
        # a consumer that steers into a mirror image of open space one day.
        self._tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._base_frame
        t.child_frame_id = self._frame_id
        t.transform.translation.x = float(_p("mount_x_m").value)
        t.transform.translation.z = float(_p("mount_height_m").value)
        pitch = math.radians(-float(_p("mount_pitch_deg").value))   # +pitch = nose UP
        t.transform.rotation.y = math.sin(pitch / 2.0)
        t.transform.rotation.w = math.cos(pitch / 2.0)
        self._tf.sendTransform(t)

        self._lock = threading.Lock()
        self._sensor = None
        self._read_errors = 0
        self._frames = 0
        self._last_frame_at = None
        self._started = time.monotonic()
        self._open_sensor()

        period = 1.0 / max(1.0, float(_p("publish_rate_hz").value))
        self.create_timer(period, self._tick)
        self.create_timer(1.0, self._publish_state)
        self.get_logger().info(
            f"tof ready — I2C 0x{int(_p('i2c_address').value):02x} bus "
            f"{int(_p('i2c_bus').value)}, publishing ~/points ~/obstacles ~/state. "
            "NOTHING CONSUMES THESE YET (stage i)."
        )

    def _open_sensor(self):
        """Open the sensor, or record why not. A driver that cannot reach its hardware
        must SAY SO on ~/state rather than publishing empty clouds that read exactly
        like clear floor."""
        try:
            from DFRobot_matrixLidar import DFRobot_matrixLidar_i2c
            sensor = DFRobot_matrixLidar_i2c(int(self.get_parameter("i2c_address").value))
            sensor.begin()
            for _ in range(3):
                if sensor.set_Ranging_Mode(ZONES) == 0:
                    self._sensor = sensor
                    return
                time.sleep(1.0)
            self.get_logger().error("tof: sensor did not accept 8x8 ranging mode")
        except Exception as exc:                       # noqa: BLE001 - reported, not raised
            self.get_logger().error(f"tof: cannot open sensor: {exc}")

    def _read_frame(self):
        """One frame of 64 millimetre readings, or None. Little-endian uint16 pairs,
        exactly as DFRobot's own driver returns them."""
        if self._sensor is None:
            return None
        try:
            data = self._sensor.get_all_data()
        except Exception as exc:                       # noqa: BLE001
            with self._lock:
                self._read_errors += 1
            self.get_logger().warn(f"tof: I2C read failed: {exc}", throttle_duration_sec=5.0)
            return None
        if not data or len(data) < 2 * N_ZONES:
            with self._lock:
                self._read_errors += 1
            return None
        return [(data[i + 1] << 8) | data[i] for i in range(0, 2 * N_ZONES, 2)]

    def _tick(self):
        frame = self._read_frame()
        if frame is None:
            return
        stamp = self.get_clock().now().to_msg()
        with self._lock:
            self._frames += 1
            self._last_frame_at = time.monotonic()

        points = []
        for i, value in enumerate(frame):
            row, col = divmod(i, ZONES)
            p = zone_point(row, col, value, self._cfg)
            if p is not None:
                points.append(p)
        self._points_pub.publish(_cloud(self._base_frame, stamp, points))

        result = self._detector.update(frame)
        obstacles = []
        for row, col in result["obstacles"]:
            p = zone_point(row, col, frame[row * ZONES + col], self._cfg)
            if p is not None:
                obstacles.append(p)
        self._obstacles_pub.publish(_cloud(self._base_frame, stamp, obstacles))
        self._last_result = result

    def _publish_state(self):
        """Health, once a second. EVENTS AND DISTINCT PLACES kept separate (D35): a
        count of detections reads as a count of obstacles unless the line says which."""
        with self._lock:
            frames, errors, last = self._frames, self._read_errors, self._last_frame_at
        age = None if last is None else time.monotonic() - last
        elapsed = max(1e-6, time.monotonic() - self._started)
        result = getattr(self, "_last_result", None)
        obstacles = result["obstacles"] if result else []
        rule_i = result["nearer_than_floor"] if result else []
        rule_ii = result["confirmed_unexpected"] if result else []
        state = String()
        state.data = (
            f"{'OK' if self._sensor is not None and age is not None and age < 1.0 else 'STALE'} "
            f"frames={frames} rate_hz={frames / elapsed:.2f} "
            f"i2c_errors={errors} "
            f"frame_age_s={'None' if age is None else round(age, 3)} "
            f"obstacle_zones={len(obstacles)} "
            f"rule_i_zones={len(rule_i)} rule_ii_zones={len(rule_ii)} "
            f"consumers=none_stage_i"
        )
        self._state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = TofNode()
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
