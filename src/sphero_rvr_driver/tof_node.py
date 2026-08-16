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

import collections
import dataclasses
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String
import tf2_ros
from tf2_ros import StaticTransformBroadcaster

# The supervisor's planar scan transform, imported rather than re-derived. It is
# tested, it already handles the ~179 deg laser yaw, and a second implementation of
# the same arithmetic is how two consumers come to disagree about where a return is.
# IMPORTING does not modify the safety path -- collision_stop.py's diffstat stays empty.
from sphero_rvr_driver.collision_stop import Transform2D

from sphero_rvr_core.tof_frame import (
    ObstacleDetector, TofConfig, ZONES, N_ZONES, rule_a_rows, scan_min_by_column,
    valid_mm, windowed_rate_hz, zone_point,
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


#: THE DEFAULTS COME FROM TofConfig, NOT FROM LITERALS HERE. Every geometry and
#: detection constant used to be written twice -- once in `TofConfig` and once as this
#: node's declared parameter default -- and the node's copy WON, because it is what
#: gets passed into the config object.
#:
#: That cost a gauntlet gate on 2026-08-14. Bench item J pinned `disagreement_margin_m`
#: to 0.06 and enabled rule B in `TofConfig`; the deployed config carried the citation;
#: the tests were green; and the flying node still reported `rule_b=UNPINNED
#: margin_m=0.1 rules=rule_a_only`, because these two lines still said 0.10 and False.
#: The stack was one gate away from flying a mission believing rule B held authority
#: it did not have.
#:
#: One source of truth. A parameter may still OVERRIDE any of these at runtime -- that
#: is what parameters are for -- but the default is now the same object the pure core
#: uses, so pinning a constant in one place pins it everywhere.
_D = TofConfig()


class TofNode(Node):
    def __init__(self):
        super().__init__("tof")
        self.declare_parameter("i2c_address", 0x33)
        self.declare_parameter("i2c_bus", 1)
        # DFRobot ships the only implementation of this sensor's packet protocol and
        # it is not a pip package. Rather than re-deriving their framing (and owning
        # every bug in it), the node imports theirs from a path -- named here so a
        # missing driver is a CONFIGURATION error with an obvious fix, not an import
        # traceback at bringup. Restoring it is in the provisioning repo's manifest.
        self.declare_parameter("driver_path", "/home/jsperson/tof_smoke")
        self.declare_parameter("frame_id", "tof_link")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 10.0)   # poll faster than the ~7.6 Hz sensor
        # Geometry, JOINTLY FITTED on the tilted mount 2026-08-13 and no longer
        # provisional: the two-distance wall test settled the reporting convention and
        # the floor rows settled the rest. Parameters rather than constants so that a
        # re-mount is a config change -- and a re-mount DOES require one, because these
        # are calibration values, not a description of where the sensor is bolted
        # (design note 9.7). The mount was aimed at 10 deg down; it fits at 15.7.
        # Trusting the intended number over the fitted one is how a floor model ends
        # up describing a robot nobody built.
        self.declare_parameter("mount_height_m", _D.mount_height_m)
        self.declare_parameter("mount_pitch_deg", _D.mount_pitch_deg)     # positive = nose UP
        # THE FIELD OF VIEW IS ASYMMETRIC: ~60 deg across, ~47 deg down. Two separate
        # parameters because they are two separate measurements -- the horizontal from
        # a 25 cm box filling 6 of 8 columns, the vertical from the floor-row fit.
        self.declare_parameter("zone_deg_h", _D.zone_deg_h)
        self.declare_parameter("zone_deg_v", _D.zone_deg_v)
        self.declare_parameter("mount_x_m", _D.mount_x_m)
        self.declare_parameter("reports_z", _D.reports_z)
        self.declare_parameter("floor_margin_m", _D.floor_margin_m)
        self.declare_parameter("floor_horizon_m", _D.floor_horizon_m)
        # Rule A's authority bound. NOT the visibility horizon above -- keeping those
        # two apart is the whole content of design note 9.1.
        self.declare_parameter("stop_distance_m", _D.stop_distance_m)
        # Rule B: the lidar as a live background (design 9.3). The margin is UNPINNED --
        # no synchronised /scan has ever been recorded beside this sensor, and bench
        # item J exists to fix that. Until it does, ~/state says so on every line.
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("disagreement_margin_m", _D.disagreement_margin_m)
        # Gated until bench item J pins the margin above. Rule A flies without J.
        self.declare_parameter("rule_b_enable", _D.rule_b_enable)
        # Matches the supervisor's own scan-staleness bound, deliberately: two different
        # answers to "is this scan usable" is two components disagreeing about reality.
        self.declare_parameter("max_scan_age_s", 0.30)
        self.declare_parameter("laser_frame", "laser")

        _p = self.get_parameter
        self._frame_id = str(_p("frame_id").value)
        self._base_frame = str(_p("base_frame").value)
        self._cfg = TofConfig(
            mount_height_m=float(_p("mount_height_m").value),
            mount_pitch_deg=float(_p("mount_pitch_deg").value),
            # THE SAME PARAMETER THE STATIC TF BELOW USES, and that is the whole point.
            # It was declared, published to TF, and never handed to the geometry that
            # builds the clouds -- so TF said the sensor was 0.10 m forward while every
            # point those clouds carried said it was at base_link. One parameter, two
            # answers, and only the wrong one reached the consumers.
            mount_x_m=float(_p("mount_x_m").value),
            zone_deg_h=float(_p("zone_deg_h").value),
            zone_deg_v=float(_p("zone_deg_v").value),
            reports_z=bool(_p("reports_z").value),
            floor_margin_m=float(_p("floor_margin_m").value),
            floor_horizon_m=float(_p("floor_horizon_m").value),
            stop_distance_m=float(_p("stop_distance_m").value),
            disagreement_margin_m=float(_p("disagreement_margin_m").value),
            rule_b_enable=bool(_p("rule_b_enable").value),
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

        # Rule B's inputs. The lidar plane height comes from TF, never from a constant:
        # `lidar_plane_m`'s default in TofConfig exists only so the pure core runs in a
        # test, and using it here would be the N1 defect in a new place.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._scan = None                 # (monotonic_at, Transform2D, bearings, ranges)
        self._scan_reason = "no_scan_yet"
        self._lidar_plane_m = None
        self.create_subscription(
            LaserScan, str(_p("scan_topic").value), self._on_scan, 5)

        self._lock = threading.Lock()
        self._sensor = None
        self._read_errors = 0
        self._frames = 0
        self._last_frame_at = None
        # Frame stamps for the WINDOWED rate. `_frames` stays as the cumulative
        # total because "how many frames since boot" is a genuine and separate
        # question; what it must never again do is masquerade as the live rate.
        # maxlen bounds memory at a few seconds of the fastest plausible sensor.
        self._rate_window_s = 5.0
        self._frame_stamps = collections.deque(maxlen=256)
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

    def _on_scan(self, msg: LaserScan):
        """Keep the freshest scan, already in ROBOT bearings.

        Transforming HERE rather than at use means the ~179 deg laser yaw is applied
        exactly once, by the same tested primitive the supervisor uses. `scan_min_by_column`
        downstream takes bearings and believes them -- it has no way to notice a mirror,
        which is why the mirror has to be impossible rather than merely unlikely.
        """
        transform, reason = self._lookup_scan_transform(msg.header.frame_id)
        if transform is None:
            with self._lock:
                self._scan, self._scan_reason = None, reason
            return
        bearings, ranges = [], []
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                x, y = transform.transform_point(r * math.cos(angle), r * math.sin(angle))
                bearings.append(math.atan2(y, x))
                ranges.append(math.hypot(x, y))
            angle += msg.angle_increment
        with self._lock:
            self._scan = (time.monotonic(), bearings, ranges)
            self._scan_reason = "ok"

    def _lookup_scan_transform(self, frame_id: str):
        """(Transform2D, reason). Also latches the lidar's PLANE HEIGHT from the same
        lookup -- rule B's applicability test needs it and TF is where it lives."""
        if not frame_id:
            return None, "no_scan_frame"
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, frame_id, rclpy.time.Time())
        except Exception as exc:                       # noqa: BLE001 - reported, not raised
            self.get_logger().warn(f"tof: no TF {self._base_frame}<-{frame_id}: {exc}",
                                   throttle_duration_sec=10.0)
            return None, "no_tf"
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._lidar_plane_m = float(tf.transform.translation.z)
        return Transform2D(x=float(tf.transform.translation.x),
                           y=float(tf.transform.translation.y), yaw=yaw), "ok"

    def _background(self):
        """(column_min, reason) for rule B, or (None, reason) when it is UNAVAILABLE.

        Unavailable is not "clear" and not "blocked" -- design 9.6. Rule B can only ADD
        obstacles, so returning None can only lose sensitivity; it cannot invent a brake.
        Note also what the supervisor is doing during case (1): a scan this stale is
        already SENSOR_STALE over there and the robot is stopping anyway. The requirement
        here is only that this node not manufacture a conclusion in the meantime.
        """
        with self._lock:
            scan, reason = self._scan, self._scan_reason
        if scan is None:
            return None, reason
        at, bearings, ranges = scan
        age = time.monotonic() - at
        if age > float(self.get_parameter("max_scan_age_s").value):
            return None, "stale_scan"
        if self._lidar_plane_m is None:
            return None, "no_lidar_plane"
        return scan_min_by_column(bearings, ranges, self._effective_cfg()), "ok"

    def _effective_cfg(self) -> TofConfig:
        """The config with the lidar plane height TF actually reported. Rebuilt rather
        than mutated because TofConfig is frozen, and frozen is what stops a stray
        assignment somewhere from quietly redefining the geometry mid-run."""
        if self._lidar_plane_m is None:
            return self._cfg
        return dataclasses.replace(self._cfg, lidar_plane_m=self._lidar_plane_m)

    def _open_sensor(self):
        """Open the sensor, or record why not. A driver that cannot reach its hardware
        must SAY SO on ~/state rather than publishing empty clouds that read exactly
        like clear floor."""
        try:
            path = str(self.get_parameter("driver_path").value)
            if path and path not in sys.path:
                sys.path.insert(0, path)
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
            now_mono = time.monotonic()
            self._last_frame_at = now_mono
            self._frame_stamps.append(now_mono)

        points = []
        for i, value in enumerate(frame):
            row, col = divmod(i, ZONES)
            p = zone_point(row, col, value, self._cfg)
            if p is not None:
                points.append(p)
        self._points_pub.publish(_cloud(self._base_frame, stamp, points))

        column_min, background_reason = self._background()
        self._detector.retune(self._effective_cfg())   # TF owns the plane height
        result = self._detector.update(frame, column_min)
        result["background_reason"] = background_reason
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
            stamps = list(self._frame_stamps)
        now_mono = time.monotonic()
        age = None if last is None else now_mono - last
        rate = windowed_rate_hz(stamps, now_mono, self._rate_window_s)
        rate_text = "None" if rate is None else f"{rate:.2f}"
        result = getattr(self, "_last_result", None)
        obstacles = result["obstacles"] if result else []
        rule_a = result["nearer_than_floor"] if result else []
        rule_b = result["confirmed_disagreement"] if result else []
        background = result.get("background_reason", "unknown") if result else "no_frame"
        cfg = self._effective_cfg()
        state = String()
        state.data = (
            f"{'OK' if self._sensor is not None and age is not None and age < 1.0 else 'STALE'} "
            f"frames={frames} rate_hz={rate_text} "
            # The window and the uptime travel WITH the rate, because
            # `rate_hz=None` is honest but not self-explaining: paired with
            # `uptime_s=0.3` it reads "too early to say", and paired with
            # `uptime_s=400` it reads "this sensor has stopped delivering".
            # Without them, None is just another number nobody can act on.
            f"rate_window_s={self._rate_window_s:.1f} "
            f"uptime_s={now_mono - self._started:.1f} "
            f"i2c_errors={errors} "
            f"frame_age_s={'None' if age is None else round(age, 3)} "
            f"obstacle_zones={len(obstacles)} "
            f"rule_a_zones={len(rule_a)} rule_b_zones={len(rule_b)} "
            # WHICH ROWS MAY CONCLUDE, every second. A recording that cannot say this
            # cannot explain what fired -- and the row set is a function of the stop
            # distance, so it is not derivable from the source either (design 9.2).
            f"rule_a_rows={'|'.join(str(r) for r in rule_a_rows(cfg))} "
            f"stop_distance_m={cfg.stop_distance_m} "
            # Rule B's health, and its provenance. UNPINNED is not a warning about this
            # run -- it says the margin below has no recorded data behind it at all, and
            # stays until bench item J supplies some (design 9.8).
            f"rules={result['rules'] if result else 'unknown'} "
            f"background={background} "
            f"lidar_plane_m={'None' if self._lidar_plane_m is None else round(self._lidar_plane_m, 4)} "
            f"rule_b={'UNPINNED' if not cfg.rule_b_enable else 'pinned'} "
            f"margin_m={cfg.disagreement_margin_m} "
            # COUNTED, NEVER ASSERTED. This read `consumers=none_stage_i` -- a literal
            # that was true when the node was written and false from the moment the
            # supervisor subscribed for the low-obstacle brake, which is to say for
            # every flight it has ever appeared in. A recording that says a sensor had
            # no consumers while that sensor is holding the brake is worse than one
            # that says nothing.
            #
            # The node cannot know who its consumers are; it can only MEASURE how many
            # there are. So it measures. The count is also a pairing check for free: on
            # a flight bringup this must be >= 1, and a 0 means the supervisor is not
            # actually subscribed whatever its own config claims.
            f"obstacle_consumers={self._obstacles_pub.get_subscription_count()}"
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
