"""Semantic map builder (Stage C): remember WHAT was seen and WHERE on the map.

The exploration stack knows geometry but forgets meaning. This node turns camera
observations into a persistent, queryable object list in the map frame -- the thing
the LLM goal layer is meant to reason over ("find the red mug", "explore toward the
kitchen") instead of reasoning over one raw camera frame at a time.

Each observation: grab a frame -> ask the VLM for objects with a horizontal image
position and a rough distance -> convert that bearing+range to a base_link ground
point -> transform to the map frame via TF -> fold into the semantic map, which
merges repeat sightings of the same label instead of duplicating them.

Observation is ON DEMAND by default (`observe` service) because every call costs a
cloud request; set `auto_observe_period_s` > 0 for a hands-off run. The map is
persisted to `map_file` so it survives restarts, and published as JSON on
`/semantic_map/objects`.

Decision/merge logic is pure and unit-tested in `sphero_rvr_core/semantic_map.py`.
"""

import json
import math
import os

import cv2
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros

from sphero_rvr_core.ground_projection import clear_ray_point
from sphero_rvr_core.image_decode import imgmsg_to_array
from sphero_rvr_core.semantic_map import SemanticMap
from sphero_rvr_core.vlm_client import extract_json, query_vlm

PROMPT = (
    "You are the vision system of a small ground robot mapping an indoor space. "
    "List the notable, permanent-ish objects you can see (furniture, appliances, "
    "doors, boxes, equipment). Ignore the floor, walls and ceiling themselves. "
    "Respond with ONLY a JSON object: "
    '{"objects": [{"label": "<short noun phrase>", '
    '"u": <horizontal centre as a fraction of image width, 0.0=left 1.0=right>, '
    '"distance_m": <rough distance in metres>, '
    '"confidence": <0.0-1.0>}]}'
)


class SemanticMapNode(Node):
    def __init__(self):
        super().__init__("semantic_map")
        self.declare_parameter("api_key_file", os.path.expanduser("~/.config/synthetic/api_key"))
        self.declare_parameter("base_url", "https://api.synthetic.new/v1")
        self.declare_parameter("model", "syn:large:vision")
        self.declare_parameter("image_topic", "/camera_node/image_raw")
        self.declare_parameter("camera_info_topic", "/camera_node/camera_info")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_file", os.path.expanduser("~/.ros/semantic_map.json"))
        self.declare_parameter("merge_radius_m", 0.6)
        self.declare_parameter("min_confidence", 0.3)
        self.declare_parameter("max_distance_m", 5.0)
        self.declare_parameter("max_width", 640)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("request_timeout_s", 45.0)
        self.declare_parameter("auto_observe_period_s", 0.0)  # 0 = on demand only

        self._base_url = str(self.get_parameter("base_url").value)
        self._model = str(self.get_parameter("model").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._map_file = str(self.get_parameter("map_file").value)
        self._min_conf = float(self.get_parameter("min_confidence").value)
        self._max_dist = float(self.get_parameter("max_distance_m").value)
        self._max_width = int(self.get_parameter("max_width").value)
        self._jpeg_q = int(self.get_parameter("jpeg_quality").value)
        self._timeout = float(self.get_parameter("request_timeout_s").value)
        self._key = self._read_key(str(self.get_parameter("api_key_file").value))

        self._map = self._load_map(float(self.get_parameter("merge_radius_m").value))
        self._latest = None
        self._K = None
        self._busy = False

        cbg = ReentrantCallbackGroup()
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value), self._on_image, 1, callback_group=cbg
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value), self._on_info, 1, callback_group=cbg
        )
        self._pub = self.create_publisher(String, "/semantic_map/objects", 10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.create_service(Trigger, "observe", self._on_observe, callback_group=cbg)
        self.create_service(Trigger, "semantic_map/publish", self._on_publish, callback_group=cbg)
        period = float(self.get_parameter("auto_observe_period_s").value)
        if period > 0.0:
            self.create_timer(period, self._auto_observe, callback_group=cbg)
        self.get_logger().info(
            f"semantic_map ready ({len(self._map)} objects loaded; "
            f"{'auto' if period > 0 else 'on-demand'} observation)"
        )

    # --- setup helpers -----------------------------------------------------
    def _read_key(self, path):
        try:
            with open(path) as f:
                return f.read().strip()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"could not read API key from {path}: {e}")
            return None

    def _load_map(self, merge_radius):
        try:
            with open(self._map_file) as f:
                return SemanticMap.from_json(f.read(), merge_radius_m=merge_radius)
        except FileNotFoundError:
            return SemanticMap(merge_radius_m=merge_radius, min_confidence=0.0)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"could not load {self._map_file}: {e}; starting empty")
            return SemanticMap(merge_radius_m=merge_radius, min_confidence=0.0)

    def _save_map(self):
        try:
            os.makedirs(os.path.dirname(self._map_file), exist_ok=True)
            with open(self._map_file, "w") as f:
                f.write(self._map.to_json(indent=2))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"could not save {self._map_file}: {e}")

    def _on_image(self, msg):
        self._latest = msg

    def _on_info(self, msg):
        k = msg.k
        if len(k) >= 6 and k[0] > 0:
            self._K = (float(k[0]), float(k[2]))  # fx, cx at full resolution

    # --- observation -------------------------------------------------------
    def _auto_observe(self):
        if not self._busy:
            self._observe()

    def _on_observe(self, request, response):
        ok, message = self._observe()
        response.success = ok
        response.message = message
        return response

    def _on_publish(self, request, response):
        self._publish()
        response.success = True
        response.message = f"{len(self._map)} objects"
        return response

    def _observe(self):
        if self._busy:
            return False, "an observation is already in flight"
        if self._latest is None or self._K is None:
            return False, "no camera image/info yet"
        if not self._key:
            return False, "no API key"
        pose = self._robot_pose()
        if pose is None:
            return False, f"no {self._map_frame}->{self._base_frame} TF (nav stack up?)"
        self._busy = True
        try:
            objects = self._query_objects()
            added = self._integrate(objects, pose)
            self._save_map()
            self._publish()
            return True, f"observed {len(objects)} object(s); map now holds {len(self._map)} ({added} placed)"
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"observation failed: {e}")
            return False, f"observation failed: {e}"
        finally:
            self._busy = False

    def _query_objects(self):
        img = imgmsg_to_array(self._latest, order="bgr")
        h, w = img.shape[:2]
        if w > self._max_width:
            img = cv2.resize(img, (self._max_width, int(h * self._max_width / w)))
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_q])
        if not ok:
            raise RuntimeError("JPEG encode failed")
        # syn:large:vision reasons before the JSON; it needs the token headroom.
        text = query_vlm(
            self._base_url, self._key, self._model, PROMPT, buf.tobytes(),
            max_tokens=1500, timeout=self._timeout, json_mode=True,
        )
        return extract_json(text).get("objects", []) or []

    def _integrate(self, objects, pose):
        """Place each detection on the map. Returns how many were accepted."""
        rx, ry, ryaw = pose
        fx, cx = self._K
        width = float(self._latest.width)
        placed = 0
        stamp = self.get_clock().now().nanoseconds / 1e9
        for obj in objects:
            try:
                label = str(obj["label"]).strip()
                u_frac = float(obj.get("u", 0.5))
                dist = float(obj.get("distance_m", 0.0))
                conf = float(obj.get("confidence", 0.5))
            except (KeyError, TypeError, ValueError):
                continue
            if not label or conf < self._min_conf or not (0.05 < dist <= self._max_dist):
                continue
            # bearing from the image column, at the VLM's rough range
            fwd, left = clear_ray_point(u_frac * width, cx, fx, dist)
            mx = rx + fwd * math.cos(ryaw) - left * math.sin(ryaw)
            my = ry + fwd * math.sin(ryaw) + left * math.cos(ryaw)
            if self._map.observe(label, mx, my, confidence=conf, stamp=stamp):
                placed += 1
        return placed

    def _robot_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return tf.transform.translation.x, tf.transform.translation.y, yaw

    def _publish(self):
        self._pub.publish(String(data=json.dumps({"objects": [o.to_dict() for o in self._map.objects()]})))


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
