"""VLM-driven exploration (Stage C north-star): the camera steers where the rover goes.

When idle, grabs a camera frame, asks a vision-LLM (via Synthetic) for a structured
decision — which way to go to explore new/interesting space — and sends a
NavigateToPose goal a lookahead ahead in that direction. On arrival it looks again.
The planner + decisive controller + collision brake keep it safe; this node only
chooses *where to look next*.

Publishes each decision on /vlm_explorer/decision. Run instead of explore_lite /
coverage_explorer. Decision parsing + geometry are pure/tested
(sphero_rvr_core/vlm_client.py). API key from a file (default ~/.config/synthetic/
api_key). Needs the full nav stack + odom (chassis) to actually drive.
"""

import os

import cv2
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from std_msgs.msg import String
import math
import tf2_ros

from sphero_rvr_core.image_decode import imgmsg_to_array
from sphero_rvr_core.vlm_client import direction_to_goal, extract_json, query_vlm

PROMPT = (
    "You steer a small ground robot with a forward-facing camera, exploring an "
    "indoor space. From this image choose the best direction to move to explore new "
    "or interesting space, preferring doorways, hallways and open areas, avoiding "
    "walls and obstacles. Respond with ONLY a JSON object and no other text: "
    '{"turn_deg": <integer -60 to 60, negative=left, positive=right, 0=straight>, '
    '"go": <true if moving that way is safe and useful, false if blocked>, '
    '"reason": "<short phrase>"}'
)


class VlmExplorerNode(Node):
    def __init__(self):
        super().__init__("vlm_explorer")
        self.declare_parameter("api_key_file", os.path.expanduser("~/.config/synthetic/api_key"))
        self.declare_parameter("base_url", "https://api.synthetic.new/v1")
        self.declare_parameter("model", "syn:large:vision")
        self.declare_parameter("image_topic", "/camera_node/image_raw")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("lookahead_m", 1.5)
        self.declare_parameter("max_width", 640)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("decision_period_s", 3.0)
        self.declare_parameter("request_timeout_s", 30.0)

        self._base_url = str(self.get_parameter("base_url").value)
        self._model = str(self.get_parameter("model").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._lookahead = float(self.get_parameter("lookahead_m").value)
        self._max_width = int(self.get_parameter("max_width").value)
        self._jpeg_q = int(self.get_parameter("jpeg_quality").value)
        self._timeout = float(self.get_parameter("request_timeout_s").value)
        self._key = self._read_key(str(self.get_parameter("api_key_file").value))

        self._latest = None
        self._deciding = False
        self._active_goal = None
        cbg = ReentrantCallbackGroup()
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value), self._on_image, 1, callback_group=cbg
        )
        self._decision_pub = self.create_publisher(String, "/vlm_explorer/decision", 10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=cbg)
        self.create_timer(float(self.get_parameter("decision_period_s").value), self._tick, callback_group=cbg)
        self.get_logger().info("vlm_explorer ready (camera-driven exploration)")

    def _read_key(self, path):
        try:
            with open(path) as f:
                return f.read().strip()
        except Exception as e:
            self.get_logger().error(f"could not read API key from {path}: {e}")
            return None

    def _on_image(self, msg):
        self._latest = msg

    def _tick(self):
        if self._deciding or self._active_goal is not None or self._latest is None or not self._key:
            return
        self._deciding = True
        try:
            self._decide_and_go()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"decision cycle failed: {e}")
        finally:
            self._deciding = False

    def _decide_and_go(self):
        img = imgmsg_to_array(self._latest, order="bgr")  # honor step; cv_bridge ignores it -> shear
        h, w = img.shape[:2]
        if w > self._max_width:
            img = cv2.resize(img, (self._max_width, int(h * self._max_width / w)))
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_q])
        # syn:large:vision reasons before emitting the JSON; 300 tokens truncates
        # it mid-monologue (no closing brace -> parse fail). 1500 lets it finish.
        text = query_vlm(
            self._base_url, self._key, self._model, PROMPT, buf.tobytes(),
            max_tokens=1500, timeout=self._timeout, json_mode=True,
        )
        d = extract_json(text)
        turn = max(-60, min(60, int(d.get("turn_deg", 0))))
        go = bool(d.get("go", True))
        reason = str(d.get("reason", ""))
        self._decision_pub.publish(String(data=f"turn_deg={turn} go={go} reason={reason}"))
        self.get_logger().info(f"VLM decision: turn={turn} go={go} — {reason}")
        if not go:
            return  # blocked ahead; wait and look again next cycle
        pose = self._robot_pose()
        if pose is None:
            self.get_logger().warn("no map->base_link TF yet; can't send goal (nav stack/chassis up?)")
            return
        rx, ry, ryaw = pose
        gx, gy, gyaw = direction_to_goal(rx, ry, ryaw, turn, self._lookahead)
        self._send_goal(gx, gy, gyaw)

    def _robot_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, rclpy.time.Time())
        except Exception:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return tf.transform.translation.x, tf.transform.translation.y, yaw

    def _send_goal(self, gx, gy, gyaw):
        if not self._nav.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("navigate_to_pose server not available")
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        goal.pose.pose.orientation = Quaternion(z=math.sin(gyaw / 2.0), w=math.cos(gyaw / 2.0))
        self._active_goal = True
        self.get_logger().info(f"exploring toward ({gx:.2f},{gy:.2f})")
        self._nav.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self._active_goal = None
            return
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self._active_goal = None  # arrived/aborted -> look again next tick


def main(args=None):
    rclpy.init(args=args)
    node = VlmExplorerNode()
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
