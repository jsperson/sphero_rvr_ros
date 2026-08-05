"""VLM scene understanding for exploration (Stage C, semantic track).

On demand (a `describe_scene` service call), grabs the latest camera frame, sends it
to a vision-LLM via Synthetic's OpenAI-compatible API, and returns/publishes a short
scene description aimed at exploration: room type, notable objects, openings toward
unexplored space, and the most interesting direction to explore next.

Deliberately NOT every-frame — a cloud VLM call per frame is slow and costly. Call
the service when you want a read (e.g., at each exploration goal). The API key is
read from a file (default ~/.config/synthetic/api_key, 600 perms) — never hardcoded.
"""

import base64
import os

import cv2
import requests
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from sphero_rvr_core.image_decode import imgmsg_to_array

DEFAULT_PROMPT = (
    "You are the vision system of a small ground robot exploring an indoor space. "
    "From this camera image, reply concisely with: (1) the room/area type; "
    "(2) notable objects and roughly where they are (left/center/right); (3) any "
    "doorways, hallways, or openings leading to unexplored space, and their "
    "direction; (4) the single most interesting direction to explore next and why. "
    "Keep it under 100 words."
)


class VlmSceneNode(Node):
    def __init__(self):
        super().__init__("vlm_scene")
        self.declare_parameter("api_key_file", os.path.expanduser("~/.config/synthetic/api_key"))
        self.declare_parameter("base_url", "https://api.synthetic.new/v1")
        # Synthetic vision aliases: syn:large:vision (more capable) / syn:small:vision.
        self.declare_parameter("model", "syn:large:vision")
        self.declare_parameter("image_topic", "/camera_node/image_raw")
        self.declare_parameter("max_width", 640)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("max_tokens", 400)
        self.declare_parameter("request_timeout_s", 30.0)
        self.declare_parameter("prompt", DEFAULT_PROMPT)

        self._base_url = str(self.get_parameter("base_url").value).rstrip("/")
        self._model = str(self.get_parameter("model").value)
        self._max_width = int(self.get_parameter("max_width").value)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self._max_tokens = int(self.get_parameter("max_tokens").value)
        self._timeout = float(self.get_parameter("request_timeout_s").value)
        self._prompt = str(self.get_parameter("prompt").value)
        self._key = self._read_key(str(self.get_parameter("api_key_file").value))

        self._latest = None
        cbg = ReentrantCallbackGroup()
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value), self._on_image, 1, callback_group=cbg
        )
        self._pub = self.create_publisher(String, "/vlm_scene/description", 10)
        self.create_service(Trigger, "describe_scene", self._describe, callback_group=cbg)
        self.get_logger().info(
            f"vlm_scene ready (model {self._model}) — call the 'describe_scene' service"
        )

    def _read_key(self, path):
        try:
            with open(path) as f:
                return f.read().strip()
        except Exception as e:
            self.get_logger().error(f"could not read API key from {path}: {e}")
            return None

    def _on_image(self, msg):
        self._latest = msg

    def _describe(self, request, response):
        frame = self._latest
        if frame is None:
            response.success = False
            response.message = "no camera frame received yet"
            return response
        if not self._key:
            response.success = False
            response.message = "no API key (check api_key_file)"
            return response
        try:
            b64 = self._encode_jpeg(frame)
            desc = self._query_vlm(b64)
            self._pub.publish(String(data=desc))
            self.get_logger().info(f"scene: {desc}")
            response.success = True
            response.message = desc
        except Exception as e:  # noqa: BLE001 - surface any failure to the caller
            response.success = False
            response.message = f"VLM query failed: {e}"
            self.get_logger().warn(response.message)
        return response

    def _encode_jpeg(self, msg):
        img = imgmsg_to_array(msg, order="bgr")  # honor step; cv_bridge ignores it -> shear
        h, w = img.shape[:2]
        if w > self._max_width:
            img = cv2.resize(img, (self._max_width, int(h * self._max_width / w)))
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
        if not ok:
            raise RuntimeError("JPEG encode failed")
        return base64.b64encode(buf.tobytes()).decode("ascii")

    def _query_vlm(self, b64):
        payload = {
            "model": self._model,
            "max_tokens": self._max_tokens,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": self._prompt},
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
                    ],
                }
            ],
        }
        # The model occasionally returns a garbage chat-template token instead of a
        # real answer; retry a couple of times before giving up.
        last = ""
        for _ in range(3):
            r = requests.post(
                f"{self._base_url}/chat/completions",
                headers={"Authorization": f"Bearer {self._key}", "Content-Type": "application/json"},
                json=payload,
                timeout=self._timeout,
            )
            r.raise_for_status()
            content = r.json()["choices"][0]["message"]["content"].strip()
            if len(content) >= 15 and "<|" not in content:
                return content
            last = content
        raise RuntimeError(f"VLM returned no usable text after retries (last: {last!r})")


def main(args=None):
    rclpy.init(args=args)
    node = VlmSceneNode()
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
