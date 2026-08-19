"""look_and_recognize — the recognition primitive's node half.

One on-demand verb per the camera charter: intelligence only, never concurrent
with driving, camera up -> snapshot -> camera DOWN unconditionally. The brain
is `sphero_rvr_core.recognition` (pure, tested anywhere); this file adapts the
camera lifecycle, the supervisor's stationarity fact, TF, and the VLM call.
Design + consensus pins: docs/design_recognition_primitive_2026-08-19.md.

INVOCATION (typed-scalar doctrine — the `ros2 param set` segfault memory is why
the target is a plain string parameter, never a JSON request):

    ros2 param set /recognition target "dr pepper bottle"
    ros2 service call /recognition/look_and_recognize std_srvs/srv/Trigger

The Trigger response carries the full JSON result (provenance included), also
latched on /recognition/result. The photo lands under ~/recognitions/.

KEY HANDLING (consensus pin): the key is read at INVOCATION time from the
operational route, never committed, never logged, never in the result —
`build_result`'s signature cannot carry it by construction — and ABSENCE IS A
LOUD REFUSAL naming the missing prerequisite (path, never contents).
"""

from __future__ import annotations

import json
import math
import os
import re
import signal
import subprocess
import time

import cv2
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros

from sphero_rvr_core.image_decode import imgmsg_to_array
from sphero_rvr_core.recognition import (
    build_prompt,
    build_result,
    parse_recognition_reply,
    pick_sharpest,
)
from sphero_rvr_core.vlm_client import query_vlm


class RecognitionNode(Node):
    def __init__(self):
        super().__init__("recognition")
        self.declare_parameter("target", "")
        self.declare_parameter("api_key_file",
                               os.path.expanduser("~/.config/synthetic/api_key"))
        self.declare_parameter("base_url", "https://api.synthetic.new/v1")
        self.declare_parameter("model", "syn:large:vision")
        # The known-good config: json_mode + headroom (the model reasons before
        # the JSON; small caps truncate it — synthetic-vlm memory, 2026-08).
        self.declare_parameter("max_tokens", 1500)
        self.declare_parameter("request_timeout_s", 60.0)
        self.declare_parameter("image_topic", "/camera_node/image_raw")
        self.declare_parameter("frame_count", 3)
        self.declare_parameter("camera_timeout_s", 25.0)
        self.declare_parameter("max_width", 640)
        self.declare_parameter("jpeg_quality", 80)
        # The charter's stationarity gate reads the SUPERVISOR'S OWN state line
        # (assert-don't-infer: the owner of "is motion commanded" publishes it).
        # Fail-closed: no fresh state line = refusal. A chassis-free bench that
        # runs no supervisor sets require_stationary:=false, with eyes open.
        self.declare_parameter("require_stationary", True)
        self.declare_parameter("state_topic", "/collision_stop/state")
        self.declare_parameter("photo_dir", os.path.expanduser("~/recognitions"))
        self.declare_parameter(
            "camera_launch_cmd",
            "ros2 launch sphero_rvr_driver camera.launch.py")

        self._frames: list = []
        self._collect = False
        self._state = None            # (text, monotonic)
        cbg = ReentrantCallbackGroup()
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value),
            self._on_image, 3, callback_group=cbg)
        self.create_subscription(
            String, str(self.get_parameter("state_topic").value),
            lambda m: setattr(self, "_state", (m.data, time.monotonic())),
            10, callback_group=cbg)
        self._result_pub = self.create_publisher(String, "~/result", 1)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.create_service(Trigger, "~/look_and_recognize", self._look,
                            callback_group=cbg)
        self.get_logger().info(
            "recognition ready — set the 'target' parameter, then call "
            "~/look_and_recognize. Camera runs ONLY inside an invocation.")

    # ---- topic plumbing ---------------------------------------------------------

    def _on_image(self, msg):
        if self._collect:
            self._frames.append(msg)

    # ---- the refusal ladder (every refusal names its reason) --------------------

    def _refuse(self, response, reason):
        response.success = False
        response.message = f"REFUSED: {reason}"
        self.get_logger().warning(response.message)
        return response

    def _stationary(self):
        """(ok, reason). The supervisor's requested/output pair, fresh."""
        if self._state is None:
            return False, "no /collision_stop/state seen — cannot prove the " \
                          "rover is stationary (fail-closed; benches without " \
                          "a supervisor set require_stationary:=false)"
        text, at = self._state
        if time.monotonic() - at > 2.0:
            return False, "supervisor state line is stale"
        pairs = re.findall(r"(requested|output)=\(([-0-9.]+),([-0-9.]+)\)", text)
        if len(pairs) < 2:
            return False, "state line lacks requested/output fields"
        for _name, a, b in pairs:
            if abs(float(a)) > 1e-6 or abs(float(b)) > 1e-6:
                return False, "motion is commanded — the camera never runs " \
                              "concurrent with driving (the charter)"
        return True, ""

    # ---- the verb ----------------------------------------------------------------

    def _look(self, request, response):
        target = str(self.get_parameter("target").value).strip()
        if not target:
            return self._refuse(response, "no target set — "
                                "ros2 param set /recognition target '<thing>'")
        # KEY AT INVOCATION TIME (the pin): read now, refuse loudly if absent.
        key_path = str(self.get_parameter("api_key_file").value)
        try:
            with open(key_path) as fh:
                api_key = fh.read().strip()
        except OSError:
            api_key = ""
        if not api_key:
            return self._refuse(response,
                                f"no VLM key at {key_path} — the verb does not "
                                f"silently no-op without its prerequisite")
        if bool(self.get_parameter("require_stationary").value):
            ok, reason = self._stationary()
            if not ok:
                return self._refuse(response, reason)

        frames = self._snapshot()
        if not frames:
            return self._refuse(response, "camera produced no frames within "
                                "the timeout (camera DOWN again; see log)")
        try:
            arrays = [imgmsg_to_array(f, order="bgr") for f in frames]
            grays = [a.mean(axis=2) for a in arrays]
            best = pick_sharpest(grays)
            frame, img = frames[best], arrays[best]
            pose = self._pose_at(frame.header.stamp)
            if pose is None:
                return self._refuse(response, "no map->base_link TF at the "
                                    "frame stamp — a sighting at an invented "
                                    "pose is worse than none")
            jpeg = self._encode(img)
            reply = query_vlm(
                str(self.get_parameter("base_url").value).rstrip("/"),
                api_key,
                str(self.get_parameter("model").value),
                build_prompt(target), jpeg,
                max_tokens=int(self.get_parameter("max_tokens").value),
                timeout=float(self.get_parameter("request_timeout_s").value),
                json_mode=True)
            parsed = parse_recognition_reply(reply)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            photo_dir = str(self.get_parameter("photo_dir").value)
            os.makedirs(photo_dir, exist_ok=True)
            photo_path = os.path.join(photo_dir, f"recognition_{stamp}.jpg")
            with open(photo_path, "wb") as fh:
                fh.write(jpeg)
            result = build_result(
                target=target, parsed=parsed, photo_path=photo_path,
                map_x=pose[0], map_y=pose[1], capture_yaw_rad=pose[2],
                stamp=stamp, model=str(self.get_parameter("model").value))
        except (ValueError, RuntimeError) as exc:
            # Parse/contract breaches are LOUD refusals, never empty results.
            return self._refuse(response, f"recognition failed: {exc}")
        payload = json.dumps(result)
        self._result_pub.publish(String(data=payload))
        self.get_logger().info(f"recognition: {payload}")
        response.success = True
        response.message = payload
        return response

    # ---- camera lifecycle: up -> N frames -> DOWN, unconditionally ---------------

    def _snapshot(self):
        """Start the camera pipeline, collect frames, stop it. The stop is in a
        finally: the charter's snapshot-then-down survives every error path."""
        self._frames = []
        cmd = str(self.get_parameter("camera_launch_cmd").value)
        want = int(self.get_parameter("frame_count").value)
        deadline = time.monotonic() + float(
            self.get_parameter("camera_timeout_s").value)
        proc = subprocess.Popen(cmd, shell=True, start_new_session=True,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL,
                                stdin=subprocess.DEVNULL)
        try:
            self._collect = True
            while len(self._frames) < want and time.monotonic() < deadline:
                time.sleep(0.1)
            return list(self._frames[:want])
        finally:
            self._collect = False
            self._frames = []
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=10.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
            self.get_logger().info("camera DOWN (snapshot-then-down)")

    # ---- provenance helpers -------------------------------------------------------

    def _pose_at(self, stamp):
        try:
            tf = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return (t.x, t.y, yaw)

    def _encode(self, img):
        max_w = int(self.get_parameter("max_width").value)
        h, w = img.shape[:2]
        if w > max_w:
            img = cv2.resize(img, (max_w, int(h * max_w / w)))
        ok, buf = cv2.imencode(
            ".jpg", img,
            [cv2.IMWRITE_JPEG_QUALITY, int(self.get_parameter("jpeg_quality").value)])
        if not ok:
            raise RuntimeError("JPEG encode failed")
        return buf.tobytes()


def main(args=None):
    rclpy.init(args=args)
    node = RecognitionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
