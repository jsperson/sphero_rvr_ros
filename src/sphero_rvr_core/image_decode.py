"""Decode a sensor_msgs/Image honoring the row stride (`step`).

Python `cv_bridge.imgmsg_to_cv2` reshapes the buffer as if each row were a
contiguous `width * channels` bytes and ignores the message's `step` field. When
the camera driver pads the row stride for alignment (the Pi 5 PiSP pipeline aligns
to 64 bytes: 800*3 = 2400 payload, but `step` = 2432 with 32 pad bytes/row), that
assumption slips every row by the pad amount and accumulates into a diagonal shear
that also scrambles the colour channels. Reshaping to `(height, step)` and slicing
off the padding before the `(height, width, channels)` view fixes it.

Pure (numpy only) so it can be unit-tested without ROS or a camera.
"""

import numpy as np

# channel count per supported ROS image encoding
_CHANNELS = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1, "8uc1": 1, "8uc3": 3}


def imgmsg_to_array(msg, order=None):
    """(H, W, C) uint8 array from a sensor_msgs/Image, honoring `msg.step`.

    `order`: None keeps the message's native channel order; "rgb" or "bgr" forces
    that order (swapping R/B for 3-channel encodings), so callers that need a
    specific order (e.g. cv2/JPEG want BGR) get it explicitly.
    """
    enc = msg.encoding.lower()
    channels = _CHANNELS.get(enc)
    if channels is None:
        raise ValueError(f"unsupported image encoding {msg.encoding!r}")
    row_bytes = msg.width * channels
    step = msg.step if msg.step else row_bytes
    buf = np.frombuffer(msg.data, dtype=np.uint8)[: step * msg.height].reshape(msg.height, step)
    img = buf[:, :row_bytes].reshape(msg.height, msg.width, channels)
    if order in ("rgb", "bgr") and channels == 3:
        native = "rgb" if enc.startswith("rgb") else "bgr"
        if order != native:
            img = img[:, :, ::-1]
    return np.ascontiguousarray(img)
