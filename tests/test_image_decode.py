"""Tests for stride-honoring image decode (the fix for cv_bridge's step-ignoring
diagonal shear on padded camera strides)."""

import numpy as np

from sphero_rvr_core.image_decode import imgmsg_to_array


class FakeImg:
    def __init__(self, data, width, height, step, encoding):
        self.data = data
        self.width = width
        self.height = height
        self.step = step
        self.encoding = encoding


def _padded_msg(img_hwc, step, encoding):
    """Pack an (H,W,C) array into a buffer with `step` bytes/row (extra = padding)."""
    h, w, c = img_hwc.shape
    buf = np.zeros((h, step), dtype=np.uint8)
    buf[:, : w * c] = img_hwc.reshape(h, w * c)
    return FakeImg(buf.tobytes(), w, h, step, encoding)


def test_padded_stride_recovers_image():
    # 4x3 RGB image with a distinct value per pixel; pad the row stride by 5 bytes.
    h, w, c = 3, 4, 3
    img = (np.arange(h * w * c, dtype=np.uint8)).reshape(h, w, c)
    msg = _padded_msg(img, step=w * c + 5, encoding="rgb8")
    out = imgmsg_to_array(msg)
    assert out.shape == (h, w, c)
    assert np.array_equal(out, img)  # padding stripped, no shear


def test_zero_step_falls_back_to_contiguous():
    h, w, c = 2, 2, 3
    img = np.arange(h * w * c, dtype=np.uint8).reshape(h, w, c)
    msg = _padded_msg(img, step=w * c, encoding="rgb8")
    msg.step = 0  # some publishers leave step unset
    out = imgmsg_to_array(msg)
    assert np.array_equal(out, img)


def test_order_bgr_swaps_channels_of_rgb():
    h, w, c = 1, 1, 3
    img = np.array([[[10, 20, 30]]], dtype=np.uint8)  # rgb
    msg = _padded_msg(img, step=w * c, encoding="rgb8")
    out = imgmsg_to_array(msg, order="bgr")
    assert list(out[0, 0]) == [30, 20, 10]


def test_order_native_keeps_channels():
    h, w, c = 1, 1, 3
    img = np.array([[[10, 20, 30]]], dtype=np.uint8)
    msg = _padded_msg(img, step=w * c, encoding="rgb8")
    out = imgmsg_to_array(msg)  # native
    assert list(out[0, 0]) == [10, 20, 30]


def test_mono8_single_channel():
    h, w = 2, 3
    img = np.arange(h * w, dtype=np.uint8).reshape(h, w, 1)
    msg = _padded_msg(img, step=w + 2, encoding="mono8")
    out = imgmsg_to_array(msg)
    assert out.shape == (h, w, 1) and np.array_equal(out, img)
