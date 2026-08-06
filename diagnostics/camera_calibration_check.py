"""Validate the camera's horizontal calibration against the lidar (~2 minutes).

Put a VERTICAL BLUE strip (tape works) on a flat surface, aim the rover at it, and
run this. The script finds the tape's image column, converts it to a bearing using
the live calibration, and compares that against the lidar's independent bearing to
the same surface. Agreement within a couple of degrees means cx/fx are sound.

RUN THIS AFTER ANY CAMERA MOUNT CHANGE OR RECALIBRATION -- the low-obstacle
projection, the costmap marks and the camera brake all inherit these intrinsics.

History: on 2026-08-06 cx=538.2 in an 800-wide image looked obviously wrong (cy was
centred, cx was 138 px off -- the fingerprint of a 16:9 calibration applied to 4:3).
This check proved it CORRECT 3/3 (camera +1.7 deg vs lidar +1.7..+4.2 deg). "Fixing"
it to 400 would have injected an 11.5 deg error. Measure before you patch.

Read-only: commands no motion. Needs camera + lidar running.
"""
import math
import time

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image, LaserScan

from sphero_rvr_core.image_decode import imgmsg_to_array

rclpy.init()
n = rclpy.create_node("cx_check")
box = {}
n.create_subscription(Image, "/camera_node/image_raw", lambda m: box.__setitem__("img", m), 1)
n.create_subscription(CameraInfo, "/camera_node/camera_info", lambda m: box.__setitem__("K", np.array(m.k)), 1)
n.create_subscription(LaserScan, "/scan", lambda m: box.__setitem__("scan", m), 1)

t = time.time()
while rclpy.ok() and not all(k in box for k in ("img", "K", "scan")) and time.time() - t < 15:
    rclpy.spin_once(n, timeout_sec=0.3)
if not all(k in box for k in ("img", "K", "scan")):
    print("missing data:", [k for k in ("img", "K", "scan") if k not in box])
    raise SystemExit

img = imgmsg_to_array(box["img"], order="rgb")
h, w = img.shape[:2]
K = box["K"]
fx, cx_cal = K[0], K[2]

# --- find the blue tape (blue is distinctive against a door) ---
hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
mask = cv2.inRange(hsv, np.array([90, 70, 40]), np.array([135, 255, 255]))
cols = mask.sum(axis=0)
if cols.max() < 255 * 10:
    print("no strong blue found (max column score %.0f)" % cols.max())
    print("blue pixels total:", int(mask.sum() / 255))
    raise SystemExit
# weighted centroid of the strongest blue columns
thresh = cols.max() * 0.4
idx = np.where(cols >= thresh)[0]
u_tape = float((idx * cols[idx]).sum() / cols[idx].sum())
print("blue tape found: column u = %.1f  (image centre = %.1f, calibration cx = %.1f)"
      % (u_tape, w / 2.0, cx_cal))
print("  tape spans columns %d..%d, %d blue px" % (idx.min(), idx.max(), int(mask.sum() / 255)))

bearing_cal = math.degrees(math.atan2(u_tape - cx_cal, fx))
bearing_ctr = math.degrees(math.atan2(u_tape - w / 2.0, fx))
print()
print("bearing to tape implied by CURRENT cx=%.1f : %+6.1f deg" % (cx_cal, bearing_cal))
print("bearing to tape implied by CENTRED cx=%.1f : %+6.1f deg" % (w / 2.0, bearing_ctr))

# --- lidar: bearing of the nearest surface straight ahead (the door) ---
scan = box["scan"]
best = None
for i, r in enumerate(scan.ranges):
    if not (scan.range_min < r < scan.range_max) or math.isinf(r) or math.isnan(r):
        continue
    a = scan.angle_min + i * scan.angle_increment
    if abs(a) > math.radians(40):
        continue
    if best is None or r < best[0]:
        best = (r, math.degrees(a))
if best:
    print("\nlidar nearest surface within +/-40 deg: %.2f m at %+.1f deg" % (best[0], best[1]))
    print("  (rover is aimed at the tape, so truth should be near this bearing)")
    d_cal = abs(bearing_cal - best[1])
    d_ctr = abs(bearing_ctr - best[1])
    print("\n  |current cx  - lidar| = %5.1f deg" % d_cal)
    print("  |centred cx  - lidar| = %5.1f deg" % d_ctr)
    print("\n  ==> %s" % ("CENTRED cx is closer: the calibration cx is WRONG"
                          if d_ctr + 2.0 < d_cal else
                          "current cx is closer: calibration looks OK" if d_cal + 2.0 < d_ctr
                          else "inconclusive (difference too small)"))
cv2.imwrite("/tmp/cx_check.jpg", cv2.cvtColor(img, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 88])
vis = img.copy()
cv2.line(vis, (int(u_tape), 0), (int(u_tape), h - 1), (255, 0, 0), 2)          # tape (blue->red line)
cv2.line(vis, (int(w / 2), 0), (int(w / 2), h - 1), (0, 255, 0), 1)            # image centre
cv2.line(vis, (int(cx_cal), 0), (int(cx_cal), h - 1), (255, 255, 0), 1)        # calibration cx
cv2.imwrite("/tmp/cx_vis.jpg", cv2.cvtColor(vis, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 88])
print("\nwrote /tmp/cx_vis.jpg (red=tape, green=image centre, yellow=calibration cx)")
