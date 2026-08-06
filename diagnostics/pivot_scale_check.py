"""Measure the pivot over-rotation: commanded vs wheel-odom vs gyro-truth.

CHASSIS REQUIRED, ATTENDED. Give the rover ~1 m of clear space; it pivots in place.

Why: a pivot commanded at 0.4 rad/s for 2.5 s (expect 57 deg) produced ~115 deg of
odom yaw and a visually-estimated ~250 deg. Two suspected causes, which this
separates:

  1. The commanded MAGNITUDE is discarded. driver.py drives every in-place pivot
     toward a fixed `pivot_target_rate_rad_s` (default 1.3 rad/s) and uses only the
     SIGN of angular_rad_s. -> actual rate is the same no matter what you ask for.
  2. The closed loop's feedback is WHEEL ODOMETRY (`set_measured_yaw_rate` is fed
     from the encoder odom tracker). If encoder-derived yaw under-reports true
     rotation (tread scrub), the loop over-drives until the *measured* rate reaches
     the target, so the TRUE rate overshoots the target as well.

Test: pivot at several commanded rates for a fixed duration; compare commanded
angle, odom-integrated angle, and gyro-integrated angle (/imu angular_velocity.z,
the ground truth here).

Reading the result:
  * odom angle ~equal across different commanded rates -> cause 1 confirmed.
  * gyro angle >> odom angle                           -> cause 2 confirmed
                                                          (ratio = the scale error).

Run:  ros2 launch sphero_rvr_driver supervised_rvr.launch.py     # + IMU streaming
      python3 ~/ros2_ws/src/sphero_rvr_ros/diagnostics/pivot_scale_check.py
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

DURATION_S = 2.0
COMMANDED_RATES = [0.3, 0.6, 0.9]  # if the outcome is identical, magnitude is ignored
SETTLE_S = 2.0

rclpy.init()
n = rclpy.create_node("pivot_scale_check")
pub = n.create_publisher(Twist, "/cmd_vel", 10)
st = {"yaw": None, "gyro_z": 0.0}


def on_odom(m):
    q = m.pose.pose.orientation
    st["yaw"] = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def on_imu(m):
    st["gyro_z"] = m.angular_velocity.z


n.create_subscription(Odometry, "/odom", on_odom, 20)
n.create_subscription(Imu, "/imu", on_imu, 20)


def spin(s):
    end = time.monotonic() + s
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.02)


def unwrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def pivot(rate):
    """Accumulate BOTH signals continuously. Start-vs-end yaw aliases the moment the
    rover turns more than 180 deg (it wraps), which is exactly the regime under test,
    so odom yaw is summed per sample with each step unwrapped."""
    spin(SETTLE_S)
    if st["yaw"] is None:
        print("  no /odom -- is the chassis powered and rvr_node up?")
        return None
    odom_angle = 0.0
    gyro_angle = 0.0
    prev_yaw = st["yaw"]
    t0 = time.monotonic()
    last = t0
    while rclpy.ok() and time.monotonic() - t0 < DURATION_S:
        tw = Twist()
        tw.angular.z = float(rate)
        pub.publish(tw)
        rclpy.spin_once(n, timeout_sec=0.02)
        now = time.monotonic()
        gyro_angle += st["gyro_z"] * (now - last)      # integrate gyro rate
        odom_angle += unwrap(st["yaw"] - prev_yaw)     # sum unwrapped odom steps
        prev_yaw = st["yaw"]
        last = now
        time.sleep(0.02)
    for _ in range(10):                                 # stop
        pub.publish(Twist())
        time.sleep(0.05)
    spin(1.5)
    return math.degrees(odom_angle), math.degrees(gyro_angle)


print("Pivoting in place %.1f s per trial. KEEP CLEAR." % DURATION_S)
print("%-10s %-12s %-12s %-12s %s" % ("cmd rad/s", "expect deg", "odom deg", "gyro deg", "gyro/expect"))
rows = []
for rate in COMMANDED_RATES:
    res = pivot(rate)
    if res is None:
        break
    odom_d, gyro_d = res
    expect = math.degrees(rate * DURATION_S)
    rows.append((rate, expect, odom_d, gyro_d))
    print("%-10.2f %-12.1f %-12.1f %-12.1f %.2fx   (true rate %.2f rad/s)"
          % (rate, expect, odom_d, gyro_d, abs(gyro_d) / expect if expect else 0,
             math.radians(abs(gyro_d)) / DURATION_S))
    pub.publish(Twist())

print()
if len(rows) >= 2:
    gyros = [abs(r[3]) for r in rows]
    spread = (max(gyros) - min(gyros)) / max(max(gyros), 1e-6)
    if spread < 0.25:
        print("CAUSE 1 CONFIRMED: rotation barely changes with the commanded rate")
        print("  -> the pivot path ignores the commanded magnitude (fixed target rate).")
    else:
        print("Rotation scales with the command; cause 1 NOT confirmed.")
    ratios = [abs(r[3]) / max(abs(r[2]), 1e-6) for r in rows]
    avg = sum(ratios) / len(ratios)
    print("gyro/odom ratio avg = %.2fx" % avg)
    if avg > 1.3:
        print("CAUSE 2 CONFIRMED: wheel odom UNDER-reports yaw by ~%.1fx, so the" % avg)
        print("  closed loop (fed from odom) over-drives the true pivot rate.")
        print("  FIX: feed set_measured_yaw_rate from the IMU gyro, not the odom tracker.")
for _ in range(10):
    pub.publish(Twist())
    time.sleep(0.05)
print("stopped.")
