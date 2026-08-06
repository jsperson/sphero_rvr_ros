"""Measure the steady-state in-place pivot rate for the CURRENTLY RUNNING driver.

Used by a duty sweep: restart rvr_node with a different `pivot_min_duty`, run this,
and read off the rate. The closed-loop pivot targets `pivot_target_rate_rad_s`
(1.3 rad/s) but clamps duty to [pivot_min_duty, pivot_max_duty] -- if the FLOOR
already spins faster than the target, the loop is saturated at its minimum and
cannot slow down. This finds the floor that actually yields the target.

Reports the rate over the last part of the run (steady state, after the duty ramp)
from the gyro, plus any motor stall/fault the firmware reported -- too low a duty
grinds instead of rolling, which is the reason the floor exists.

Usage: python3 pivot_duty_measure.py [duration_s] [commanded_rate]
"""

import math
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
RATE_CMD = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
STEADY_FRAC = 0.5  # measure the rate over the last half (after the ramp)

rclpy.init()
n = rclpy.create_node("pivot_duty_measure")
pub = n.create_publisher(Twist, "/cmd_vel", 10)
st = {"gyro": 0.0, "stall": "0", "fault": "0"}


def on_imu(m):
    st["gyro"] = m.angular_velocity.z


def on_diag(m):
    for s in m.status:
        for kv in s.values:
            if kv.key in ("motor_stall_event_count", "motor_stall"):
                st["stall"] = kv.value
            elif kv.key in ("motor_fault_event_count", "motor_fault"):
                st["fault"] = kv.value


n.create_subscription(Imu, "/imu", on_imu, 20)
n.create_subscription(DiagnosticArray, "/diagnostics", on_diag, 10)


def spin(s):
    end = time.monotonic() + s
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.02)


spin(2.0)
stall_before = st["stall"]
samples = []
t0 = time.monotonic()
while rclpy.ok() and time.monotonic() - t0 < DURATION:
    tw = Twist()
    tw.angular.z = RATE_CMD
    pub.publish(tw)
    rclpy.spin_once(n, timeout_sec=0.02)
    samples.append((time.monotonic() - t0, abs(st["gyro"])))
    time.sleep(0.02)
for _ in range(12):
    pub.publish(Twist())
    time.sleep(0.05)
spin(1.5)

steady = [g for t, g in samples if t >= DURATION * STEADY_FRAC]
peak = max((g for _, g in samples), default=0.0)
mean = sum(steady) / len(steady) if steady else 0.0
print("steady_rate_rad_s=%.2f peak_rad_s=%.2f stall_before=%s stall_after=%s fault=%s"
      % (mean, peak, stall_before, st["stall"], st["fault"]))
