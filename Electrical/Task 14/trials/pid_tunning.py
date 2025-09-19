#!/usr/bin/env python3
import rospy
import itertools
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np
import csv
import time

# -----------------------------
# PID Class
# -----------------------------
class PID:
    def __init__(self, Kp, Ki, Kd, output_limits=(-1.0, 1.0)):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.out_min, self.out_max = output_limits

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.integral += error * dt
        output = self.Kp*error + self.Ki*self.integral + self.Kd*derivative
        self.prev_error = error
        return max(self.out_min, min(output, self.out_max))

# -----------------------------
# Globals
# -----------------------------
yaw = 0.0
pub = None

def yaw_callback(msg: Float32):
    global yaw
    yaw = msg.data

def shortest_angle_diff(target, current):
    return (target - current + 540.0) % 360.0 - 180.0

# -----------------------------
# Perform one 90° turn with given PID
# -----------------------------
def test_turn(pid, direction="right"):
    global pub, yaw
    pid.reset()
    start_yaw = yaw
    if direction == "right":
        target = (start_yaw - 90 + 360) % 360
    else:
        target = (start_yaw + 90) % 360

    rospy.loginfo(f"Testing PID: {pid.Kp},{pid.Ki},{pid.Kd} -> {direction} turn")

    rate = rospy.Rate(20)
    last_time = rospy.Time.now().to_sec()
    errors = []
    overshoot = 0.0
    settled = False
    settle_time = None

    while not rospy.is_shutdown():
        now = rospy.Time.now().to_sec()
        dt = now - last_time
        last_time = now

        error = shortest_angle_diff(target, yaw)
        errors.append(abs(error))

        # track overshoot
        overshoot = max(overshoot, abs(error))

        if abs(error) < 2.0 and not settled:
            settled = True
            settle_time = now

        if settled and abs(error) < 2.0 and (now - settle_time) > 0.5:
            break

        control = pid.compute(error, dt)
        twist = Twist()
        twist.angular.z = control
        pub.publish(twist)
        rate.sleep()

    # stop robot
    pub.publish(Twist())
    rospy.sleep(0.5)

    final_error = errors[-1] if errors else 999
    return overshoot, final_error

# -----------------------------
# Main autotune
# -----------------------------
if __name__ == "__main__":
    rospy.init_node("pid_autotune")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/yaw", Float32, yaw_callback)

    rospy.sleep(1.0)  # wait for yaw updates

    # grid search ranges
    Kp_vals = np.linspace(0.01, 0.5, 100)   # 0.01, 0.02, 0.03, 0.04, 0.05
    Kd_vals = np.linspace(0.001, 0.1, 100)  # 5 evenly spaced values
    Ki_vals = np.linspace[0.0 , 0.001 , 100]                        # keep 0 for now

    results = []

    for Kp, Ki, Kd in itertools.product(Kp_vals, Ki_vals, Kd_vals):
        pid = PID(Kp, Ki, Kd, output_limits=(-0.5, 0.5))
        overshoot, final_err = test_turn(pid, "right")
        results.append((Kp, Ki, Kd, overshoot, final_err))
        rospy.loginfo(f"PID {Kp},{Ki},{Kd} -> overshoot={overshoot:.2f}, final_err={final_err:.2f}")
        rospy.sleep(1.0)

    # Save results
    with open("pid_results.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Kp", "Ki", "Kd", "Overshoot", "FinalError"])
        writer.writerows(results)

    rospy.loginfo("Autotune finished, results saved to pid_results.csv")
