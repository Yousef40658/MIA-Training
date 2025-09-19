to this
#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt, atan2, sin, cos

class PID:
    def __init__(self, kp, ki, kd):
        # PID paras
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    # Computing PID values
    def compute(self, setpoint, current_value):
        error = setpoint - current_value
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class TurtlebotPID:
    def __init__(self):
        #robot_state
        self.x = 0.0 
        self.y = 0.0 
        self.yaw = 0.0 

        #goal
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None

        #create two PID controllers: one for linear, one for angular
        self.pid_lin = PID(0.6, 0.0, 0.08)
        self.pid_ang = PID(1.6, 0.0, 0.18)

        #ROS initialization 
        rospy.init_node("Turtlebot_PID")
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #get goal from rviz
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.get_goal)
        #get current position from odom
        rospy.Subscriber("/odom", Odometry, self.write_new_vel)

    #goal from rviz
    def get_goal(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        # rotation only yaw matters
        rot = msg.pose.orientation
        _, _, self.goal_yaw = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        rospy.loginfo(f"New Goal: ({self.goal_x:.2f}, {self.goal_y:.2f}, {self.goal_yaw:.2f})")

    #convert odometry into a Twist command using PID control
    def write_new_vel(self, data: Odometry):
        #extract position
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y

        #convert quaternion to yaw (theta)
        q = data.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if self.goal_x is None:
            return

        #compute distance error
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance = sqrt(dx**2 + dy**2)

        #compute angular error
        target_yaw = atan2(dy, dx)
        yaw_error = target_yaw - self.yaw
        #normalize between -pi and pi
        yaw_error = atan2(sin(yaw_error), cos(yaw_error))

        #compute velocities with PID
        linear_velocity = self.pid_lin.compute(0.0, -distance)   # distance → 0
        angular_velocity = self.pid_ang.compute(0.0, -yaw_error) # yaw error → 0

        #stop condition
        if distance < 0.05:
            linear_velocity = 0.0
            angular_velocity = 0.0

        #limit velocities
        linear_velocity = max(min(linear_velocity, 0.3), -0.3)
        angular_velocity = max(min(angular_velocity, 1.5), -1.5)

      
        new_pose = Twist()
        new_pose.linear.x = linear_velocity
        new_pose.angular.z = angular_velocity
        self.cmd_pub.publish(new_pose)

if __name__ == "__main__":
    controller = TurtlebotPID()
    rospy.spin()