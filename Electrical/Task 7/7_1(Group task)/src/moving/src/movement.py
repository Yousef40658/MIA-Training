#!/usr/bin/env python3
import rospy
from std_msgs.msg import Char
from geometry_msgs.msg import Twist

# Max speeds
MAX_LINEAR = 0.33      # m/s
MAX_ANGULAR = 0.3      # rad/s
ACC_STEP = 0.05        # increment per callback (adjust for smoothness)

# Current speeds (start at zero)
current_linear = 0.0
current_angular = 0.0

def callback(msg):
    global current_linear, current_angular

    command = chr(msg.data)
    rospy.loginfo(f"Received through /orders: {command}")

    twist = Twist()

    # Stop all movement if X is pressed
    if command == 'X':
        current_linear = 0.0
        current_angular = 0.0

    # Linear forward/backward
    elif command == 'W':  
        current_linear += ACC_STEP
        if current_linear > MAX_LINEAR:
            current_linear = MAX_LINEAR
        current_angular = 0.0  # cancel rotation
    elif command == 'S':  
        current_linear -= ACC_STEP
        if current_linear < -MAX_LINEAR:
            current_linear = -MAX_LINEAR
        current_angular = 0.0  # cancel rotation

    # Rotation
    elif command == 'A':  
        current_angular += ACC_STEP
        if current_angular > MAX_ANGULAR:
            current_angular = MAX_ANGULAR
        current_linear = 0.0   # cancel linear movement
    elif command == 'D':  
        current_angular -= ACC_STEP
        if current_angular < -MAX_ANGULAR:
            current_angular = -MAX_ANGULAR
        current_linear = 0.0   # cancel linear movement

    else:
        rospy.loginfo(f"Received unknown command: {command}")

    # Set Twist values
    twist.linear.x = current_linear
    twist.angular.z = current_angular

    # Publish Twist message
    cmd_pub.publish(twist)
    rospy.loginfo(f"Published Twist -> linear.x: {twist.linear.x:.2f}, angular.z: {twist.angular.z:.2f}")

def listener():
    rospy.init_node('Robot_movement', anonymous=True)
    rospy.Subscriber("/orders", Char, callback)
    rospy.spin()

if __name__ == '__main__':
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    listener()
