#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray , String
import time
# Global variables
front, right, left = 0, 0, 0
degree = 0
threshhold = 13
move = Twist()
situation = "None"
last_sign_time = 0 

def sensors_reading():
    rospy.init_node("Wall_detection_move")
    rospy.loginfo("node set")
    rospy.Subscriber("/ultrasonics", Float32MultiArray, callback=ultrasonic_feedback)
    rospy.Subscriber("/yaw", Float32, callback=yaw_feedback)
    rospy.Subscriber("/direction" , String , callback=direction_feedback)

def ultrasonic_feedback(msg: Float32MultiArray):
    global front, right, left  
    front = msg.data[0]
    right = msg.data[1]
    left  = msg.data[2]

def yaw_feedback(msg: Float32):
    global degree
    degree = msg.data

def move_forward():
    global move
    move.linear.x = 0.2
    move.angular.z = 0

def turn_right():
    global move, degree, pub

    # Fix target once based on current yaw
    target = (degree - 90 + 360) % 360
    rospy.loginfo(f"Turning right, target = {target}")

    move.linear.x = 0
    move.angular.z = -0.3

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(move)

        # Check difference (handling wrap-around at 0/360)
        diff = abs((degree - target + 540) % 360 - 180)
        if diff < 5:   # within 5 degrees tolerance
            break

        rate.sleep()
    # stop rotation
    move.angular.z = 0
    pub.publish(move)
    # rospy.loginfo("Finished turning right")


def turn_left():
    global move, degree, pub

    # Fix target once based on current yaw
    target = (degree + 90) % 360
    # rospy.loginfo(f"Turning left, target = {target}")

    move.linear.x = 0
    move.angular.z = 0.3

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(move)

        # Check difference (handling wrap-around at 0/360)
        diff = abs((degree - target + 540) % 360 - 180)
        if diff < 5:   # within 5 degrees tolerance
            break

        rate.sleep()
    # stop rotation
    move.angular.z = 0
    pub.publish(move)
    # rospy.loginfo("Finished turning right")


def direction_feedback(msg : String) :
    global situation
    situation = msg.data
    rospy.loginfo(f"situation is {situation}")


if __name__ == '__main__':
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sensors_reading()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # rospy.loginfo(f"Front: {front}, Right: {right}, Left: {left}, Yaw: {degree}")
        if front > threshhold:
            move_forward()
            pub.publish(move)
        else:
            now = time.time()
            if (situation == "right" or situation == "left") and (now - last_sign_time > 3):
                if situation == "right":
                    rospy.loginfo("right sign (accepted)")
                    turn_right()
                elif situation == "left":
                    rospy.loginfo("left sign (accepted)")
                    turn_left()
                last_sign_time = now   # reset timer after following a sign
            else :
                if right > left:
                    turn_right()
                elif left> right:
                    turn_left()
            rate.sleep()
