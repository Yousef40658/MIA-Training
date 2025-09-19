#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray

class UltrasonicsMerger:
    def __init__(self):
        # Storage for distances
        self.front = 0.0
        self.right = 0.0
        self.left = 0.0

        # Subscribers
        rospy.Subscriber("/ultrasonics_front", Range, self.front_cb)
        rospy.Subscriber("/ultrasonics_right", Range, self.right_cb)
        rospy.Subscriber("/ultrasonics_left", Range, self.left_cb)

        # Publisher
        self.pub = rospy.Publisher("/ultrasonics", Float32MultiArray, queue_size=10)

    def front_cb(self, msg):
        self.front = msg.range
        self.publish()

    def right_cb(self, msg):
        self.right = msg.range
        self.publish()

    def left_cb(self, msg):
        self.left = msg.range
        self.publish()

    def publish(self):
        arr = Float32MultiArray()
        arr.data = [self.front * 100, self.right * 100, self.left * 100]
        self.pub.publish(arr)

if __name__ == "__main__":
    rospy.init_node("ultrasonics_merger")
    UltrasonicsMerger()
    rospy.spin()
