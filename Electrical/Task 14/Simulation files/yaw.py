#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16


def imu_callback(msg : Imu):
    x = msg.orientation.x
    y = msg.orientation.y
    z = msg.orientation.z
    w = msg.orientation.w

    # quaternion -> yaw
    yaw = math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
    yaw_degree = int(math.degrees(yaw))

    pub.publish(yaw_degree)


if __name__ == "__main__" :
    rospy.init_node("yaw_reader")
    pub = rospy.Publisher("/yaw" , Int16 , queue_size= 10)
    rospy.Subscriber("/imu" , Imu , imu_callback)
    rospy.spin()

