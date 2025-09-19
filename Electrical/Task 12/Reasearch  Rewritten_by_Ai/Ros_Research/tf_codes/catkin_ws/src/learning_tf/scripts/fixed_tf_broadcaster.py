#!/usr/bin/env python3
import roslib
roslib.load_manifest('learning_tf')


import rospy
import tf

if __name__ == '__main__' :
    rospy.init_node("fixed_tf")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        br.sendTransform(
            (0.0,2.0,0.0),      #translation from parent to child
            (0 ,0 ,0 ,1),       #rotation
            rospy.Time.now(),   #timestamp
            "carrot1",          #child
            "turtle1"           #parent
        )