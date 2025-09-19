#!/usr/bin/env python3

# This node publishes a transform "carrot1" relative to "turtle1"
# The transform moves in a circle over time (scaled by pi)
# Additionally, it demonstrates time-shifted transforms:
#   turtle2 follows turtle1 but delayed by a few seconds.

import rospy
import tf
import math

if __name__ == "__main__" :
    rospy.init_node('dynamic_broadcaster')

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    delay_secs = 2.0  # <-- how many seconds turtle2 lags behind turtle1

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        t = now.to_sec() * math.pi  # multiply by pi to scale time

        # Carrot moving relative to turtle1
        br.sendTransform(
            (2 * math.sin(t), 2 * math.cos(t), 0),
            (0.0, 0.0, 0.0, 1.0),
            now,
            "carrot1",
            "turtle1"
        )

        # Turtle2 following turtle1 with a time offset
        past_time = now - rospy.Duration(delay_secs)
        br.sendTransform(
            (0.0, 0.0, 0.0),  # no offset, just "follow"
            (0.0, 0.0, 0.0, 1.0),
            past_time,        # <-- use the delayed timestamp
            "turtle2",
            "turtle1"
        )

        rate.sleep()
