#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger , TriggerResponse
import random
from std_msgs.msg import String


def megatron_server_client () :
    #publisher_node
    rospy.init_node("megatron")
    pub = rospy.Publisher('/decepticons',String, queue_size= 10)
    rospy.loginfo("Node Connected")
    rate = rospy.Rate(2)


    #waiting for service
    rospy.wait_for_service('starscream')
    rospy.wait_for_service('cyclonus')

    starscream_client =  rospy.ServiceProxy('starscream' , Trigger)
    cyclonus_client = rospy.ServiceProxy('cyclonus' , Trigger)

    while not rospy.is_shutdown() :

        starscream_response = starscream_client() 
        starscream_msg = f"Starscream : {starscream_response.message}"
        pub.publish(starscream_msg)

        cyclonus_response = cyclonus_client() 
        cyclonus_msg = f"Cyclonus : {cyclonus_response.message}"
        pub.publish(cyclonus_msg)

        rate.sleep()


if __name__ == "__main__" :
    megatron_server_client() 

        
