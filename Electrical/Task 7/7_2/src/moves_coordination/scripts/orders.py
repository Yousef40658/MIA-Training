#!/usr/bin/env python3 
import rospy
import random
from std_msgs.msg import String



if __name__ == "__main__" :
    rospy.init_node("Megatron_orders" , anonymous= False    )
    rospy.loginfo("Node connected")

    pub = rospy.Publisher("/decepticons" , String , queue_size= 10) #the topic will be created once the node is made

    rate = rospy.Rate(5) #frequency


    messages = ["starscream: Attack", "starscream: Defend" ,"cyclonus: Attack" ,"cyclonus: Defend"]


    while not rospy.is_shutdown() :
        #randomizing 
        rand_index = random.randint(0,3)
        rand_message = messages[rand_index] 
        pub.publish(rand_message)
        rate.sleep()

