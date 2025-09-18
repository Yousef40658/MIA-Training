#!/usr/bin/env python3 

import rospy
from std_msgs.msg import String
from std_msgs.msg import Twist
from enum import Enum 


action_move = String()
def action(msg : String) :
    action_move.data = msg.data.split(":")
    rospy.loginfo(msg) 


if __name__ == "__main__" :
   
    rospy.init_node("Starscream_motion")
    
    motion = rospy.Subscriber("/starscream" , String , action)

    rospy.spin()
