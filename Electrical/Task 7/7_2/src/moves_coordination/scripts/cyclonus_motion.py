#!/usr/bin/env python3 

import rospy
from std_msgs.msg import String

action_move = String()
def action(msg : String) :
    action_move.data = msg.data.split(":")
    rospy.loginfo(msg) 


if __name__ == "__main__" :
   
    rospy.init_node("Cyclonus_motion")
    
    motion = rospy.Subscriber("/cyclonus" , String , action)

    rospy.spin()
