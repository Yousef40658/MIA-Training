#!/usr/bin/env python3 
import rospy
import random
from std_msgs.msg import String

prev_msg = None

def receive_order(msg : String) :
    global prev_msg
    
    msg_contents = msg.data.split(":")

    if msg_contents[0] == "cyclonus" :
        prev_msg = msg.data
        published_msg = msg.data

    elif msg_contents[0] != "cyclonus" and prev_msg == None :
        published_msg = "cyclonus : standby"

    else  :
        published_msg = prev_msg 

    #Splitting to send only the attack
    published_msg = published_msg.split(":")
    #publish_move
    star_scream_action.publish(String(data = published_msg[1]))
    #what was recivied , to debugg
    rospy.loginfo(f"received {msg}")


if __name__ == "__main__" :
    rospy.init_node("Cyclonus")

    listening_to_order = rospy.Subscriber("/decepticons" , String , callback= receive_order)
    
    star_scream_action = rospy.Publisher("/cyclonus" , String, queue_size= 10) 

    rospy.spin()
