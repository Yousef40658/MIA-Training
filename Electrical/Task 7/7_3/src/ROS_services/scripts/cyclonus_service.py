#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger , TriggerResponse
import random

moves = ['Attack' , 'Defend']

def handle_megatron_request(req) :
    move = random.choice(moves)
    rospy.loginfo(f"cyclonus:{move}")
    return TriggerResponse(success = True , message = move)

def cyclonus_server () :
    rospy.init_node("cyclonus")

    service = rospy.Service('cyclonus', Trigger , handle_megatron_request)
    rospy.spin()

if __name__ == "__main__" :
    cyclonus_server()

