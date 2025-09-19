#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

#ref for equations
#https://answers.ros.org/question/346884/

#hard_coded parameters
r = 0.04
L = 0.39 / 2.0
W = 0.347 / 2.0   #width&length from meshes folder

def wheels_calculation(msg :Twist) :
    #movement of robot in xy plane
    Vx = msg.linear.x
    Vy = msg.linear.y
    Wz = msg.angular.z
    
    La = L + W


    W1 = (1/r) * (Vx - Vy - La * Wz)   # Front-left
    W2 = (1/r) * (Vx + Vy + La * Wz)   # Front-right
    W3 = (1/r) * (Vx + Vy - La * Wz)   # Rear-left
    W4 = (1/r) * (Vx - Vy + La * Wz)   # Rear-right

    wheel_msgs = Float64MultiArray()
    wheel_msgs.data = [W1 , W2 , W3 , W4]
    pub.publish(wheel_msgs)

    rospy.loginfo(f"Wheel speeds: {wheel_msgs.data}")

    

if __name__ == "__main__" :
    rospy.init_node("mechanum_noded" , anonymous= True)

    #pub to wheel_speeds
    pub = rospy.Publisher("wheel_speeds" , Float64MultiArray , queue_size= 10)
    #sub to cmd
    rospy.Subscriber("/cmd_vel" , Twist , wheels_calculation)

    rospy.spin()