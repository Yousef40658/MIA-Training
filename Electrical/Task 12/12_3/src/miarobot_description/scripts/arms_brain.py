#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

#gets the message info using
#rosmsg show geometry_msgs/PoseStamped

class ArmsBrain():
    def __init__(self):
        #initializing
        rospy.init_node("arms_brain" , anonymous= True)
        self.sub = rospy.Subscriber("/goal" , PoseStamped , self.goal_detection)  #i found it with the name /goal
        self.pub = rospy.Publisher("joint_commands" ,Float64MultiArray , queue_size= 10)
        rospy.loginfo("Arm brain node started")

        self.base_height = 0.05 + 0.25 #base + robot height
        self.arm_length  = 0.2 + 0.05 #0.05 for gripper length

        rospy.spin()

    def goal_detection(self,msg : PoseStamped) :
        #get position
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        rospy.loginfo(f"received goal x : {x} , y :{y} , z : {z}")

        #limitations
        distance_xy = math.sqrt(x**2 + y**2) #distance in xy plane
        distance_z  = z - self.base_height  #arm extension

        if (distance_xy > self.arm_length) or (0.4 < distance_z) or (distance_z> 0.4) :
            rospy.logwarn("Goal unreachable")
            return
        
        #inverse kinematics
        else :
            joint1 = z - self.base_height
            joint2 = math.atan2(y,x)    #rotation around z

        #get the msg type info using 
        cmd = Float64MultiArray()
        cmd.data = [joint1,joint2]

        self.pub.publish(cmd)
        rospy.loginfo(f"Published joint commands: joint1={joint1:.2f}, joint2={joint2:.2f}")


            


if __name__ == "__main__":
    arms_brain = ArmsBrain()

        