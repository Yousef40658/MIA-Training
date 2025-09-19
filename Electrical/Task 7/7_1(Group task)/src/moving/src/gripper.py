#!/usr/bin/env python3

import rospy
from std_msgs.msg import Char
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Initialize the gripper state
gripper_state = 0.0  # 0.0 = open, 1.0 = closed

def change_grip(state: float):
    """Publish a JointTrajectory to open/close the gripper."""
    global gripper_state
    gripper_state = state

    traj = JointTrajectory()
    traj.joint_names = ["gripper"]  

    point = JointTrajectoryPoint()
    point.positions = [state]
    point.time_from_start = rospy.Duration(1.0)
    traj.points = [point]

    grip_state_pub.publish(traj)

def handle_order(order: Char):
    """Open or close the gripper based on the button pressed."""
    key = chr(order.data)
    if key == 'Z':  # Open
        rospy.loginfo("Opening")
        change_grip(0.019)
    elif key == 'C':  # Close
        change_grip(-0.01)
        rospy.loginfo("Closing")
                    
if __name__ == "__main__":
    rospy.init_node("Arm_Gripper")

    # Publisher to send commands to the gripper
    grip_state_pub = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=10)

    # Subscriber to listen for key/button orders
    gripper_sub = rospy.Subscriber("/orders", Char, callback=handle_order)

    rospy.loginfo("Gripper node started. Press 'O' to open, 'C' to close.")
    rospy.spin()
