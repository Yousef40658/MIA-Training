#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Char
from sensor_msgs.msg import JointState
import math

# Joint limits and step sizes
JOINT_MIN = -2.83
JOINT_MAX = 2.83
STEP = 0.1  # Step size for joint2 (in radians)
STEP_OTHER = 0.25  # Step size for joint1, joint3, joint4 (0.25 degrees to radians, approx 0.00436332)

# Global variables to store joint positions
joint_positions = {
    "joint1": 0.0,
    "joint2": 0.0,
    "joint3": 0.0,
    "joint4": 0.0
}

def joint_states_callback(msg):
    """Update current joint positions from /joint_states topic."""
    global joint_positions
    for name, pos in zip(msg.name, msg.position):
        if name in joint_positions:
            joint_positions[name] = pos

def orders_callback(msg):
    global joint_positions
    command = chr(msg.data).upper()  # Convert ASCII to character

    # Update joint positions based on command
    # if command == 'U':  # Move joint2 up
    #     joint_positions["joint2"] = min(joint_positions["joint2"] + STEP, JOINT_MAX)
    # elif command == 'B':  # Move joint2 down
    #     joint_positions["joint2"] = max(joint_positions["joint2"] - STEP, JOINT_MIN)
    if command == 'L':  # Move joint1 up
        joint_positions["joint1"] = min(joint_positions["joint1"] + STEP_OTHER, JOINT_MAX)
    elif command == 'J':  # Move joint1 down
        joint_positions["joint1"] = max(joint_positions["joint1"] - STEP_OTHER, JOINT_MIN)
    elif command == 'I':  # Move joint3 up
        joint_positions["joint3"] = min(joint_positions["joint3"] + STEP_OTHER, JOINT_MAX)
    elif command == 'K':  # Move joint3 down
        joint_positions["joint3"] = max(joint_positions["joint3"] - STEP_OTHER, JOINT_MIN)
    elif command == 'O':  # Move joint4 up
        joint_positions["joint4"] = min(joint_positions["joint4"] + STEP_OTHER, JOINT_MAX)
    elif command == 'U':  # Move joint4 down
        joint_positions["joint4"] = max(joint_positions["joint4"] - STEP_OTHER, JOINT_MIN)
    else:
        return  # Ignore other keys

    # Create a trajectory message
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = ["joint1", "joint2", "joint3", "joint4"]  # All expected joints
    point = JointTrajectoryPoint()
    point.positions = [
        joint_positions["joint1"],
        joint_positions["joint2"],
        joint_positions["joint3"],
        joint_positions["joint4"]
    ]
    point.time_from_start = rospy.Duration(0.5)  # Half-second movement
    traj.points.append(point)

    # Publish the message
    pub.publish(traj)
    rospy.loginfo(f"Moved to positions: joint1={joint_positions['joint1']:.2f}, "
                  f"joint2={joint_positions['joint2']:.2f}, "
                  f"joint3={joint_positions['joint3']:.2f}, "
                  f"joint4={joint_positions['joint4']:.2f} rad (command: {command})")

if __name__ == "__main__":
    rospy.init_node("vertical_joint_topic_control")

    # Publisher to the arm command topic
    pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=10)

    # Subscriber to the orders topic
    rospy.Subscriber("/orders", Char, orders_callback)

    # Subscriber to joint states to track current positions
    rospy.Subscriber("/joint_states", JointState, joint_states_callback)

    rospy.loginfo("Vertical joint controller ready. Listening to /orders (U/B for joint2, L/J for joint1, I/K for joint3, P/[ for joint4).")
    rospy.spin()