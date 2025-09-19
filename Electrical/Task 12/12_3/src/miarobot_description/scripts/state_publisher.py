#!/usr/bin/env python3
import rospy
import tf
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class StatePublisher:

    def __init__(self):
        # Initialize node
        rospy.init_node("state_publisher", anonymous=True)

        # TF broadcaster
        self.br = tf.TransformBroadcaster()

        # Subscribe to joint_states
        rospy.Subscriber("/joint_states", JointState, self.joint_movement)

        #--------------
        #Arm brain
        # rospy.Subscriber("/joint_commands" , Float64MultiArray , self.arm_brain_integration)
        #--------------

        #state last transforms
        self.last_transforms = {}
        #
        rospy.loginfo("State publisher node activated")
        rospy.spin()

    # ----------------
    # Helper function
    # ----------------
    def transformation(self, parent, child,
                       tx=0, ty=0, tz=0,
                       Xth=0, Yth=0, Zth=0):
        #only publish when something changes to remove red redundant readings 
        transform_key = (
            round(tx, 6), round(ty, 6), round(tz, 6),
            round(Xth, 6), round(Yth, 6), round(Zth, 6)
        )

        #publishing only when changed to prevent redundancy readings
        if self.last_transforms.get(child) == transform_key :
            return
        
        self.last_transforms[child] = transform_key

        self.br.sendTransform(
            (tx, ty, tz),  # translation
            tf.transformations.quaternion_from_euler(Xth, Yth, Zth),
            rospy.Time.now(),
            child,
            parent
        )
    # ----------------
    # Joint updates
    # ----------------
    def joint_movement(self, msg: JointState):  #works with sliders
        # Extract values
        joint1 = msg.position[msg.name.index("joint1")]       # prismatic z
        joint2 = msg.position[msg.name.index("link2_joint")]  # revolute z
        left_finger = msg.position[msg.name.index("left_finger_joint")]   # prismatic y
        right_finger = msg.position[msg.name.index("right_finger_joint")] # prismatic -y

        # base -> joint1 (prismatic z)
        self.transformation("arm_base", "joint1_link",
                            0, 0, joint1,
                            0, 0, 0)

        # joint1 -> joint2 (revolute around z)
        self.transformation("joint1_link", "joint2_link",
                            0, 0, 0,
                            0, 0, joint2)

        # gripper_base -> left_finger (prismatic y+)
        self.transformation("gripper_base", "left_finger",
                            0.0,  + left_finger, 0.0,
                            0, 0, 0)

        # gripper_base -> right_finger (prismatic y-)
        self.transformation("gripper_base", "right_finger",
                            0.0,  - right_finger, -0.0,
                            0, 0, 0)
        
    #------------
    #Arm_brain
    def arm_brain_integration(self,msg:Float64MultiArray) :
        joint1 = msg.data[0]
        joint2 = msg.data[1]
        #
        left_finger = 0.0
        right_finger = 0.0

        rospy.loginfo(f"Received /joint_commands: joint1={joint1:.2f}, joint2={joint2:.2f}")

        # base -> joint1 (prismatic z)
        self.transformation("arm_base", "joint1_link",
                            0, 0, joint1,
                            0, 0, 0)

        # joint1 -> joint2 (revolute around z)
        self.transformation("joint1_link", "joint2_link",
                            0, 0, 0,
                            0, 0, joint2)


        # gripper_base -> left_finger (prismatic y+)
        self.transformation("gripper_base", "left_finger",
                            0.0,left_finger, -0.,
                            0, 0, 0)

        # gripper_base -> right_finger (prismatic y-)
        self.transformation("gripper_base", "right_finger",
                            0.0, - right_finger, -0.0,
                            0, 0, 0)



if __name__ == "__main__":
    try:
        StatePublisher()
    except rospy.ROSInterruptException:
        pass
