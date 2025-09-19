#!/usr/bin/env python3
import roslib
roslib.load_manifest('learning_tf') #looks up for the ros package called learning_tf and loads its manifest.xml
                                    #which describes the package's dependencies 
import rospy
import tf
import turtlesim.msg

def handle_turtle_pose (msg , turtle_name) :
    br =tf.TransformBroadcaster()           #object that can publish transformations between frames
     
    br.sendTransform((msg.x , msg.y , 0), #(1 ,) is the syntax to send a one element tuple , z = 0 as its 2D world              
    tf.transformations.quaternion_from_euler(0, 0, msg.theta), #transforms the angles from euler (RPY) to a quaternion
    rospy.Time.now(),
    turtle_name ,                                              # the child's frame
    "world"                                                    # parent's frame
    )


if __name__ == "__main__" :
    rospy.init_node("turtle_tf_broadcaster")
    turtle_name = rospy.get_param("~turtle")     #Gets parameter turtle from the parameter server [set as a dict]
    
    rospy.Subscriber(f"/{turtle_name}/pose",
                    turtlesim.msg.Pose,
                    handle_turtle_pose,
                    turtle_name)


rospy.spin()
