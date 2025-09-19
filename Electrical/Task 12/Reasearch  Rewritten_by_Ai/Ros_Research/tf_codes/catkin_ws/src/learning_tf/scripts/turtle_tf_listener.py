#!/usr/bin/env python3
import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv


if __name__ == '__main__' :
    rospy.init_node('turtle_tf_listener') 

    listener = tf.TransformListener() #creates an object that subscribes to /tf 
    rospy.wait_for_service('spawn')   #block until the spawn service provided by the model is available
    
    spawner = rospy.ServiceProxy('spawn' , turtlesim.srv.Spawn)
    spawner(4,2,0,'turtle2')          #spawns turtle 2 with position and name

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1) #queue_size =1 to avoid lag/backlog

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try :
            (trans,rot) = listener.lookupTransform('/turtle2' , '/carrot1' , rospy.Time(0)) #target frame , source_frame , rospy.Time(0) takes the last common type between both
                                                                                            #source is expressed -translated- to target
                                                                                            #returns (x,y,z)> how far they're for trans
                                                                                            #rot returns (qx , qy , qz , qw) encodes the orientation of the source frame relative to target
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): 
            continue


        angular = 4 * math.atan2(trans[1], trans[0]) #(x,y ) is the vector from turtle 2 to turtle 1 , atan(x,y) gets the direction of this vector #higher rotation speed
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2) #magnitude of the vector #lower speed
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)

        rate.sleep()