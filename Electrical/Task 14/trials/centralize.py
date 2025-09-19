 #!/usr/bin/env python
import PID_centralize
import rospy 
from std_msgs.msg import Int16, Int16MultiArray ,Float32
#from PID_centralize import PID_centralize
#pass the distance to PID_centralize.py

#subscribe on /wall_detection topic check for (2)'corridor'
#centralize the robot
# keep changing the angle until the robot is centralized
#subscribe on /ultrasonic topic
#check if ultrasonic[1]==ultrasonic[2] or yaw==0


class Centrilize:

    def __init__(self):

        self.wall_type=None
        self.ultrasonic_data= []
        self.yaw_angle=None
        self.distance=None

        rospy.Subscriber("/wall_detection", Int16, self.centralize)
        self.pub = rospy.Publisher("/PID_US", Float32, queue_size=10)

    def centralize(self, detected):
        self.wall_type = detected.data
        if self.wall_type == 2:  # Check if the third element is 'corridor'
            rospy.Subscriber("/ultrasonic", Int16MultiArray, self.store_ultra)
            rospy.Subscriber("/yaw", Int16, self.store_yaw)
            if self.ultrasonic_data[0] == self.ultrasonic_data[1] or self.yaw_angle == 0:
                self.distance = 0.0  # Robot is centralized
            else:
                self.distance = self.ultrasonic_data[0] - self.ultrasonic_data[1]  # Adjust distance to centralize
                
            rospy.loginfo("Publishing distance error: %s", self.distance)
            self.pub.publish(self.distance)

            # Code to centralize the robot goes here
            # This could involve changing the robot's angle until it is centered
        else:
            rospy.loginfo("Not in corridor, no action taken.")

    def store_ultra(self, sensor_data):
        rospy.loginfo("Received data: %s", sensor_data.data)
        self.ultrasonic_data = sensor_data.data[1:3]


    def store_yaw(self, angle):
        # Placeholder function to get the robot's yaw
        self.yaw_angle = angle.data


def listener():
    #create a node that subscribes to /wall_detection and /ultrasonic, /yaw
    rospy.init_node('centralize', anonymous=True)
    
    centrilize = Centrilize()#create object
    centrilize.centralize()#use method
    #pub=rospy.Publisher('/PID_US', Int16, queue_size=10)
    while not rospy.is_shutdown():
        rospy.loginfo(centrilize.distance)
        rospy.Rate(10).sleep()
    rospy.spin()
    
if __name__ == '__main__':
    listener()