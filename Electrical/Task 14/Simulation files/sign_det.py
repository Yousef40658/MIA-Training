#!/usr/bin/env python3
import cv2
from ultralytics import YOLO
from sensor_msgs.msg import Image
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge
import rospkg
import os



rospack=rospkg.RosPack()
package_path=rospack.get_path('miarobot_description')
model_path=os.path.join(package_path,'scripts','SignModel.pt')


model = YOLO(model_path)
bridge=CvBridge()
pub=None
def callback(msg):
    global pub
    frame=bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
    result=model(frame)[0]
    if len(result.boxes)==0:
        rospy.loginfo("none")
        pub.publish("none")
        return
    else:
        for box in result.boxes:
            cls_id=int(box.cls[0])
            cls_name=model.names[cls_id]
            if cls_name.lower()=="left":
                rospy.loginfo("left")
                pub.publish("left")
            elif cls_name.lower()=="right":
                rospy.loginfo("right")
                pub.publish("right")
            else:
                rospy.loginfo("none")
                pub.publish("none")
    

def main():
    global pub
    rospy.init_node('sign_det',anonymous=True)
    pub=rospy.Publisher("directiont",String,queue_size=10)
    rospy.Subscriber("/camera",Image, callback)
    rospy.loginfo("start")
    rospy.spin()
if __name__=="__main__":
    try :
        main()
    except rospy.ROSInterruptException:
        pass
