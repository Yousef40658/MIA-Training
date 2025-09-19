#!/usr/bin/env python3

import rospy
from std_msgs.msg import Char
from pynput import keyboard   # no root needed
import string


if __name__ == "__main__" :
    #publisher node
    rospy.init_node("Keyboard")
    pub = rospy.Publisher("/orders" ,Char , queue_size= 10)
    rospy.loginfo("Press W/A/S/D/E/Q (ESC to quit)...")

    #loop rate
    rate = rospy.Rate(50)

    #key pressing
    def on_press(key):
        try:
            k = key.char.upper()                    # get letter, convert to uppercase
            if k in string.ascii_uppercase :
                pub.publish(ord(k))                 # ord returns ASCII
                rospy.loginfo(f"Sent {k} ({ord(k)})")
        except AttributeError:
            if key == keyboard.Key.esc:             # ESC = quit
                rospy.loginfo("ESC pressed, shutting down...")
                rospy.signal_shutdown("ESC pressed")

    # start listener
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # keep node alive
    while not rospy.is_shutdown():
        rate.sleep()

    listener.stop()
