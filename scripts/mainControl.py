#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import *

class mainControl:
    def __init__(self):
        
        rospy.Subscriber("/transition", String, callback)
        self.joy_control = rospy.Subscriber("/vesc/joy", Joy, joy_buttons)
        self.wall_pub = rospy.Publisher("/wallfollow", String, queue_size=1)
        rospy.Publisher("/transition", String, queue_size=10)

    def callback(self, msg):
        if msg.data == "follow left" or msg.data == "follow right" or msg.data == "stop follow"
            self.wall_pub.publish(msg)
    def joy_buttons(self, msg):


if __name__=="__main__":
    rospy.init_node("MainControl")
    node = Control()
    rospy.spin()
