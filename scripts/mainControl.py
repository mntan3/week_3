#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import *

class mainControl:
    #Other nodes will be controlled from here
    def __init__(self):
        rospy.Subscriber("/transition", String, self.transition_callback)
	rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=10)
        self.joy_control = rospy.Subscriber("/vesc/joy", Joy, joy_buttons)

        #Add publisher for every node that is to be controlled
        self.wall_pub = rospy.Publisher("/wallfollow", String, queue_size=1)
        rospy.Publisher("/transition", String, queue_size=10)

	#

    #When message recieved from /transition topic, start the appropriate node by publishing a message 
    def transition_callback(self, msg):
        if msg.data == "follow left" or msg.data == "follow right" or msg.data == "stop follow"
            self.wall_pub.publish(msg)

    def laser_callback(self, msg):
        dis = msg.ranges[540]
	if dis < 0.05:
            drive_msg = AckermannDriveStamped(self.header, AckermannDrive(speed=0.0, steering_angle=0.0))
	    self.drive_pub.publish(drive_msg)

    #Optional function to start programs using the joystick
    #def joy_buttons(self, msg):


if __name__=="__main__":
    rospy.init_node("MainControl")
    node = Control()
    rospy.spin()
