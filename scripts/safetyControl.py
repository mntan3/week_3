#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class Safety:
    def __init__(self):
        rospy.Subsciber('/scan', LaserScan, self.laser_callback, queue_size=10)

	self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 1)
        
        self.header = std_msgs.msg.Header()
    def laser_callback(self, msg):
        dis = msg.ranges[540]
        if dis < 0.05:
            drive_msg = AckermannDriveStamped(self.header, AckermannDrive(speed=0.0, steering_angle=0.0))
            self.drive_pub.publish(drive_msg)

if __name__ == "__main__":
    rospy.init_node("safety")
    node = Safety()
    rospy.spin()

