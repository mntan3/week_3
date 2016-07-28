#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import *

class Safety:
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=10)

	self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size = 1)
        
        self.header = std_msgs.msg.Header()
    def laser_callback(self, msg):
        fwd_scan = msg.ranges[509:570] 
	dist = min(fwd_scan)
        if dist < 0.5:
            drive_msg = AckermannDriveStamped(self.header, AckermannDrive(speed=0.0, steering_angle=0.0))
            self.drive_pub.publish(drive_msg)
	elif dist < 0.2:
	    drive_msg2 = AckermannDriveStamped(self.header, AckermannDrive(speed=1.0, steering_angle=0.0))
	    self.drive_pub.publish(drive_msg2)

if __name__ == "__main__":
    rospy.init_node("safety")
    node = Safety()
    rospy.spin()

