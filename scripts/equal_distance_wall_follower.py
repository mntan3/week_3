#!/usr/bin/env python

import rospy
from sensor_msgs.msg import *
from ackermann_msgs.msg import *
from std_msgs.msg import *

class EqualDistanceWallFollower:
    def __init__(self):
        self.node_name = "equal_distance_wall_follower"
	self.header = std_msgs.msg.Header()
        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.drive_cb)
        self.pub_drive = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 1)

        rospy.loginfo("initialized")

    def drive_cb(self, msg):
	drivemsg = AckermannDriveStamped(self.header, AckermannDrive(speed=0.0, steering_angle=0.0))
        drivemsg.header = self.header
        drivemsg.drive.speed = 1.0

        left_wall_distance = msg.ranges[720] # average probably
	right_wall_distance = msg.ranges[360] # average proabably

        K_prop = 0.1
        K_int = 0
        K_deriv = 0

        error = left_wall_distance - right_wall_distance

        drivemsg.drive.steering_angle = K_prop * error

        self.pub_drive.publish(drivemsg)

if __name__ == "__main__":
    rospy.init_node("equal_distance_wall_follower")
    node = EqualDistanceWallFollower()
    rospy.spin()
