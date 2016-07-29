#! /usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

class SafetyControllerNode:
    def __init__(self):
        rospy.Subscriber("/scan", LaserScan, self.laser_callback, queue_size=10)

        self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=10)

    def laser_callback(self, msg):
        center = len(msg.ranges) / 2
        avg = 0
        for i in range(0, 10):
            avg += msg.ranges[center - 5 + i]
        avg /= 10

        drivemsg = AckermannDriveStamped()
        #drivemsg.drive.steering_angle = 0
        if avg < 0.5:
            drivemsg.drive.speed = -0.5
            self.pub.publish(drivemsg)
	    rospy.loginfo("center range(s): %f", avg)

if __name__ == "__main__":
    rospy.init_node("safety_controller_py")
    node = SafetyControllerNode()
    rospy.spin()


