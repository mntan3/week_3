#!/usr/bin/env python

import numpy as np
import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import *
from ackermann_msgs.msg import *

class WinterDodgeNode:
    def __init__(self):
        self.node_name = "winter_dodge_node"
        self.K_energy = -0.12
        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 1)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.header = std_msgs.msg.Header()

    def laser_callback(self, msg):

        x_force_total = -self.K_energy / 0.025**2
        y_force_total = 0
        for i in range(180, 900):
            angle = math.radians((i - 540)/4)
            dist = msg.ranges[i]
            
            force = self.K_energy / dist**2

            x_force = math.cos(angle) * force
            y_force = math.sin(angle) * force

            x_force_total = x_force_total + x_force
            y_force_total = y_force_total + y_force

        speed = 0.005 * math.sqrt(x_force_total**2 + y_force_total**2) * np.sign(x_force_total)
        steering_angle = 1.0 * math.atan2(y_force_total, x_force_total) * np.sign(x_force_total)
        print(x_force_total, y_force_total, speed, steering_angle)
        self.drive_pub.publish(AckermannDriveStamped(self.header, AckermannDrive(speed = speed, steering_angle = steering_angle)))

if __name__ == "__main__":
    rospy.init_node("winter_dodge_node")
    node = WinterDodgeNode()
    rospy.spin()
