#!/usr/bin/env/python

import rospy
from sensor_msgs.msg import *
from ackermann_msgs.msg import *
from std_msgs.msg import *

class FindEmptySpace:
	def __init__(self):
		self.node_name = "find_empty_space"
		self.header = std_msgs.msg.Header()
		self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.empty_space)
		self.pub_drive = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 1)

		rospy.loginfo("Initialized")
	def empty_space(self, msg):
		drivemsg = AckermannDriveStamped(self.header, AckermannDrive(speed=0.0, steering_angle=0.0))
		drivemsg.header = self.header
		drivemsg.drive.speed = 0.7
		Pd = 1.0
		#Scan 120 deg in front (from 300 to 780)
		pts = msg.ranges[420:660]
		max_index = pts.index(max(pts)) + 420
		max_center = 540
		
		error_index = max_index - max_center
		error_index = float(error_index)
		
		steer_output = (error_index / 120) * Pd 
		print(max_index)
		print(steer_output)
		
		drivemsg.drive.steering_angle = steer_output
		self.pub_drive.publish(drivemsg)
if __name__ == '__main__':
	rospy.init_node('find_empty_space')
	node = FindEmptySpace()
	rospy.spin()
 
		
			
			
