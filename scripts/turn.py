#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import *
from std_msgs.msg import *

#Use tabs
#Turns approximately 90 degrees either left or right
#Publish onto /turn either "turn right" or "turn left" as a string to run program

class Turn:
	def __init__(self):
		self.direction = 0		#0 is left, 1 is right
		rospy.Subscriber("/turn", String, self.setSide)
		rospy.Publisher("/transition", String, queue_size=1)
		self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 1)
		self.turn_counter = 0
		self.header = std_msgs.msg.Header()

	def setSide(self, msg):
		if msg.data == "turn right":
			self.direction = 1
		elif msg.data == "turn left":
			self.direction = 0
		self.turn_timer = rospy.Timer(rospy.Duration(.2), self.turn)

	def turn(self, _):
		if self.direction == 0:
			drive_msg = AckermannDriveStamped(self.header, AckermannDrive(speed=1.5, steering_angle=0.4))
			self.drive_pub.publish(drive_msg)
			#turn left, (1.5, .4)
		else:
			drive_msg = AckermannDriveStamped(self.header, AckermannDrive(speed=1.0,steering_angle=-0.35))
			self.drive_pub.publish(drive_msg)
			#turn right, (1.0, -0.35)
		self.turn_counter += 1
		if self.turn_counter > 6:
			self.turn_timer.shutdown()
		self.turn_counter = 0
if __name__ == "__main__":
	rospy.init_node("Turn")
	node = Turn()
	rospy.spin()
