#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import std_msgs
import math

PID_KP_LEFT = 0.9
PID_KP_RIGHT = 1.3

PID_KD = 0

class wall_follow:
    def __init__(self):
        # don't forget to define the node name ya goose
        rospy.Subscriber('/scan', LaserScan, self.simulate_callback, queue_size=10)
        rospy.Subscriber('/wallfollow'¸ String, self.enable)
	self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 1)

        self.last_error = None
        self.STOP = AckermannDriveStamped(self.header, AckermannDrive(steering_angle=0.0, speed    =0.0))

        self.followState = 0    #0 is stop, 1 is follow left, 2 is follow right
        
        self.desired = 0.45
    def enable(self.msg):
        if msg.data == "follow left":
            self.followState = 1
        elif msg.data == "follow right":
            self.followState = 2
        else msg.data == "stop follow"
            self.followState = 0
    def calc_actual_dist(self, ranges):
        if self.followState == 1:
            end_index = 900
            start_index = 850
        else: # follow right
            end_index = 1081 - 850
            start_index = 1081 - 900

        angle_degrees = (270.0 / 1081.0) * (end_index - start_index)
        r1 = ranges[start_index] # looking forward-left
        r2 = ranges[end_index] # looking left

        dist = (r1 * r2 * math.sin(math.radians(angle_degrees)))
        dist /= math.sqrt(r1**2 + r2**2 - 2*r1*r2*math.cos(math.radians(angle_degrees)))
        return dist
        
               
    def simulate_callback(self, msg): # what is the point of the "publisher" argument? # Jk I figured it out, pretty sneaky
        actual_dist = self.calc_actual_dist(msg.ranges)
        # rospy.loginfo("The actual distance: %f", actual_dist)
        # rospy.loginfo("Direction: %d", self.follow_left)


        error = self.desired - actual_dist
        
        if self.last_error != None:
            deriv = (error - self.last_error) / msg.scan_time
        else:
            deriv = 0

        sign = 1
        if self.followState == 1:
            sign = -1

        if self.followState == 1:
            pid_kp = PID_KP_LEFT
        else:
            pid_kp = PID_KP_RIGHT
        steer_output = (sign * pid_kp * error) + (sign * PID_KD * deriv)

        if steer_output > 0.4:
            steer_output = 0.4
        elif steer_output < -0.4:
            steer_output = -0.4

        # rospy.loginfo("steering is %f", steer_output)
        if self.followState != 0:
            drive_msg = AckermannDriveStamped()
            # I think you also need to define the "header"
            drive_msg.drive.speed = 2 # max speed
            drive_msg.drive.steering_angle = steer_output
	    self.drive_pub.publish(drive_msg)
        else:
            self.drive_pub.publish(self.STOP)

        self.last_error = error

if __name__ == '__main__':
    rospy.init_node('wall_follower')
    node = wall_follow()
    rospy.spin()
