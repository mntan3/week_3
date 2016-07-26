#!/usr/bin/env python

import rospy
from sensor_msgs.msg import *
from std_msgs.msg import *
import operator
import math

class ObstacleDetectionNode:
    def __init__(self):
        self.node_name = "obstacle_detection_node"
        self.header = std_msgs.msg.Header()
        self.isObject = False

        # object position 0 is left, 1 is middle, 2 is right
        self.objectPosition = 0

        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.laser_cb)
        #self.pub_lane = rospy.Publisher("/topicName", Int, queue_size = 1)

        self.loginfo("initialized")

    def laser_cb(self, msg):
        right_wall_index, right_wall_dist = min(enumerate(msg.ranges[0:360]), key=operator.itemgetter(1))
        left_wall_index, left_wall_dist = min(enumerate(msg.ranges[720:1080]), key=operator.itemgetter(1))
        for i in range(360,720):
            isLeft = True
            potential_object_dist = msg.ranges[i]
            #if potential_object_dist > 10.0:
                #potential_object_dist = 10.0
            if 360 > i - right_wall_index:
                isLeft = False
                supposed_wall_dist = right_wall_dist/(math.cos(math.radians(4 *(i - right_wall_index))))
                if potential_object_dist < right_wall_dist/math.cos(math.radians(
            else:
                isLeft = True
                supposed_wall_dist = left_wall_dist/(math.cos(math.radians(4 * (left_wall_index - i))))
            if supposed_wall_dist > 10.0:
                supposed_wall_dist = 10.0
            if potential_object_dist < supposed_wall_dist:
                self.isObject = True
                if isLeft:
                    self.object_position = 0
                elif not isLeft:
                    self.object_position = 2
        if isObject:
            warnRobot(self.object_position)   
        
    def warnRobot(self, object_position):
        #publish message for wallfollower.py to pick up                
            
            
if __name__ == "__main__":
    rospy.init_node("obstacle_detection_node")
    node = ObstacleDetectionNode()
    rospy.spin()
