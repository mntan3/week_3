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

        self.sub_scan = rospy.Subscriber("/scan", LaserScan, self.laser_cb)
        self.pub_lane = rospy.Publisher("/transition", String, queue_size = 1)

        rospy.loginfo("initialized")

    def laser_cb(self, msg):
        not_object_position = "center"
        isObject = False
        isMaybeLeft = False
        isMaybeRight = False
        isLeft = False
        isRight = False
        right_wall_index, right_wall_dist = min(enumerate(msg.ranges[0:360]), key=operator.itemgetter(1))
        left_wall_index, left_wall_dist = min(enumerate(msg.ranges[720:1080]), key=operator.itemgetter(1))
        print(right_wall_index, left_wall_index)
        for i in range((180+right_wall_index),(540+right_wall_index)):

            potential_object_dist = msg.ranges[i]

            if potential_object_dist > 10.0:
                potential_object_dist = 10.0

            if 360 > i - right_wall_index:
                isMaybeRight = True
                supposed_wall_dist = right_wall_dist/(math.cos(math.radians((i - right_wall_index)/4)))
            else:
                isMaybeLeft = True
                supposed_wall_dist = left_wall_dist/(math.cos(math.radians((left_wall_index - i)/4)))

            if supposed_wall_dist > 10.0:
                supposed_wall_dist = 10.0

            if potential_object_dist < supposed_wall_dist - 0.25:
                isObject = True
                if isMaybeRight:
                    isRight = True
                if isMaybeLeft:
                    isLeft = True

                if isLeft:
                    not_object_position = "right"
                elif isRight:
                    not_object_position = "left"
                elif isLeft and isRight:
                    not_object_position = "center"
                else:
                    not_object_position = "center"

        if isObject:
            self.warnRobot(not_object_position)   
        
    def warnRobot(self, not_object_position):
        self.pub_lane.publish(not_object_position)
      
            
if __name__ == "__main__":
    rospy.init_node("obstacle_detection_node")
    node = ObstacleDetectionNode()
    rospy.spin()
