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

        # not_object_position is where the car can move to 
        not_object_position = "center"

        # isObject tells if there is an object in front
        isObject = False
        # placeholders for isLeft and isRight, which tell where the object is
        isMaybeLeft = False
        isMaybeRight = False

        isLeft = False
        isRight = False

        # mutable lists for the ranges on each side so I can remove any flukes in the laser scan data
        right_side_list = msg.ranges[1:359]
        left_side_list = msg.ranges[721:1079]

        # these aren't real indexes in the above lists, these are just used so the below function doesn't accidentally remove real values
        right_wall_index = 359
        left_wall_index = 359

        # crazy initial values to ensure that the program will enter the while loop.
        right_wall_dist = 0
        left_wall_dist = 0

        # Okay this is kind of crazy... I'm  checking if the distances below and above the found min are close to the min distance, which woud indicate whether or not it is an anomaly or not. If it is a fluke, the while loop repeats and gives that value the max distance the laser scan data can handle, thus making it unlikely to be the minagain.
        while abs(msg.ranges[right_wall_index + 1] - right_wall_dist) > 0.1 and abs(msg.ranges[right_wall_index - 1] - right_wall_dist) > 0.1:
            right_side_list[right_wall_index] = 10.0
            right_wall_index, right_wall_dist = min(enumerate(right_side_list), key=operator.itemgetter(1))
        while abs(msg.ranges[left_wall_index + 1] - left_wall_dist) > 0.1 and abs(msg.ranges[left_wall_index - 1] - left_wall_dist) > 0.1:
            left_side_list[left_wall_index] = 10.0
            left_wall_index, left_wall_dist = min(enumerate(left_side_list), key=operator.itemgetter(1))

        print(right_wall_index, left_wall_index)

        # This for loop runs through every laser scan data 45 degrees left and right of straight ahead.
        for i in range((180+right_wall_index),(540+right_wall_index)):

            potential_object_dist = msg.ranges[i]

            # capping the range of the laser scan data
            if potential_object_dist > 10.0:
                potential_object_dist = 10.0

            # okay this actually is a little bit flawed, because it is based on local perception, not global, but it finds the straight ahead angle, and sees if the object is to the left or right based on which side the object is on.
            if 360 > i - right_wall_index:
                # tells that the MAYBE object is right and that is not left
                isMaybeRight = True
                isMaybeLeft = False
                # trig function to find the hypotenuse distance to the wall based on perpendicular distance to the wall and the angle it's at.
                supposed_wall_dist = right_wall_dist/(math.cos(math.radians((i - right_wall_index)/4)))
            else:
                isMaybeLeft = True
                isMayberight = False
                supposed_wall_dist = left_wall_dist/(math.cos(math.radians((left_wall_index - i)/4)))

            # capping the trig function as well, because it could become infinite at the straight forward point. Oh shoot, maybe we need to throw an exception for that point...
            if supposed_wall_dist > 10.0:
                supposed_wall_dist = 10.0

            # this tests to see if the captured distance is less than the distance to the wall, meaning there is an object in the way. Gives some error room as well.
            if potential_object_dist < supposed_wall_dist - 1:
               
                # what do you know? There's an object because of that!
                isObject = True
                # captures the temporary MAYBE booleans if there's an actual object
                if isMaybeRight:
                    isRight = True
                if isMaybeLeft:
                    isLeft = True
                # this will get mixed up a bit, but by the end of the for loop, it will show the correct not_object_position I think.
                if isLeft:
                    not_object_position = "right"
                elif isRight:
                    not_object_position = "left"
                elif isLeft and isRight:
                    not_object_position = "center"
                else:
                    not_object_position = "center"
      
        # called a new function because why the fuck not
        if isObject:
            self.warnRobot(not_object_position)   
    
    # made a new function because why the fuck not
    def warnRobot(self, not_object_position):

        # publishes the string that contains the lane information.
        self.pub_lane.publish(not_object_position)
      
            
if __name__ == "__main__":
    rospy.init_node("obstacle_detection_node")
    node = ObstacleDetectionNode()
    rospy.spin()
