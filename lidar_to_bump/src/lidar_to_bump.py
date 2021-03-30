#!/usr/bin/env python

# Author: Trevor Sherrard
# Course: Motion Planning
# Project: Shared Autonomy Telenursing Robot
# Since: 03/30/2021
# 
# Description: This file contains the ROS node that will 
#              look for "bumped" obstacles within the enviornment
#              using the LiDAR data, and will return a vector representing
#              the direction of the "bump"

import rospy
from sensor_msgs.msg import LaserScan

# misc variables
laserscan_topic = "/trina2_1/base_scan"
bump_tresh = 0.5 # distanced to be considered a bump hazard
min_bumps = 10   # minimum number of consecutive close points to be considered a bump

def proc_laserscan(msg):
    """
    this function serves as the callback for the laserscan subscriber. It will attempt
    to find "bumps" and locate the direction of said bump.
    """
    start_angle = float(msg.angle_min)
    end_angle = float(msg.angle_max)
    angle_inc = float(msg.angle_increment)
    points = msg.ranges

    # loop over points 
    num_consecutive_bumps = 0
    last_point_bump = False
    for point in points:
        # increment current angle
        start_angle += angle_inc

        # if current point is below threshold, set flags and increment
        # number of bumps
        if(point < bump_thresh):
            last_point_bump = True
            num_consecutive_bumps += 1
        
        # otherwise, set flags and reset number of bumps
        else:
            last_point_bump = False
            num_consecutive_bumps = 0
        


def main():
    # init node
    rospy.init_node("lidar_to_bump")
    rospy.loginfo("lidar_to_bump node initialized")

    # create subscriber
    laser_sub = rospy.Subscriber(laserscan_topic, LaserScan, proc_laserscan)
    rospy.spin()

if(__name__ == "__main__"):
    main()
