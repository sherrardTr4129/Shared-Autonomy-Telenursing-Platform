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
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

# misc variables
laserscan_topic = "/trina2_1/base_scan"
angle_topic = "/trina2_1/bump_angle"
bump_thresh = 0.6 # distanced to be considered a bump hazard
min_bumps = 10   # minimum number of consecutive close points to be considered a bump

# create publisher for angle estimate
angle_pub = rospy.Publisher(angle_topic, Float64, queue_size=5)

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
    total_angle_accum = 0
    for point in points:
        # increment current angle
        start_angle += angle_inc

        # if current point is below threshold, set flags and increment
        # number of bumps
        if(point < bump_thresh):
            last_point_bump = True
            num_consecutive_bumps += 1
            total_angle_accum += start_angle

            # if we have gone over minimum number of 
            # consecutive bump points, estimate angle towards
            # bump from pespective of robot.
            if(num_consecutive_bumps > min_bumps):
                angle_est_rad = total_angle_accum / num_consecutive_bumps
                angle_est_deg = angle_est_rad * (180/math.pi)

                # create new float message
                angle_msg = Float64()
                angle_msg.data = angle_est_deg

                # publish new message
                angle_pub.publish(angle_msg)
        
        # otherwise, set flags and reset number of bumps
        else:
            last_point_bump = False
            num_consecutive_bumps = 0
            total_angle_accum = 0

def main():
    # init node
    rospy.init_node("lidar_to_bump")
    rospy.loginfo("lidar_to_bump node initialized")

    # create subscriber
    laser_sub = rospy.Subscriber(laserscan_topic, LaserScan, proc_laserscan)
    rospy.spin()

if(__name__ == "__main__"):
    main()
