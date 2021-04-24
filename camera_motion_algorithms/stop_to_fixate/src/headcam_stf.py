#!/usr/bin/env python

# Author: Albert Enyedy
# Course: Motion Planning
# Project: Shared Autonomy Telenursing Robot
# Since: 04/06/2021
# 
# Description: This file contains the ROS node that will 
#              perform the stop-to-fixate algorithm for 
#              the head pan-tilt camera based on the Float64 msg
#              sent by lidar_to_bump ("/trina2_1/bump_angle")
#              -> returns a 3D vector in which y represents 
#                 the yaw in radians and z = pitch in radians

import rospy
import math
from math import pi
from math import atan2
import copy
import moveit_commander
from moveit_commander.conversions import pose_to_list
from telenursing_trina2_ctrl.msg import joint_angle
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3 
from std_msgs.msg import Float64

# callback for bump angle
def proc_bump_angle(data):
    rospy.loginfo("angle received")

    # determine pitch using height of robot and distance from wall
    # we assume wall distance is 0.6 based on lidar to bump bump_thresh(could be less)
    robot_height = 1.2 #m
    dist_to_bump = 0.6 #m
    # calculate pitch using negative robot height due to camera must look downwards
    rad_from_straight_down = atan2(dist_to_bump/(-1*robot_height))
    pitch = pi/2 - rad_from_straight_down
    
    # convert the yaw data to radians as well
    yaw_deg = data.data # is in degrees currently
    yaw = yaw_deg*pi/180

    # set the camera pitch and yaw joint angles to the pitch and yaw
    headcam_ctrl = Vector3()
    headcam_ctrl.y = yaw
    headcam_ctrl.z = pitch
    vec_pub.publish(headcam_ctrl)

def main():
    # init node
    rospy.init_node("headcam_stf")
    rospy.loginfo("headcam_stf node initialized")

    # create subscriber
    angle_sub = rospy.Subscriber("/trina2_2/bump_angle", Float64, proc_bump_angle)
    vec_pub = rospy.Publisher("/stop_to_fixate_vec", Vector3, queue_size=1)
    rospy.spin()

if(__name__ == "__main__"):
    main()
