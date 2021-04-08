#!/usr/bin/env python

# Author: Albert Enyedy
# Course: Motion Planning
# Project: Shared Autonomy Telenursing Robot
# Since: 04/08/2021
# 
# Description: This file contains the ROS node that will 
#              move the arm or pan-tilt camera based on 
#              the 3D vector output by viewpoint selection

import rospy
import math
import sys
from math import pi
import copy
import moveit_commander
from moveit_commander.conversions import pose_to_list
#from telenursing_trina2_ctrl.msg import joint_angle
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64

# callback for bump angle
# def proc_bump_angle(data):
#     rospy.loginfo("angle received")

#     # must calculate the pitch angle based on range from lidar from lidar_to_bump
#     pitch = 0.0
#     yaw = data.data

    # set the camera pitch and yaw joint angles to the pitch and yaw

# this class sets up the node for the secondary viewpoint controller
class AutoViewMotionControl(object):
    
    def __init__(self):
        super(AutoViewMotionControl, self).__init__()

        # initialize the moveit_commander and node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('auto_viewpoint_controller', anonymous=True)

        # instantiate robot commander (name diff to avoid namespace conflicts?) 
        robot_auto = moveit_commander.RobotCommander()

        # instantiate movegroup commander
        group3_name = "auto_view_control"
        move_group3 = moveit_commander.MoveGroupCommander(group3_name)
        
# def main():
#     # init node
#     rospy.init_node("headcam_stf")
#     rospy.loginfo("headcam_stf node initialized")

#     # create subscriber
#     angle_sub = rospy.Subscriber("/trina2_2/bump_angle", Float64, proc_bump_angle)
#     rospy.spin()

if(__name__ == "__main__"):
    controller = AutoViewMotionControl()
    rate = rospy.Rate(10)
    
