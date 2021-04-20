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
#              -> returns a 3D vector or PoseStamped (will see which is better)

import rospy
import math
from math import pi
import copy
import moveit_commander
from moveit_commander.conversions import pose_to_list
from telenursing_trina2_ctrl.msg import joint_angle
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64

# callback for bump angle
def proc_bump_angle(data):
    rospy.loginfo("angle received")

    # must calculate the pitch angle based on range from lidar from lidar_to_bump
    pitch = 0.0
    yaw = data.data

    # set the camera pitch and yaw joint angles to the pitch and yaw
    

def main():
    # init node
    rospy.init_node("headcam_stf")
    rospy.loginfo("headcam_stf node initialized")

    # create subscriber
    angle_sub = rospy.Subscriber("/trina2_2/bump_angle", Float64, proc_bump_angle)
    #vec_pub = rospy.Publisher("/stop_to_fixate_vec", )
    rospy.spin()

if(__name__ == "__main__"):
    main()
