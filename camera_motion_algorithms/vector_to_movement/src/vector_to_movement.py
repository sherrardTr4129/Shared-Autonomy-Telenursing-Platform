#!/usr/bin/env python

## Author: Trevor Sherrard
## Course: Directed Research
## Project: telepresence telenursing robot
## Since: 01/30/2021

## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_srvs.srv import Empty
from moveit_commander.conversions import pose_to_list
from telenursing_trina2_ctrl.msg import joint_angle
from telenursing_trina2_ctrl.srv import homeRobotLeftArmReq, homeRobotLeftArmReqResponse
from std_msgs.msg import Float32, Float64
from sensor_msgs.msg import Joy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
global interfaceInstance

# Misc variables
JOY_START_INDEX = 2
JOY_STOP_INDEX = 8

def startNode():

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    #moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('roll_reduction', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    group1_name = "left_arm"
    group2_name = "right_arm"
    move_group1 = moveit_commander.MoveGroupCommander(group1_name)
    move_group2 = moveit_commander.MoveGroupCommander(group2_name)
    current_pose = move_group1.get_current_pose().pose

    ox = current_pose.orientation.x
    oy = current_pose.orientation.y
    oz = current_pose.orientation.z
    ow = current_pose.orientation.w
    (r, p, y) = euler_from_quaternion([ox, oy, oz, ow])
    roll = r
    pitch = p
    yaw = y
    x =current_pose.position.x
    y= current_pose.position.y
    z= current_pose.position.z


if __name__ == "__main__":
    startNode()
    rospy.spin()