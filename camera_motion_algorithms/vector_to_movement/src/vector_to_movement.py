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

# STATE VARIABLES
# sim starts with left arm as the secondary camera by default
sec_cam = "leftArm" # "head" for main_cam, "leftArm" for left_arm movement

# default starting positions of the head camera, keeping track of it here as 
# it gives same info as subscribing to the controller topic (it just has setpoint not actual)
global headcam_pitch
global headcam_yaw

# move group definitions (should be unchanging)
## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
## kinematic model and the robot's current joint states   
group1_name = "left_arm"
#group2_name = "right_arm"
move_group1 = moveit_commander.MoveGroupCommander(group1_name)

global headcam_pitch_pub
global headcam_yaw_pub

def startNode():
    global headcam_pitch_pub
    global headcam_yaw_pub
    global headcam_pitch
    global headcam_yaw


    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    #moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('roll_reduction', anonymous=True)

    # initialize head_cam pitch and yaw positions
    headcam_pitch = 0.0
    headcam_yaw = 0.0


    # subscribe to secondary cam topic to check which camera must be moved
    #rospy.Subscriber("/secondaryCam", String, which_cam_callback, queue_size=1)
    # publish the yaw and pitch to the main head camera directly (no movegroup)
    headcam_yaw_pub = rospy.Publisher('/trina2_1/main_cam_yaw_controller/command', Float64,queue_size=1)
    headcam_pitch_pub = rospy.Publisher('/trina2_1/main_cam_pitch_controller/command', Float64, queue_size=10)


def which_cam_callback(primary):
    # primary = std_msgs/String
    sec_cam = primary.data

# Takes in a 3d vector, moves the secondary camera incrementally
def vec_to_move(vector):
    global headcam_pitch_pub
    global headcam_yaw_pub
    global headcam_pitch
    global headcam_yaw


    # determine xyz of vector, change format based on datatype of "vector"
    vecX = vector[0] # currently assuming vector is a 1x3 array
    vecY = vector[1]
    vecZ = vector[2]

    # since only the left arm uses a movegroup for the cameras, movegroup will always be move_group1
    move_group = move_group1
    if (sec_cam == "left_arm"):

        #current_move_group = move_group1
        current_pose = move_group.get_current_pose().pose

        x = current_pose.position.x
        y = current_pose.position.y
        z = current_pose.position.z
        ox = current_pose.orientation.x
        oy = current_pose.orientation.y
        oz = current_pose.orientation.z
        ow = current_pose.orientation.w
        (r, p, y) = euler_from_quaternion([ox, oy, oz, ow])
        roll = r
        pitch = p
        yaw = y

        # maintain the same orientation to begin with
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = ow
        pose_goal.orientation.x = ox
        pose_goal.orientation.y = oy
        pose_goal.orientation.z = oz

        # increment the motion for the goal based on the input vector
        pose_goal.position.x = y + vecX * 0.05
        pose_goal.position.y = x + vecY * 0.05
        pose_goal.position.z = z + vecZ * 0.05

        # execute the planned motion
        move_group.set_pose_target(pose_goal)
        move_group.set_goal_tolerance = 0.08
        
        
        while True:
            plan = move_group.go(wait=True)
            current_pose = move_group.get_current_pose().pose
            if all_close(pose_goal, current_pose, 0.05):
                break

        # Stop all movement and clear targets
        move_group.stop()
        move_group.clear_pose_targets()

    else:
        # publish to "/trina2_1/main_cam_yaw_controller/command" (type Float64) for yaw control
        # publish to "/trina2_1/main_cam_pitch_controller/command" (type Float64) for pitch control
        print("head_cam is selected")

        # set up the control messages (starting at current pitch and yaw)
        pitch_msg = Float64()
        pitch_msg.data = headcam_pitch
        yaw_msg = Float64()
        yaw_msg.data = headcam_yaw

        # move in direction of the vector in small steps
        if vecY > 0: 
            yaw_msg.data += 0.1
            headcam_yaw += 0.1
        elif vecY < 0:
            yaw_msg.data -= 0.1
            headcam_yaw -= 0.1
        if vecZ > 0:
            pitch_msg.data += 0.1
            headcam_pitch += 0.1
        elif vecZ < 0:
            pitch_msg.data -= 0.1
            headcam_pitch -= 0.1

        headcam_pitch_pub.publish(pitch_msg)
        headcam_yaw_pub.publish(yaw_msg)



if __name__ == "__main__":
    global headcam_pitch_pub
    global headcam_yaw_pub

    startNode()

    move_vector = [1,1,1]

    #vec_to_move([5, 5, 5])
    # rostopic pub /trina2_1/main_cam_pitch_controller/command std_msgs/Float64 20
    # pitch_msg = Float64()
    # pitch_msg.data = 0.0
    # yaw_msg = Float64()
    # yaw_msg.data = 0.0
    # # headcam_pitch_pub.publish(pitch_msg)
    # rospy.loginfo(pitch_msg)
    # rospy.loginfo(" we are publishing pitch")
    #rospy.spin()
    while not rospy.is_shutdown():
        
        vec_to_move(move_vector)
        
        # if(pitch_msg.data < 60.0):
        #     pitch_msg.data += 0.1
        # if(yaw_msg.data < 60.0):
        #     yaw_msg.data += 0.1

        # #headcam_pitch_pub.publish(pitch_msg)
        # headcam_yaw_pub.publish(yaw_msg)
        # #rospy.loginfo(pitch_msg)
        # rospy.loginfo(yaw_msg)
        # #rospy.loginfo(" we are publishing pitch")
        # rospy.loginfo(" we are publishing yaw")

