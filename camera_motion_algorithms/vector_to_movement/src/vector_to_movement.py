#!/usr/bin/env python

## Authors: Phillip Abell and Albert Enyedy
## Course: RBE 550 Motion Planning
## Project: telepresence telenursing robot
## Since: 4/6/2021

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
from geometry_msgs.msg import PoseStamped
from moveit_commander.conversions import pose_to_list
from telenursing_trina2_ctrl.msg import joint_angle
from telenursing_trina2_ctrl.srv import homeRobotLeftArmReq, homeRobotLeftArmReqResponse
from std_msgs.msg import Float32, Float64
from sensor_msgs.msg import Joy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Vector3 
global interfaceInstance

# Misc variables
JOY_START_INDEX = 2
JOY_STOP_INDEX = 8

# STATE VARIABLES
# sim starts with left arm as the secondary camera by default
global sec_cam

# default starting positions of the head camera, keeping track of it here as 
# it gives same info as subscribing to the controller topic (it just has setpoint not actual)
global headcam_pitch
global headcam_yaw


# move group definition for left arm
global move_group1
group1_name = "left_arm"

# publishers for the head camera yaw and pitch control
global headcam_pitch_pub
global headcam_yaw_pub

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if (abs(actual[index] - goal[index]) > tolerance) and \
               (abs(abs(actual[index]-goal[index])-2*pi) > tolerance) :
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

def startNode():
    """
    Instantiate the node, its subscribers and publishers, and global variables such as the moveit commander
    """
    global headcam_pitch_pub
    global headcam_yaw_pub
    global headcam_pitch
    global headcam_yaw
    global sec_cam
    global move_group1
    global switchCam
    global listener

    # instantiate the node
    rospy.init_node('roll_reduction', anonymous=True)

    # initialize head_cam pitch and yaw positions to match the default starting ones
    headcam_pitch = 0.0
    headcam_yaw = 0.0
    # default secondary cam starts as left arm, values are "head" for main_cam, "leftArm" for left_arm movement
    sec_cam = "leftArm" 
    # set up moveit commander for left arm
    move_group1 = moveit_commander.MoveGroupCommander("left_arm")
    # set up transform listener for pose transformations
    listener = tf.TransformListener()

    # subscribe to secondary cam topic to check which camera must be moved
    rospy.Subscriber("/secondaryCam", String, which_cam_callback, queue_size=1)
    # publisher for overriding primary camera for stop-to-fixate interrupt
    switchCam = rospy.Publisher("/secondaryCam", String, queue_size=1)

    # publish the yaw and pitch to the main head camera directly (no movegroup)
    headcam_yaw_pub = rospy.Publisher('/trina2_1/main_cam_yaw_controller/command', Float64,queue_size=1)
    headcam_pitch_pub = rospy.Publisher('/trina2_1/main_cam_pitch_controller/command', Float64, queue_size=10)

    # set up subscribers for the weighted viewpoint selection algorithm vector and stop-to-fixate interrupt vector
    rospy.Subscriber("/info_and_saliency_avg_pose", PoseStamped, vec_to_move, queue_size=1)
    rospy.Subscriber("/stop_to_fixate_vec", Vector3, stop_and_fixate, queue_size=1)

def which_cam_callback(primary):
    """
    callback to update state variable of which camera is secondary camera
    """
    global sec_cam
    # primary's datatype is std_msgs/String
    sec_cam = primary.data

def stop_and_fixate(vector):
    """
    Callback for achieving stop-to-fixate behavior.
    Only moves head camera via interrupt when bump is detected
    """
    global headcam_pitch_pub
    global headcam_yaw_pub
    global headcam_pitch
    global headcam_yaw

    # set up messages for publishing
    pitch_msg = Float64()
    yaw_msg = Float64()

    # start with current head camera pitch and yaw
    pitch_msg.data = headcam_pitch
    yaw_msg.data = headcam_yaw

    # only move head if goal is > pi/6 radians difference from current head camera pitch and yaw
    if (abs(vector.y - headcam_yaw) > 3.14 / 6):
        # update the head camera yaw in memory, then set new yaw for publishing
        headcam_yaw = vector.y
        yaw_msg.data = vector.y
    if (abs(vector.z - headcam_pitch) > 3.14 / 6):
        # update the head camera pitch in memory, then set new pitch for publishing
        headcam_pitch = vector.z
        pitch_msg.data = vector.z

    # publish the pitch and yaw to control the head camera directly with quick motion
    headcam_pitch_pub.publish(pitch_msg)
    headcam_yaw_pub.publish(yaw_msg)

    # switch left arm to secondary camera, thus having teleoperator instantly see the obstacle after motion is complete
    switchCam.publish("leftArm")

def vec_to_move(vector):
    """
    Callback from weighted viewpoint selection vector subscriber
    Takes in the weighted 3d vector, moves the secondary camera incrementally towards the viewpoint
    """
    global headcam_pitch_pub
    global headcam_yaw_pub
    global headcam_pitch
    global headcam_yaw
    global sec_cam
    global move_group1

    # set goal position
    x_goal = vector.pose.position.x
    y_goal = vector.pose.position.y
    z_goal = vector.pose.position.z
    
    # set goal orientation
    o_x_goal = vector.pose.orientation.x
    o_y_goal = vector.pose.orientation.y
    o_z_goal = vector.pose.orientation.z
    o_w_goal = vector.pose.orientation.w

    # since only the left arm uses a movegroup for the cameras, movegroup will always be move_group1
    move_group = move_group1
    # if left arm is secondary camera, use moveit commander to move the arm towards the vector
    if (sec_cam == "leftArm"):
        # get current pose of arm to generate incremental move goals
        current_pose = move_group.get_current_pose()
        # transform pose to correct frame id
        current_pose = listener.transformPose('/map', move_group.get_current_pose()).pose
        # set current pose of arm in correct frame id
        x = current_pose.position.x
        y = current_pose.position.y
        z = current_pose.position.z
        ox = current_pose.orientation.x
        oy = current_pose.orientation.y
        oz = current_pose.orientation.z
        ow = current_pose.orientation.w
        
        # convert current pose from quaternion to roll pitch yaw
        (rollRe, pitchRe, yawRe) = euler_from_quaternion([ox, oy, oz, ow])
        roll = rollRe
        pitch = pitchRe
        yaw = yawRe

        # increment the motion for the goal based on the input vector, using small scale
        movescale = 0.05 

        # maintain the same orientation to begin with
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = ow
        pose_goal.orientation.x = ox
        pose_goal.orientation.y = oy
        pose_goal.orientation.z = oz

        # swapping of x and y reflects how it works in webgui
        pose_goal.position.x = x - (x_goal * movescale)
        pose_goal.position.y = y - (y_goal * movescale)
        pose_goal.position.z = z - (z_goal * movescale)

        # execute the planned motion
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = '/map'
        ps.pose = pose_goal
        ps = listener.transformPose('/trina2_1/base_link', ps)
        pose_goal = ps.pose
        move_group.set_pose_target(pose_goal)
        move_group.set_goal_tolerance = 0.08
        
        # move incrementally in the vector's direction until close enough to the incremental goal
        while True:
            plan = move_group.go(wait=True)
            current_pose = move_group.get_current_pose().pose
            if all_close(pose_goal, current_pose, 0.05):
                break

        # Stop all movement and clear targets
        move_group.stop()
        move_group.clear_pose_targets()

    # use the head camera to achieve the viewpoint 
    else:
        # publish to "/trina2_1/main_cam_yaw_controller/command" (type Float64) for yaw control
        # publish to "/trina2_1/main_cam_pitch_controller/command" (type Float64) for pitch control
        
        # set up the control messages (starting at current pitch and yaw)
        pitch_msg = Float64()
        pitch_msg.data = headcam_pitch
        yaw_msg = Float64()
        yaw_msg.data = headcam_yaw

        # move in direction of the vector in small steps
        if y_goal > 0: 
            yaw_msg.data += 0.1
            headcam_yaw += 0.1
        elif y_goal < 0:
            yaw_msg.data -= 0.1
            headcam_yaw -= 0.1
        if z_goal > 0:
            pitch_msg.data += 0.1
            headcam_pitch += 0.1
        elif z_goal < 0:
            pitch_msg.data -= 0.1
            headcam_pitch -= 0.1

        # publish the move goals to move the head camera directly
        headcam_pitch_pub.publish(pitch_msg)
        headcam_yaw_pub.publish(yaw_msg)

if __name__ == "__main__":
    """
    initialize the node then spin to perform desired motions based on the vector callbacks
    """
    global headcam_pitch_pub
    global headcam_yaw_pub
    global sec_cam
    # initializes the node and instantiates default variable values
    startNode()
    # spin to update the callback functions and perform the motion towards the received vector
    rospy.spin()

