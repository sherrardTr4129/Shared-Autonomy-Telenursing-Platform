#!/usr/bin/env python

## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from std_srvs.srv import Empty
from moveit_commander.conversions import pose_to_list
from telenursing_trina2_ctrl.msg import joint_angle
from tf.transformations import euler_from_quaternion, quaternion_from_euler
global x,y,z, roll, pitch, yaw, ox, oy, oz, ow

#Misc variables
# arm home position in radian

# this node is for right arm
class ControlRightArm(object):
    """MoveGroupPythonInteface"""
    def __init__(self):
        super(ControlRightArm, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('control_right_arm', anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints). 
        ## This interface can be used to plan and execute motions:
        group1_name = "left_arm"
        group2_name = "right_arm"
        move_group1 = moveit_commander.MoveGroupCommander(group1_name)
        move_group2 = moveit_commander.MoveGroupCommander(group2_name)

        rospy.Subscriber('/camArmPoseGoal', Pose, self.go_to_pose_goal, queue_size=1)
        self.pub_right_pose = rospy.Publisher('/currentCamArmPose', Pose ,queue_size=1)
        self.pub_angle_pose = rospy.Publisher('/currentCamArmPose_angle', String ,queue_size=1)
        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group1 = move_group1
        self.move_group2 = move_group2

    def pub_current_pose(self):
        move_group = self.move_group2  # right arm
        current_pose = move_group.get_current_pose().pose
        angle_pose = move_group.get_current_rpy()
        yaw = angle_pose[2]
        roll = angle_pose[0]
        pitch = angle_pose[1]
        string = "roll:"+str(roll) +"  yaw:"+str(yaw)+"  pitch:"+str(pitch)
        self.pub_angle_pose.publish(string)
        self.pub_right_pose.publish(current_pose)


    def go_to_pose_goal(self, goal):
        rospy.loginfo("move cam arm")
        move_group = self.move_group2  # right arm

        move_group.set_pose_target(goal)
        while True:
            plan = move_group.go(wait=True)
            current_pose = move_group.get_current_pose().pose
            if all_close(goal, current_pose, 0.5):
                break

        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()


    def move_left_arm(self,angles):
        move_group1 = self.move_group1
        joint_goal1 = move_group1.get_current_joint_values()
        joint_goal1[0] = angles.left[0]
        joint_goal1[1] = angles.left[1]
        joint_goal1[2] = angles.left[2]
        joint_goal1[3] = angles.left[3]
        joint_goal1[4] = angles.left[4]
        joint_goal1[5] = angles.left[5]
        joint_goal1[6] = angles.left[6]
        # self.move_to_goal(move_group1, joint_goal1, 10)
        # move_group1.stop()
        move_group1.go(joint_goal1, wait=True)
        move_group1.stop()

    def move_right_arm(self,angles):
        move_group2 = self.move_group2
        joint_goal2 = move_group2.get_current_joint_values()
        joint_goal2[0] = angles.right[0]
        joint_goal2[1] = angles.right[1]
        joint_goal2[2] = angles.right[2]
        joint_goal2[3] = angles.right[3]
        joint_goal2[4] = angles.right[4]
        joint_goal2[5] = angles.right[5]
        joint_goal2[6] = angles.right[6]
        # self.move_to_goal(move_group2, joint_goal2, 10)
        # move_group2.stop()
        # move_group2.go(joint_goal2, wait=True)
        # move_group2.stop()

    def move_to_goal(self, move_group, joint_goal, trials):
        trial = 0
        while True:
            move_group.go(joint_goal, wait=True)
            current_joints = move_group.get_current_joint_values()
            if all_close(joint_goal, current_joints, 0.05):
                break
            
            # Check trial times
            trial += 1
            if trial > 9:
                rospy.loginfo("Moving arm to home failed")
                break


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


if __name__ == "__main__":
    interface = ControlRightArm()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        interface.pub_current_pose() 
        rate.sleep()
