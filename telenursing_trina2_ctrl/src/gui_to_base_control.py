#!/usr/bin/env python

## Author: Trevor Sherrard
## Course: Directed Research
## Project: COVID-19 telenursing platform
## Since: 01/30/2021

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# set joystick topic name
joyTopic = "/virtualJoystick"

# create publisher for cmd_vel messages
cmdVelTopic = "/trina2_1/base_controller/cmd_vel"
cmdValPub = rospy.Publisher(cmdVelTopic, Twist, queue_size=1)

def joyCallback(msg):
    """
    this function extracts the linear and angular command
    components from the recieved sensor_msgs/Joy message
    and computes and publishes a Twist message to the command_vel topic

    params:
        msg (sensor_msgs/Joy message): the recieved Joy message
    returns:
        None
    """

    # extract message components
    fwdRev = msg.axes[0]
    spin = msg.axes[1]

    # halve each value
    if(fwdRev != 0):
        fwdRev /= 2
    if(spin != 0):
        spin /= 2

    # invert the direction of the scaled spin value
    spin = -spin

    # construct twist
    cmdVelTwist = Twist()
    cmdVelTwist.linear.x = float(fwdRev)
    cmdVelTwist.angular.z = float(spin)

    # publish new message
    cmdValPub.publish(cmdVelTwist)

def startNode():
    #init node
    rospy.init_node("gui_to_base_control")
    rospy.loginfo("gui_to_base_control initialized")

    # subscribe to web_gui joystick message
    rospy.Subscriber(joyTopic, Joy, joyCallback)
    rospy.loginfo("joystick subscriber initialized")

    rospy.spin()

if __name__ == "__main__":
    try:
        startNode()
    except rospy.ROSInterruptException:
        pass
