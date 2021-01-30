#!/usr/bin/env python

## Author: Trevor Sherrard
## Course: Directed Research
## Project: Telenursing Robot Interface 
## Since: 01/30/2021

import rospy
import threading
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

# declare image topics from which images will be resized
leftArmCamTopic = "/trina2_1/left_arm_cam/color/image_raw"
headCamTopic = "/trina2_1/main_cam/color/image_raw"

# create publisher objects for the republished image topics
primaryCamRepub = "/trina2_1/primaryCameraStream/color/imag_raw"
secondaryCamRepub = "/trina2_1/secondaryCameraStream/color/imag_raw"

# topics for knowing which camera is the primary or secondary
primaryCamTopic = "primaryCam"
secondaryCamTopic = "secondaryCam"

# define and init global variables
global headCamImage
global armCamImage
global primaryCamString
global secondayCamString
headCamImage = np.zeros(shape=[512, 512, 3])
armCamImage = np.zeros(shape=[512, 512, 3])
primaryCamString = ""
secondaryCamString = ""

# misc variables
primaryResizeFactor = 0.8
secondaryResizeFactor = 0.4

def headImageCallback(msg):
    """
    This function resizes the image message recieved
    over the head image topic and republishes it to a new
    topic.

    params:
        msg (sensor_msgs/Image): The image message payload recieved
    returns:
        None
    """
    global headCamImage
    
    # extract head cam image and update globally
    bridge = CvBridge()
    headCamImage = bridge.imgmsg_to_cv2(msg, "bgr8")

def leftArmImageCallback(msg):
    """
    This function resizes the image message recieved
    over the left arm image topic and republishes it to a new
    topic.

    params:
        msg (sensor_msgs/Image): The image message payload recieved
    returns:
        None
    """
    global armCamImage

    # extract arm cam image and update globally
    bridge = CvBridge()
    armCamImage = bridge.imgmsg_to_cv2(msg, "bgr8")

def primaryCamCallback(msg):
    """
    This function subscribes to the primaryCam topic 
    and updates its state in the global scope.

    params:
        msg (std_msgs/String): The String message payload recieved
    returns:
        None
    """

    global primaryCamString
    primaryCamString = msg.data

def secondayCamCallback(msg):
    """
    This function subscribes to the secondaryCam topic 
    and updates its state in the global scope.

    params:
        msg (std_msgs/String): The String message payload recieved
    returns:
        None
    """

    global secondaryCamString
    secondaryCamString = msg.data

def resizeImage(image, resizeFactor):
    """
    This function uniformally scales and image by
    resizeFactor

    params:
        Image (OpenCV image): The image to resize
        resizeFactor (float): A value between 0 and 1 to scale image by

    returns:
        scaledImage (OpenCV image): the scaled image
    """

    # compute new image size
    newWidth = image.shape[1] * resizeFactor
    newHeight = image.shape[0] * resizeFactor
    newSize = (int(newWidth), int(newHeight))

    # scale image
    scaledImage = cv2.resize(image, newSize)
    return scaledImage

def resizeAndRepubThread():
    """
    This function first determines which camera is the primary and which
    is secondary. The image streams from the respective primary and seconday
    cameras are resized and republished
    
    params:
        None
    returns:
        None
    """

    # reference globals
    global primaryCamString
    global secondaryCamString
    global armCamImage
    global headCamImage

    # initialize image publishers
    primaryPub = rospy.Publisher(primaryCamRepub, Image, queue_size=1)
    secondaryPub = rospy.Publisher(secondaryCamRepub, Image, queue_size=1)

    # create CvBridge object for converting CV2 images to sensor_msgs/Image messages
    backBridge = CvBridge()

    while(True):
        primaryImage = np.zeros(shape=[512, 512, 3])
        secondaryImage = np.zeros(shape=[512, 512, 3])

        # just keep looping until we get images
        if(headCamImage.all() == 0 or armCamImage.all() == 0):
            rospy.loginfo("still waiting on camera images...")
            continue

        # get primary image
        if(primaryCamString == "head"):
            primaryImage = resizeImage(headCamImage, primaryResizeFactor)
        elif(primaryCamString == "leftArm"):
            primaryImage = resizeImage(armCamImage, primaryResizeFactor)
        elif(primaryCamString == ""):
            pass
        else:
            rospy.logerr("Invalid Option for primaryCamString recieved!")

        # get secondary image
        if(secondaryCamString == "head"):
            secondaryImage = resizeImage(headCamImage, secondaryResizeFactor)
        elif(secondaryCamString == "leftArm"):
            secondaryImage = resizeImage(armCamImage, secondaryResizeFactor)
        elif(secondaryCamString == ""):
            pass
        else:
            rospy.logerr("Invalid Option for secondaryCamString recieved!")

        # publish both new images
        if(primaryImage.all() != 0 and secondaryImage.all() != 0):
            primaryImageMessage = backBridge.cv2_to_imgmsg(primaryImage, "bgr8")
            primaryPub.publish(primaryImageMessage)

            secondaryImageMessage = backBridge.cv2_to_imgmsg(secondaryImage, "bgr8")
            secondaryPub.publish(secondaryImageMessage)

def startNode():
    """
    this function serves to initialize the ROS node
    """

    # init node
    rospy.init_node("resize_and_repub")
    rospy.loginfo("resize_and_repub node started")

    # setup subcribers
    rospy.Subscriber(leftArmCamTopic, Image, leftArmImageCallback)
    rospy.Subscriber(headCamTopic, Image, headImageCallback)
    rospy.Subscriber(primaryCamTopic, String, primaryCamCallback)
    rospy.Subscriber(secondaryCamTopic, String, secondayCamCallback)
    rospy.loginfo("all subscribers initialized, entering publishing loop...")

    # start repub thread
    thread = threading.Thread(target=resizeAndRepubThread)
    thread.start()
    rospy.spin()

if __name__ == "__main__":
    try:
        startNode()
    except rospy.ROSInterruptException:
        pass
