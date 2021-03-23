#!/usr/bin/env python
# Author: Trevor Sherrard
# Project: HRI Autonomous Camera Teleoperation Assistance
# Since: November 10, 2020

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from tf.transformations import quaternion_from_euler
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import operator

# declare movement vector topics
moveVecTopic = "/saliency_move_vec"

# image topic
imageTopic = "/trina2_1/secondaryCameraStream/color/image_raw"

# create global vector3 message
global moveVec
moveVec = Vector3

# set up Pose publisher for newly computed Pose
vecPub = rospy.Publisher(moveVecTopic, Vector3)

def subLists(listA, listB):
    diffList = []
    for i in range(len(listA)):
        diffList.append(listA[i] - listB[i])

    return diffList

def countNumOverThresh(diffList, threshVal):
    count = 0
    for elem in diffList:
        if elem > 0 and elem > threshVal:
            count += 1

    return count

def euclidDist(pt1, pt2):
    return math.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)

def findMinimizationDir(imgCenterPt, momentCenterPts):
    threshVal = 40
    # compute initial distances
    initDist = []
    for elem in momentCenterPts:
        dist = euclidDist(elem, imgCenterPt)
        initDist.append(dist)

    # create four shifted copies of the image center point
    # in each of the cardinal directions.
    newCenterPtUp = (imgCenterPt[0], imgCenterPt[1] - 50)
    newCenterPtDown = (imgCenterPt[0], imgCenterPt[1] + 50)
    newCenterPtLeft = (imgCenterPt[0] - 50, imgCenterPt[1])
    newCenterPtRight = (imgCenterPt[0] + 50, imgCenterPt[1])

    # find euclidean distance between each of the shifted center points
    # and each of the saliency contour COM's.
    newDistUp = []
    newDistDown = []
    newDistLeft = []
    newDistRight = []
    for elem in momentCenterPts:
        distUp = euclidDist(elem, newCenterPtUp)
        distDown = euclidDist(elem, newCenterPtDown)
        distLeft = euclidDist(elem, newCenterPtLeft)
        distRight = euclidDist(elem, newCenterPtRight)

        newDistUp.append(distUp)
        newDistDown.append(distDown)
        newDistLeft.append(distLeft)
        newDistRight.append(distRight)

    subListUp = subLists(initDist, newDistUp)
    subListDown = subLists(initDist, newDistDown)
    subListLeft = subLists(initDist, newDistLeft)
    subListRight = subLists(initDist, newDistRight)

    countUp = countNumOverThresh(subListUp, threshVal)
    countDown = countNumOverThresh(subListDown, threshVal)
    countLeft = countNumOverThresh(subListLeft, threshVal)
    countRight = countNumOverThresh(subListRight, threshVal)

    # create direction count dictionary
    dictToSort = {"Up":countUp, "Down":countDown, "Left":countLeft, "Right":countRight}

    # if moving won't help reduce any distance from image center to COM of countours, then we
    # are done
    if(dictToSort["Up"] == dictToSort["Down"] and dictToSort["Left"] == dictToSort["Right"]):
        return "Done"

    elif(dictToSort["Up"] == dictToSort["Down"] and dictToSort["Left"] != dictToSort["Right"]):
        rightVal = dictToSort["Right"]
        leftVal = dictToSort["Left"]

        if(rightVal > leftVal):
            return "Right"
        elif(rightVal < leftVal):
            return "Left"
    elif(dictToSort["Up"] != dictToSort["Down"] and dictToSort["Left"] == dictToSort["Right"]):
        upVal = dictToSort["Up"]
        downVal = dictToSort["Down"]

        if(upVal > downVal):
            return "Up"
        elif(upVal < downVal):
            return "Down"
    else:
        maxVal = max(dictToSort.iteritems(), key=operator.itemgetter(1))[0]
        return maxVal

def computeSaliencyMap(img):
    # define constants
    maxVal = 255
    threshVal = 130
    minArea = 500
    maxArea = 50000

    # compute image center
    imgCY = np.size(img, 0)/2
    imgCX = np.size(img, 1)/2
    imgCenterPt = (imgCX, imgCY) 

    # compute saliency image
    saliencyObj = cv2.saliency.StaticSaliencyFineGrained_create()
    (success, saliencyMap) = saliencyObj.computeSaliency(img) 
    if(success):
        # scale image to (0, 255)
        saliencyMap = (saliencyMap * 255).astype("uint8")

        #blur image
        blur = cv2.GaussianBlur(saliencyMap, (5, 5), 0)

        # threshold image
        ret, thresh = cv2.threshold(blur, threshVal, maxVal, cv2.THRESH_BINARY)

        _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        # filter detected contours
        filteredContours = []
        for cont in contours:
            area = cv2.contourArea(cont)
            if(area > minArea and area < maxArea):
                filteredContours.append(cont)
        
        # draw filtered contours
        cv2.drawContours(img, filteredContours, -1, (0, 255, 0), 1)
        
        # find center of all filter contours
        momentCenterPts = []
        for cont in filteredContours:

            # find COF of single contour
            M = cv2.moments(cont)
	    centerX = int(M["m10"] / M["m00"])
	    centerY = int(M["m01"] / M["m00"])
            MomentCenterPt = (centerX, centerY)

            # draw circle on COM, draw line between image center and COM
            cv2.circle(img, (centerX, centerY), 7, (255, 0, 0), -1)
            cv2.line(img, (centerX, centerY), (imgCX, imgCY), (0, 0, 0), thickness=2)

            # add COM point to list
            momentCenterPts.append(MomentCenterPt)

        # draw marker on image center
        cv2.circle(img, (imgCX, imgCY), 7, (255, 255, 255), -1)

        # find cardinal direction that minimizes distances between 
        # saliency COM and center of image
        direction = findMinimizationDir(imgCenterPt, momentCenterPts)
        
        if(direction == "Up"):
            cv2.arrowedLine(img, (imgCX, imgCY), (imgCX, imgCY - 100), (0,0,255), 5)
        elif(direction == "Down"):
            cv2.arrowedLine(img, (imgCX, imgCY), (imgCX, imgCY + 100), (0,0,255), 5)
        elif(direction == "Left"):
            cv2.arrowedLine(img, (imgCX, imgCY), (imgCX - 100, imgCY), (0,0,255), 5)
        elif(direction == "Right"):
            cv2.arrowedLine(img, (imgCX, imgCY), (imgCX + 100, imgCY), (0,0,255), 5)
        elif(direction == "Done"):
            cv2.putText(img, 'Done!', imgCenterPt, cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        return img, direction

def procImage(img):
    """
    This function processes the recieved ROS image
    and calculate the direction to move to maximize
    saliency within the frame
    """
    # create new vector to be used
    tempVec = Vector3()

    # crop image
    width = np.size(img, 1)
    height = np.size(img, 0)
    croppedImage = img[0:height - 140, 0:width]

    # perform saliency processing
    img, direction = computeSaliencyMap(croppedImage)

    # populate vector components
    if(direction == "Up"):
        tempVec.x = 0
        tempVec.y = 1
        tempVec.z = 0
    elif(direction == "Down"):
        tempVec.x = 0
        tempVec.y = -1
        tempVec.z = 0
    elif(direction == "Right"):
        tempVec.x = 1
        tempVec.y = 0
        tempVec.z = 0
    elif(direction == "Left"):
        tempVec.x = -1
        tempVec.y = 0
        tempVec.z = 0
    else:
        tempVec.x = 0
        tempVec.y = 0
        tempVec.z = 0

    cv2.imshow("test", img)
    cv2.waitKey(1)

    # return new vector
    return tempVec

def imageCallback(msg):
    """
    This function serves as the ROS subscription callback. It converts
    the raw image data to an OpenCV compatable format.

    params:
        msg -> The recieved ROS image message.
    returns:
        None
    """

    bridge = CvBridge()
    orig = bridge.imgmsg_to_cv2(msg, "bgr8")

    # process image
    movementVec = procImage(orig)

    # publish new pose
    vecPub.publish(movementVec)

def startNode():
    """
    This function serves to initialize the ROS node.
    """
    rospy.init_node("saliency_to_pose")
    rospy.loginfo("saliency_to_pose node started")

    rospy.Subscriber(imageTopic, Image, imageCallback)
    rospy.spin()
if __name__ == "__main__":
    try:
        startNode()
    except rospy.ROSInterruptException:
        pass
