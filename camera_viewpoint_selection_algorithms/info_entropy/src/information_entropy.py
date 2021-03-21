#!/usr/bin/env python
# Author: Jesse d'Almeida
# Project: HRI Autonomous Camera Teleoperation Assistance
# Since: November 10, 2020

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
import cv2
import numpy as np
import math


class InfoEntropy:
    # TODO
    #   import gazebo object mesh
    #   get faces visible from cameras
    #   for each face, calculate entropy

    def __init__(self):


        return

    def get_gazebo_obj(self, objID):
        """
        Get the gazebo object from the given id

        params:
            objID -> (String) name of the gazebo object
        returns:
            Mesh object -> gazebo object mesh

        """

        return

    def get_visible_faces(self, obj, camera_view):
        """
        Get normals of each face of gazebo object and faces visible by camera

        params:
            obj -> gazebo object
        """

if __name__ == "__main__":
    try:
        InfoEntropy()
    except rospy.ROSInterruptException:
        pass