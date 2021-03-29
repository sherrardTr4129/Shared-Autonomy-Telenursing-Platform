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
from stl import mesh
import rospkg
import os


class InfoEntropy:
    # TODO
    #   import gazebo object mesh
    #   get faces visible from cameras
    #   for each face, calculate entropy

    def __init__(self):
        self.get_obj()
        self.cube
        return

    def get_obj(self):
        """
        Get the gazebo object from the given id

        params:
            objID -> (String) name of the gazebo object
        returns:
            Mesh object -> gazebo object mesh

        """
        rospack = rospkg.RosPack()
        pkgpath = rospack.get_path('info_entropy')
        stlpath = os.path.join(pkgpath, 'urdfs', 'cube.STL')

        cube_mesh = mesh.Mesh.from_file(stlpath)
        print(cube_mesh.normals)

        self.cube = cube_mesh

        return cube_mesh

    def get_visible_faces(self, obj, camera_view):
        """
        Get normals of each face of gazebo object and faces visible by camera

        params:
            obj -> gazebo object
        """
        normals = self.cube.normals





if __name__ == "__main__":
    try:
        InfoEntropy()
    except rospy.ROSInterruptException:
        pass