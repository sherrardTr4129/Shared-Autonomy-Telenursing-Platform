#!/usr/bin/env python
# Author: Jesse d'Almeida
# Project: HRI Autonomous Camera Teleoperation Assistance
# Since: November 10, 2020

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3, PoseArray, PoseStamped, Pose
import geometry_msgs.msg
import cv2
import numpy as np
import math
from stl import mesh
import rospkg
import os
import tf
import tf_conversions


class InfoEntropy:
    # TODO
    #   import gazebo object mesh
    #   get faces visible from cameras
    #   for each face, calculate entropy

    def __init__(self):
        rospy.init_node('info_entropy', anonymous=True)

        # attributes
        self.cube = None
        self.cog = None
        self.normals_pose = None

        camera = PoseStamped()
        camera.header.frame_id = '/map'
        camera.pose.position.z = 1
        camera.pose.orientation.x = 1
        camera.pose.orientation.w = 0

        # publishers
        self.normals_pub = rospy.Publisher('norms', PoseArray, queue_size=10)
        self.facing_norms_pub = rospy.Publisher('facing_norms', PoseArray, queue_size=10)
        self.camera_pub = rospy.Publisher('cam', PoseStamped, queue_size=10)

        listener = tf.TransformListener()

        try:
            (trans, rot) = listener.lookupTransform('/map', '/box', rospy.Time(0))
            print(trans)
            self.obj_trans = trans
            self.obj_rot = rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        # start
        self.get_obj()
        rate = rospy.Rate(10)
        self.camera_pub.publish(camera)

        while not rospy.is_shutdown():
            self.get_visible_faces()
            rate.sleep()
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

        self.cube = cube_mesh
        volume, cog, inertia = cube_mesh.get_mass_properties()
        self.cog = cog
        print(self.cube.normals)

        return cube_mesh

    def get_visible_faces(self):
        """
        Get normals of each face of gazebo object and faces visible by camera

        """

        # get cube normal vectors and vertices of faces
        normals = self.cube.normals
        v0_list = self.cube.v0
        v1_list = self.cube.v1
        v2_list = self.cube.v2

        # init the pose array to populate
        normPoses = PoseArray()
        normPoses.header.frame_id = '/map'

        for i in range(len(normals)):
            # normalize the normal vector
            n = normals[i] / np.linalg.norm(normals[i])
            x, y, z, = n
            v0 = v0_list[i]
            v1 = v1_list[i]
            v2 = v2_list[i]

            # create euler angles from the vector direction
            yaw = math.atan2(y, x)
            pitch = math.atan2(-z, y)
            roll = 0
            q = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)

            # fill in the orientation information
            p = Pose()
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]

            # use center of face as position
            p.position.x = (v0[0] + v1[0] + v2[0])/300
            p.position.y = (v0[1] + v1[1] + v2[1])/300
            p.position.z = (v0[2] + v1[2] + v2[2])/300

            normPoses.poses.append(p)

        self.normals_pose = normPoses
        self.normals_pub.publish(normPoses)

    def get_cam_facing(self, camera_view):
        """

        params:
            camera_view -> (Pose)
        """

        origin = np.array([camera_view.pose.position.x, camera_view.pose.position.y, camera_view.pose.position.z])

        # iterate through each pose and keep those that are facing camera
        for pose in self.normals_pose.poses:
            pass





if __name__ == "__main__":
    try:
        InfoEntropy()
    except rospy.ROSInterruptException:
        pass