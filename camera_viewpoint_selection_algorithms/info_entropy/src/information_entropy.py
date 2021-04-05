#!/usr/bin/env python
# Author: Jesse d'Almeida
# Project: HRI Autonomous Camera Teleoperation Assistance
# Since: November 10, 2020

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3, PoseArray, PoseStamped, Pose, TransformStamped, Vector3Stamped, Quaternion, PointStamped
from sensor_msgs.msg import PointCloud
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
        self.obj_trans = None
        self.obj_rot = None

        camera = PoseStamped()
        camera.header.frame_id = '/map'
        camera.pose.position.z = .5
        camera.pose.orientation.x = 1
        camera.pose.orientation.w = 0

        # publishers
        self.normals_pub = rospy.Publisher('norms', PoseArray, queue_size=10)
        self.facing_norms_pub = rospy.Publisher('facing_norms', PoseArray, queue_size=10)
        self.camera_pub = rospy.Publisher('cam', PoseStamped, queue_size=10)

        self.verts_pub = rospy.Publisher('verts', PointCloud, queue_size=10)

        # wait for listener for transformation to object
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("map", "box", rospy.Time(0), rospy.Duration(3))

        # start
        self.get_obj()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.camera_pub.publish(camera)

            normPoses = self.get_visible_faces()
            # self.get_cam_facing(camera, normPoses)
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
        stlpath = os.path.join(pkgpath, 'urdfs', 'box.STL')

        cube_mesh = mesh.Mesh.from_file(stlpath)

        self.cube = cube_mesh
        # volume, cog, inertia = cube_mesh.get_mass_properties()
        # self.cog = cog

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

        verts = PointCloud()
        verts.header.frame_id = '/map'

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
            q = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw, axes='rxyz')
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
            # make a poseStamped for transform
            ps = PoseStamped()
            ps.pose = p
            ps.header.frame_id = '/box'
            trans_p = self.listener.transformPose('/map', ps)

            # print center of faces in a point cloud
            centers = PointStamped()
            centers.header.frame_id = '/box'
            centers.point.x = p.position.x
            centers.point.y = p.position.y
            centers.point.z = p.position.z
            trans_center = self.listener.transformPoint('/map', centers)

            # add transformed pose to the list of faces
            normPoses.poses.append(trans_p.pose)

        self.normals_pub.publish(normPoses)
        # self.verts_pub.publish(verts)
        return normPoses

    def get_cam_facing(self, camera_view, normPoses):
        """

        params:
            camera_view -> (Pose)
        """
        facingPoses = PoseArray()
        facingPoses.header.frame_id = '/map'

        # iterate through each pose and keep those that are facing camera
        for npose in normPoses.poses:
            pdiff = self.posesubtract(camera_view.pose, npose)

            dot = self.quatdot(pdiff.orientation, camera_view.pose.orientation)
            if dot < 0:
                facingPoses.poses.append(npose)

        self.facing_norms_pub.publish(facingPoses)

# -------------------------- HELPERS -------------------

    def posesubtract(self, p1, p2):
        """
        computer P = p1 - p2

        params:
            p1 -> Pose()
            p2 -> Pose()
        returns:
            P -> Pose()
        """

        P = Pose()
        # difference in position
        P.position.x = p1.position.x - p2.position.x
        P.position.y = p1.position.y - p2.position.y
        P.position.z = p1.position.z - p2.position.z

        # difference in orientation

        # invert p2 orientation by negating w
        qinv = [0]*4
        qinv[0] = p2.orientation.x
        qinv[1] = p2.orientation.y
        qinv[2] = p2.orientation.z
        qinv[3] = -p2.orientation.w

        q1 = [0] * 4
        q1[0] = p1.orientation.x
        q1[1] = p1.orientation.y
        q1[2] = p1.orientation.z
        q1[3] = p1.orientation.w

        qr = tf_conversions.transformations.quaternion_multiply(qinv, q1)

        P.orientation = Quaternion(qr[0], qr[1], qr[2], qr[3])

        return P

    def quatdot(self, q1, q2):
        """
        computers dot product of 2 quaternions
        """
        return q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w



if __name__ == "__main__":
    try:
        InfoEntropy()
    except rospy.ROSInterruptException:
        pass