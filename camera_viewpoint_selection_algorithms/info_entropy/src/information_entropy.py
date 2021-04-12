#!/usr/bin/env python
# Author: Jesse d'Almeida
# Project: HRI Autonomous Camera Teleoperation Assistance
# Since: November 10, 2020

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3, PoseArray, PoseStamped, Pose, TransformStamped, Vector3Stamped, Quaternion, PointStamped
from sensor_msgs.msg import PointCloud
import geometry_msgs.msg
from std_msgs.msg import Float32
import cv2
import numpy as np
import math
from stl import mesh
import rospkg
import os
import tf
import tf_conversions
from tf.transformations import quaternion_matrix


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
        self.numFaces = None
        self.box_frame = "box"

        # camera properties
        camera = PoseStamped()
        camera.header.frame_id = '/map'
        camera.pose.position.z = .5
        camera.pose.orientation.w = 1

        camR = quaternion_matrix([camera.pose.orientation.x, camera.pose.orientation.y, camera.pose.orientation.z, camera.pose.orientation.w])
        self.rvec, _ = cv2.Rodrigues(np.asarray(camR))
        self.tvec = [camera.pose.position.x, camera.pose.position.y, camera.pose.position.z]

        self.camera_matrix = [[619.55, 0, 429.5], [0, 619.55, 360.5], [0, 0, 1]]
        self.projection_M = [[619.55, 0, 429.5, 0], [0, 619.55, 360.5, 0], [0, 0, 1, 0]]

        self.screen_area = (2*619.55)**2

        # publishers
        self.normals_pub = rospy.Publisher('norms', PoseArray, queue_size=10)
        self.facing_norms_pub = rospy.Publisher('facing_norms', PoseArray, queue_size=10)
        self.camera_pub = rospy.Publisher('cam', PoseStamped, queue_size=10)
        self.new_camera_pub = rospy.Publisher('cam_array', PoseArray, queue_size=10)

        self.entropy_vec_pub = rospy.Publisher('entropy', PoseStamped, queue_size=10)

        # wait for listener for transformation to object
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("map", self.box_frame, rospy.Time(0), rospy.Duration(3))

        # start
        self.get_obj()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.camera_pub.publish(camera)
            self.calc_entropy_vec(camera)

            # normPoses = self.get_visible_faces()
            # facingPoses, facingIndex = self.get_cam_facing(camera, normPoses, math.pi/2)
            #
            # entropy = self.calc_entropy(facingIndex)
            rate.sleep()
        return

    def get_obj(self):
        """
        Get the gazebo object from the given id

        params:
            objID -> [String] name of the gazebo object
        returns:
            Mesh object -> gazebo object mesh

        """
        rospack = rospkg.RosPack()
        pkgpath = rospack.get_path('info_entropy')
        stlpath = os.path.join(pkgpath, 'urdfs', 'box.STL')

        cube_mesh = mesh.Mesh.from_file(stlpath)

        self.cube = cube_mesh
        self.numFaces = np.shape(cube_mesh.normals)[0]
        # volume, cog, inertia = cube_mesh.get_mass_properties()
        # self.cog = cog

        return cube_mesh

    def calc_entropy_vec(self, camera):
        """
        'move' camera through a series of points, calc entropy at each
        generate pose that is weighted avg of each pose moved

        params:
            camera -> [PoseStamped] current pose of camera

        returns:
            entropy_vec -> [PoseStamped] weight avg of direction to move
        """

        stp = 0.5       # dist to step in each direction
        # 7 possible positions: Current,       forward       back        left            right       up            down
        step_list = np.asarray([[0, 0, 0], [stp, 0, 0], [-stp, 0, 0], [0, stp, 0], [0, -stp, 0], [0, 0, stp], [0, 0, -stp]])
        num_poses = 7

        cam_pose_array = []
        entropy_vals = []

        # loop through the camera positions and get entropy at each
        for s in step_list:

            # create a new camera based on values of original camera
            new_cam = PoseStamped()
            new_cam.header.frame_id = '/map'
            new_cam.pose.orientation = camera.pose.orientation
            new_cam.pose.position.x = camera.pose.position.x + s[0]
            new_cam.pose.position.y = camera.pose.position.y + s[1]
            new_cam.pose.position.z = camera.pose.position.z + s[2]
            cam_pose_array.append(new_cam)

            camR = quaternion_matrix([new_cam.pose.orientation.x, new_cam.pose.orientation.y, new_cam.pose.orientation.z,
                                      new_cam.pose.orientation.w])
            rvec, _ = cv2.Rodrigues(np.asarray(camR))
            tvec = [new_cam.pose.position.x, new_cam.pose.position.y, new_cam.pose.position.z]


            normPoses = self.get_visible_faces()
            facingPoses, facingIndex = self.get_cam_facing(new_cam, normPoses, math.pi / 2)
            entropy = self.calc_entropy(facingIndex, rvec, tvec)
            entropy_vals.append(entropy)

        nentropy = np.linalg.norm(np.asarray(entropy_vals))     # get norm of entropy values
        entropy_vals = np.fabs(entropy_vals/nentropy)

        # take weighted value of each x, y, z in camera pose
        sum_pos = np.asarray([0.0, 0.0, 0.0])
        for i in range(len(cam_pose_array)):
            if math.isnan(entropy_vals[i]):
                continue

            point = cam_pose_array[i].pose.position
            sum_pos[0] += point.x * entropy_vals[i]
            sum_pos[1] += point.y * entropy_vals[i]
            sum_pos[2] += point.z * entropy_vals[i]

        entropy_vec = PoseStamped()
        entropy_vec.header.frame_id = '/map'
        entropy_vec.pose.orientation = camera.pose.orientation
        entropy_vec.pose.position.x = sum_pos[0]/num_poses
        entropy_vec.pose.position.y = sum_pos[1]/num_poses
        entropy_vec.pose.position.z = sum_pos[2]/num_poses

        print(entropy_vec)
        self.entropy_vec_pub.publish(entropy_vec)

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
            ps.header.frame_id = self.box_frame
            trans_p = self.listener.transformPose('/map', ps)

            # add transformed pose to the list of faces
            normPoses.poses.append(trans_p.pose)

        self.normals_pub.publish(normPoses)
        return normPoses

    def get_cam_facing(self, camera_view, normPoses, camera_ang):
        """

        params:
            camera_view -> [PoseStamped]
            normPoses -> [PoseArray] Poses of each face normal
            camera_ang -> (rad) angle of view of camera
        """
        facingPoses = PoseArray()
        facingPoses.header.frame_id = '/map'

        facing_index = np.zeros(self.numFaces)  # keep tabs of which face is seen

        # iterate through each pose and keep those that are facing camera
        f = 0
        for npose in normPoses.poses:

            pdiff = self.posesubtract(camera_view.pose, npose)
            pdiff_dir = self.dir_to_quaternion([pdiff.position.x, pdiff.position.y, pdiff.position.z])

            # check if the point is within the focal range of the camera
            alpha = math.acos(self.quatdot(camera_view.pose.orientation, pdiff_dir))
            if -camera_ang > alpha > camera_ang:
                continue

            # check if normal is facing the camera
            dot = self.quatdot(npose.orientation, camera_view.pose.orientation)
            bound = 0.01
            if math.fabs(dot) < bound:
                facingPoses.poses.append(npose)
                facing_index[f] = 1     # list face as being seen
            f += 1

        self.facing_norms_pub.publish(facingPoses)
        return facingPoses, facing_index

    def calc_projected_area(self, p0, p1, p2, rvec, tvec):
        """
        calculate the projected area from the 3D points onto the camera plane

        params:
            p0 -> [3x1] 3d coords of first point
            p1 -> [3x1] 3d coords of second point
            p2 -> [3x1] 3d coords of third point

        returns:
            area -> (px^2) projected area of point on to camera plane
        """

        obj_pts = np.asarray(self.transform_box2map([p0, p1, p2]))

        px_points, _ = cv2.projectPoints(obj_pts, np.asarray(rvec), np.asarray(tvec), np.asarray(self.camera_matrix), np.asarray([]))

        # calc area of triangle from the 3 points
        a = px_points[0][0]
        b = px_points[1][0]
        c = px_points[2][0]
        area = math.fabs(0.5 * (a[0]*(b[1] - c[1]) + b[0]*(c[1] - a[1]) + c[0]*(a[1] - b[1])))
        return area

    def calc_entropy(self, facingIndex, rvec, tvec):
        """
        calculate the information entropy of a given scene
        params:
            facingIndex -> [numFaces] bool of if each face is seen by camera
        returns:
            entropy -> (double) value of entropy
        """

        entropy = 0
        for i in range(self.numFaces):

            # skip if face is not seen by camera
            if not facingIndex[i]:
                continue

            # get the verts of the face
            v0 = self.cube.v0[i]
            v1 = self.cube.v1[i]
            v2 = self.cube.v2[i]

            ai = self.calc_projected_area(v0, v1, v2, rvec, tvec)

            if not ai:
                continue

            ratio = ai/self.screen_area

            entropy = entropy + ratio * math.log(ratio)
        return -entropy


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

    def normalize_pose(self, p):
        """
        normalizes the position of a pose
        args:
            p -> (Pose)
        returns:
            np -> (Pose)
        """
        np = Pose()
        psize = math.sqrt(p.position.x**2 + p.position.y**2 + p.position.z**2)

        np.position.x = p.position.x / psize
        np.position.y = p.position.y / psize
        np.position.z = p.position.z / psize

        return np

    def dir_to_quaternion(self, dir):
        """
        converts a direction in 3D space to euler angles to quaternion
        args:
            dir -> [3x1] vector in 3D space of direction
        returns:
            q -> (Quaternion)
        """
        x = dir[0]
        y = dir[1]
        z = dir[2]

        roll = 0
        pitch = math.atan2(-z, y)
        yaw = math.atan2(y, x)
        q = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw, axes='rxyz')
        return Quaternion(q[0], q[1], q[2], q[3])

    def transform_box2map(self, points):
        """
        transforms a list of points from the box to the map

        params:
            points -> [PointStamped] array of points to transform

        returns:
            trans_list ->
        """

        trans_list = []
        for b in points:
            point = PointStamped()
            point.header.frame_id = self.box_frame
            point.point.x = b[0]
            point.point.y = b[1]
            point.point.z = b[2]

            trans_p = self.listener.transformPoint('map', point)
            trans_list.append([trans_p.point.x, trans_p.point.y, trans_p.point.z])
        return trans_list



if __name__ == "__main__":
    try:
        InfoEntropy()
    except rospy.ROSInterruptException:
        pass