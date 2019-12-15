import cv2
import rospy
import numpy as np
from tf.transformations import quaternion_from_euler, unit_vector, quaternion_inverse
from tf import TransformerROS

import utils


class Intersection4wayModel():

    def __init__(self, pcm, scaled_homography):
        self.pcm = pcm
        self.scaled_homography = scaled_homography

        # Origin of ther intersection coordinate system is the center of stopline 0.
        tile_side_length = 0.61
        lane_width = 0.205
        stopline_thickness = 0.048
        center_markings_thickness = 0.025

        q = lane_width
        p = stopline_thickness
        s = center_markings_thickness

        self.stopline_poses = []

        self.tf1 = TransformerROS()
        self.tf2 = TransformerROS()

        def add_stopline_transform(idx, position, angle):
            orientation = unit_vector(quaternion_from_euler(0, 0, angle, axes='sxyz'))
            pose = utils.pose(position, orientation)

            # Add transform for reference stopline poses
            child_frame = self.stopline_frame(idx)
            parent_frame = 'intersection'
            transform = utils.transform_from_pose('intersection', child_frame, pose)
            self.tf1.setTransform(transform)

            # Add transform for measured stopline poses
            child_frame = self.intersection_frame(idx)
            parent_frame = self.stopline_frame(idx)
            inverted_pose = utils.invert_pose(utils.pose(position, orientation))
            transform = utils.transform_from_pose(parent_frame, child_frame, inverted_pose)
            self.tf2.setTransform(transform)

        add_stopline_transform(0, [0.0, 0.0, 0.0], 0)
        add_stopline_transform(1, [-(q/2.0+s+q+p/2.0), p/2.0+q/2.0, 0.0], -np.pi/2.0)
        add_stopline_transform(2, [-(q/2.0+s+q/2.0), p/2.0+2*q+s+p/2.0, 0.0], -np.pi)
        add_stopline_transform(3, [q/2.0+p/2.0, p/2.0+q+s+q/2.0, 0.0], -3.0*np.pi/2.0)


    def stopline_frame(self, idx):
        return 'stopline_' + str(idx)


    def intersection_frame(self, idx):
        return 'intersection_' + str(idx)


    # axle_pose needs to be in intersection coordinates
    def get_stopline_poses_reference(self, axle_pose):
        transform = utils.transform_from_pose('intersection', 'axle', axle_pose)
        self.tf1.setTransform(transform)

        stopline_poses = [self.tf1.transformPose('axle', utils.origin_pose(self.stopline_frame(i))).pose for i in range(4)]

        return stopline_poses


    def get_axle_pose_from_stopline_pose(self, stopline_pose, idx):
        transform = utils.transform_from_pose('axle', self.stopline_frame(idx), stopline_pose)
        self.tf2.setTransform(transform)

        axle_origin = utils.origin_pose('axle')
        axle_pose = self.tf2.transformPose(self.intersection_frame(idx), axle_origin)
        return axle_pose
