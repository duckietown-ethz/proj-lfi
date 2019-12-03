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

        orientation = unit_vector(quaternion_from_euler(0, 0, 0, axes='sxyz'))
        self.stopline_poses.append(utils.get_pose('intersection', [0.0, 0.0, 0.0], orientation))

        orientation = unit_vector(quaternion_from_euler(0, 0, -np.pi/2.0, axes='sxyz'))
        self.stopline_poses.append(utils.get_pose('intersection', [-(q/2.0+s+q+p/2.0), p/2.0+q/2.0, 0.0], orientation))

        orientation = unit_vector(quaternion_from_euler(0, 0, -np.pi, axes='sxyz'))
        self.stopline_poses.append(utils.get_pose('intersection', [-(q/2.0+s+q/2.0), p/2.0+2*q+s+p/2.0, 0.0], orientation))

        orientation = unit_vector(quaternion_from_euler(0, 0, -3.0*np.pi/2.0, axes='sxyz'))
        self.stopline_poses.append(utils.get_pose('intersection', [q/2.0+p/2.0, p/2.0+q+s+q/2.0, 0.0], orientation))


        # TODO This part can be written much more nicely
        intersection_poses = [utils.invert_pose(p.pose) for p in self.stopline_poses]
        self.intersection_poses = [utils.stamp_pose('stopline_{}'.format(i), p) for i, p in enumerate(intersection_poses)]

        self.transformer = TransformerROS()

        for i, intersection_pose in enumerate(self.intersection_poses):
            position, orientation = utils.pose_to_tuple(intersection_pose.pose)
            transform = utils.get_transform(intersection_pose.header.frame_id, 'intersection_{}'.format(i), position, orientation)
            self.transformer.setTransform(transform)


    # axle_pose needs to be in intersection coordinates
    def get_stopline_poses_reference(self, axle_pose):
        position, orientation = utils.pose_to_tuple(axle_pose.pose)
        transform = utils.get_transform('intersection', 'axle', position, orientation)
        self.transformer.setTransform(transform)

        stopline_poses_predicted = []

        for i in range(len(self.stopline_poses)):
            prediction_axle_frame = self.transformer.transformPose('axle', self.stopline_poses[i])
            stopline_poses_predicted.append(prediction_axle_frame.pose)

        return stopline_poses_predicted


    def get_axle_pose_from_stopline_pose(self, stopline_pose, index):
        position, orientation = utils.pose_to_tuple(stopline_pose)
        transform = utils.get_transform('axle', 'stopline_{}'.format(index), position, orientation)
        self.transformer.setTransform(transform)

        axle_origin = utils.get_pose('axle', [0.0,0.0,0.0], [0.0,0.0,0.0,1])
        # TODO Can this be done with only one intersection frame?
        axle_pose = self.transformer.transformPose('intersection_{}'.format(index), axle_origin)
        return axle_pose
