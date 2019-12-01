import cv2
import rospy
import numpy as np
from tf.transformations import quaternion_from_euler, unit_vector, quaternion_inverse

import utils

class Intersection4wayModel():

    def __init__(self, pcm, scaled_homography, tf):
        self.pcm = pcm
        self.scaled_homography = scaled_homography
        self.tf = tf

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

        self.intersection_poses = [utils.invert_pose(p, 'stopline_{}'.format(i)) for i, p in enumerate(self.stopline_poses)]


    # Returns opencv image coordinates
    def get_stopline_centers_pixel_prediction(self):
        stopline_poses_predicted = []

        self.tf.waitForTransform('axle', 'intersection', rospy.Time(0), rospy.Duration(1))

        for i in range(len(self.stopline_poses)):
            prediction_axle_frame = self.tf.transformPose('axle', self.stopline_poses[i])
            stopline_poses_predicted.append(prediction_axle_frame.pose)

        return stopline_poses_predicted


    # Translated to python from https://docs.ros.org/api/image_geometry/html/c++/pinhole__camera__model_8cpp_source.html
    def unrectifyPoint(self, point_rectified):
        ray = self.pcm.projectPixelTo3dRay(point_rectified)
        # Project the ray on the image
        r_vec, _ = cv2.Rodrigues(self.pcm.R)
        t_vec = np.zeros((3,1))
        image_points, _ = cv2.projectPoints(np.array([ray]), r_vec, t_vec, self.pcm.K, self.pcm.D)
        point_unrectified = image_points[0][0]

        return (point_unrectified[0], point_unrectified[1]);
