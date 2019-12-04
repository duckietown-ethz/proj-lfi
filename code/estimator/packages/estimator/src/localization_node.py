#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import rospkg

from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point, Quaternion, Pose
from std_msgs.msg import Bool
from duckietown_msgs.msg import Pose2DStamped
from cv_bridge import CvBridge, CvBridgeError

import utils
from feature_tracker import FeatureTracker
from config_loader import get_camera_info_for_robot, get_homography_for_robot


class LocalizationNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LocalizationNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")

        # Initialize parameters
        self.parameters['~verbose'] = None
        self.updateParameters()
        self.refresh_parameters()

        self.pose_in = None # latest input pose (open loop)
        self.prev_pose_in = None # input pose when the last estimate was made

        # Subscribers
        buffer_size = 294912 # TODO Set this dynamically based on the image size.
        self.sub_image_in = self.subscriber('~image_in/compressed', CompressedImage, self.cb_image_in, queue_size=1, buff_size=buffer_size)
        self.sub_reset = self.subscriber('~reset', Bool, self.cb_reset, queue_size=1)
        self.sub_pose_in = self.subscriber('~open_loop_pose_estimate', Pose2DStamped, self.cb_pose_in, queue_size=1)

        # Publishers
        self.pub_keypoints = self.publisher('~verbose/keypoints/compressed', CompressedImage, queue_size=1)
        self.pub_intersection = self.publisher('~verbose/intersection/compressed', CompressedImage, queue_size=1)

        self.bridge = CvBridge()
        self.feature_tracker = FeatureTracker(400, 50, self.log)

        self.start_position = np.matrix([640/2.0, 480/4.0]).T
        self.start_angle = 180.0

        self.log("Initialized")

    def cb_reset(self, msg):
        do_reset = msg.data
        if do_reset:
            self.feature_tracker.reset()

    def draw_ref_point(self, image, position, angle, length):
        # position in [px]
        # angle in [deg]

        angle_rad = np.deg2rad(angle)
        pt1 = (int(position[1]), int(position[0]))
        pt2 = (int(pt1[0] + length * np.sin(angle_rad)), int(pt1[1] + length * np.cos(angle_rad)))

        return cv2.arrowedLine(image, pt1, pt2, (0, 0, 255), 1)

    # The received pose will be used to compute expected stop line positions
    def cb_pose_in(self, msg):
        if msg.data is not None: # XXX: I believe it can't be anyway
            self.pose_in = msg.data

    def cb_image_in(self, msg):
        if not self.is_shutdown:
            # TODO Change to debug level
            self.log('Received image.', 'info')

            if self.parametersChanged:
                self.log('Parameters changed.', 'info')
                self.refresh_parameters()
                self.parametersChanged = False

            img_original = utils.read_image(msg)
            if img_original == None:
                return

            if self.last_position is None or self.last_orientation is None:
                # Set initial position
                start_orientation = unit_vector(quaternion_from_euler(0, 0, np.pi/2.0, axes='sxyz'))
                start_position = [rospy.get_param('~x'), rospy.get_param('~y'), rospy.get_param('~z')]
                axle_pose = utils.get_pose('intersection', start_position, start_orientation)
            else:
                axle_pose = utils.get_pose('intersection', self.last_position, self.last_orientation)
                if self.prev_pose_in is not None:
                    axle_pose.x += (self.pose_in.x - self.prev_pose_in.x)
                    axle_pose.y += (self.pose_in.y - self.prev_pose_in.y)
                    axle_pose.theta += (self.pose_in.theta - self.prev_pose_in.theta)
            self.prev_pose_in = self.pose_in

            stopline_poses_predicted = self.model.get_stopline_centers_pixel_prediction(axle_pose)
            self.publish_pose_array(self.pub_stoplines_predicted, 'axle', stopline_poses_predicted)
            tk.completed('predict poses')

            # Filtering red
            red_mask, dbg_image = self.stopline_detector.filter_red(img_original, verbose=self.verbose, tk=tk)
            if dbg_image is not None:
                utils.publish_image(self.bridge, self.pub_red_filter, dbg_image)

            # Clustering red points
            clusters = self.stopline_detector.cluster_mask(red_mask, self.parameters['~eps'], self.parameters['~min_samples'], tk=tk)

            # Calculate quality indicators of clusters
            cluster_qualities = list([self.stopline_detector.cluster_quality(c) for c in clusters])

            # TODO Handle case if there are no clusters found
            if len(clusters) > 0:
                # Calculate stopline poses from clusters
                stopline_poses_measured = []
                for cluster_points in clusters:
                    pose = self.stopline_detector.calculate_pose_from_points(cluster_points)
                    stopline_poses_measured.append(pose)
                tk.completed('stopline pose calculations')

                # Classify measured poses based on the predicted poses
                labels = self.stopline_detector.classify_poses(stopline_poses_measured, stopline_poses_predicted)
                tk.completed('classifying poses')

                # Correct orientation of measured poses based on the predicted poses
                stopline_poses_corrected = []
                for pose_measured, label in zip(stopline_poses_measured, labels):
                    corrected = self.stopline_detector.correct_orientation(pose_measured, stopline_poses_predicted[label])
                    stopline_poses_corrected.append(corrected)
                self.publish_pose_array(self.pub_stoplines_measured, 'axle', stopline_poses_corrected)
                tk.completed('corrected poses')

                if self.verbose:
                    # TODO Write cluster quality in cluster center
                    img_debug = self.stopline_detector.draw_debug_image(img_original, stopline_poses_predicted, stopline_poses_corrected, clusters, labels)
                    utils.publish_image(self.bridge, self.pub_clustering, img_debug)
                    tk.completed('debug image')

                # Calculate pose estimate of axle in intersection coordinate frame
                poses_estimated = []
                for stopline_pose, label in zip(stopline_poses_corrected, labels):
                    pose_estimation = self.model.get_axle_pose_from_stopline_pose(stopline_pose, label)
                    poses_estimated.append(pose_estimation.pose)

                self.publish_pose_array(self.pub_pose_estimates, 'intersection', poses_estimated)
                tk.completed('pose estimations')

                # Filter pose estimates
                self.stopline_filter.update_stopline_poses(poses_estimated, cluster_qualities)
                best_pose_estimate = self.stopline_filter.get_best_estimate()

                if best_pose_estimate is not None:
                    self.pub_best_pose_estimate.publish(utils.stamp_pose('intersection', best_pose_estimate))

                    position, orientation = utils.pose_to_tuple(best_pose_estimate)
                    self.last_position = position
                    self.last_orientation = orientation


            self.log(tk.getall())

    def draw_position_on_intersection(self, position_meter, angle):
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('estimator')
        path = self.pkg_path + "/src/four-way-intersection.png"
        image = cv2.imread(path)
        height, width, _ = image.shape
        image = cv2.copyMakeBorder(image, height/2, height/2, width/2, width/2, cv2.BORDER_CONSTANT, None, (0,0,0))
        height_m = 0.61
        width_m = 0.61

        offset = np.array([height/2 + height, width/2 + (width*3)/4])
        position_intersection = - position_meter * np.array([height / height_m, width / width_m])
        position_image = offset + position_intersection.astype(int)

        length = 100
        angle_rad = np.deg2rad(180.0 - angle)
        pt1 = (position_image[1], position_image[0])
        pt2 = (int(pt1[0] + length * np.sin(angle_rad)), int(pt1[1] + length * np.cos(angle_rad)))

        image = cv2.arrowedLine(image, pt1, pt2, (0,255,0), 5)

        return image


    def refresh_parameters(self):
        self.verbose = self.parameters['~verbose']


    def onShutdown(self):
        self.log("Stopping preprocessor_node.")

        super(LocalizationNode, self).onShutdown()


if __name__ == '__main__':
    # Initialize the node
    localization_node = LocalizationNode(node_name='localization_node') # Keep it spinning to keep the node alive
    rospy.spin()
