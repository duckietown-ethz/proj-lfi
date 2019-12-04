#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import rospkg

from duckietown import DTROS
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Point, Quaternion, PoseStamped, PoseArray
from std_msgs.msg import Bool
from duckietown_msgs.msg import Pose2DStamped
from cv_bridge import CvBridge, CvBridgeError

import utils
from config_loader import get_camera_info_for_robot, get_homography_for_robot
from intersection_model import Intersection4wayModel
from scaled_homography import ScaledHomography
from stopline_detector import StoplineDetector
from stopline_filter import StoplineFilter
from image_geometry import PinholeCameraModel
from tf.transformations import unit_vector, quaternion_from_euler, euler_from_quaternion
from timekeeper import TimeKeeper


class LocalizationNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LocalizationNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")

        # Initialize parameters
        rospy.set_param('~x', 0.0)
        rospy.set_param('~y', -0.20)
        rospy.set_param('~z', 0.0)

        rospy.set_param('~eps', 0.5)
        self.parameters['~eps'] = None
        rospy.set_param('~min_samples', 4)
        self.parameters['~min_samples'] = None

        self.parameters['~verbose'] = None
        self.parameters['/{}/preprocessor_node/image_size'.format(self.veh_name)] = None
        self.parameters['/{}/preprocessor_node/top_cutoff_percent'.format(self.veh_name)] = None
        self.updateParameters()
        self.refresh_parameters()

        self.pose_in = None # latest input pose (open loop)
        self.prev_pose_in = None # input pose when the last estimate was made

        # Subscribers
        self.sub_camera_info = self.subscriber('~camera_info', CameraInfo, self.cb_camera_info, queue_size=1)
        self.sub_reset = self.subscriber('~reset', Bool, self.cb_reset, queue_size=1)
        self.sub_pose_in = self.subscriber('~open_loop_pose_estimate', Pose2DStamped, self.cb_pose_in, queue_size=1)

        # Publishers
        self.pub_clustering = self.publisher('~verbose/clustering/compressed', CompressedImage, queue_size=1)
        self.pub_red_filter = self.publisher('~verbose/red_filter/compressed', CompressedImage, queue_size=1)
        self.pub_stoplines_measured = self.publisher('~stoplines_measured', PoseArray, queue_size=1)
        self.pub_stoplines_predicted = self.publisher('~stoplines_predicted', PoseArray, queue_size=1)
        self.pub_pose_estimates = self.publisher('~pose_estimates', PoseArray, queue_size=1)
        self.pub_best_pose_estimate = self.publisher('~best_pose_estimate', PoseStamped, queue_size=1)

        self.bridge = CvBridge()

        self.last_position = None
        self.last_orientation = None

        self.scaled_homography = None

        self.log('Waiting for camera to update its parameters.')

    def cb_camera_info(self, msg):
        if self.image_size_width == msg.width and self.image_size_height == msg.height:
            self.log('Received camera info.', 'info')
            self.pcm = PinholeCameraModel()
            self.pcm.fromCameraInfo(msg)
            homography = get_homography_for_robot(self.veh_name)
            camera_info = get_camera_info_for_robot(self.veh_name)
            self.scaled_homography = ScaledHomography(homography, camera_info.height, camera_info.width)
            self.model = Intersection4wayModel(self.pcm, self.scaled_homography)
            self.stopline_detector = StoplineDetector(self.scaled_homography, point_reduction_factor=1)
            self.stopline_filter = StoplineFilter(min_quality=0.5, policy='weighted_avg')

            # This topic subscription is only needed initially, so it can be unregistered.
            self.sub_camera_info.unregister()
            self.sub_camera_info = self.subscriber('~camera_info', CameraInfo, self.scaled_homography.cb_camera_info, queue_size=1)

            buffer_size = msg.width * msg.height * 3 * 2
            self.log('Buffer size set to {}.'.format(buffer_size), 'info')
            # Now the node can proceed to process images
            self.sub_image_in = self.subscriber('~image_in/compressed', CompressedImage, self.cb_image_in, queue_size=1, buff_size=buffer_size)

            self.log('Initialized.')


    # The received pose will be used to compute expected stop line positions
    def cb_pose_in(self, msg):
        if msg is not None: # XXX: I believe it can't be anyway
            self.pose_in = msg

    def cb_reset(self, msg):
        do_reset = msg.data
        if do_reset:
            self.last_position = None
            self.last_orientation = None

    def cb_image_in(self, msg):
        if not self.is_shutdown:
            # TODO Change to debug level
            self.log('Received image.', 'info')

            tk = TimeKeeper(msg)

            if self.parametersChanged:
                self.log('Parameters changed.', 'info')
                self.refresh_parameters()
                self.parametersChanged = False

            tk.completed('parameter check')
            img_original = utils.read_image(msg)
            if img_original is None:
                return

            if self.last_position is None or self.last_orientation is None:
                # Set initial position
                start_orientation = unit_vector(quaternion_from_euler(0, 0, np.pi/2.0, axes='sxyz'))
                start_position = [rospy.get_param('~x'), rospy.get_param('~y'), rospy.get_param('~z')]
                axle_pose = utils.pose(start_position, start_orientation)
            else:
                axle_pose = utils.pose('intersection', self.last_position, self.last_orientation)
                if self.prev_pose_in is not None:
                    axle_pose.pose.position.x += (self.pose_in.x - self.prev_pose_in.x)
                    axle_pose.pose.position.y += (self.pose_in.y - self.prev_pose_in.y)
                    prev_theta = euler_from_quaternion(utils.quat_to_tuple(axle_pose.pose.orientation), axes='sxyz')[2]
                    delta_theta = self.pose_in.theta - self.prev_pose_in.theta
                    theta = prev_theta + delta_theta
                    axle_pose.pose.orientation = unit_vector(quaternion_from_euler(0, 0, theta, axes='sxyz'))

            self.prev_pose_in = self.pose_in

            stopline_poses_predicted = self.model.get_stopline_poses_reference(axle_pose)
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


    def publish_pose_array(self, publisher, frame_id, poses):
        pose_array = PoseArray()
        pose_array.header.frame_id = frame_id
        pose_array.header.stamp = rospy.Time.now()
        pose_array.poses = poses
        publisher.publish(pose_array)


    def refresh_parameters(self):
        self.verbose = self.parameters['~verbose']
        image_size = self.parameters['/{}/preprocessor_node/image_size'.format(self.veh_name)]
        self.image_size_height = image_size['height']
        self.image_size_width = image_size['width']
        self.top_cutoff_percent = self.parameters['/{}/preprocessor_node/top_cutoff_percent'.format(self.veh_name)]


    def onShutdown(self):
        self.log("Stopping preprocessor_node.")

        super(LocalizationNode, self).onShutdown()


if __name__ == '__main__':
    # Initialize the node
    localization_node = LocalizationNode(node_name='localization_node') # Keep it spinning to keep the node alive
    rospy.spin()
