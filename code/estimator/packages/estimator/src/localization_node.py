#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import rospkg

from duckietown import DTROS
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Point, Quaternion, Pose, PoseArray
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

import utils
from feature_tracker import FeatureTracker
from config_loader import get_camera_info_for_robot, get_homography_for_robot
from intersection_model import Intersection4wayModel
from scaled_homography import ScaledHomography
from stopline_detector import StoplineDetector
from image_geometry import PinholeCameraModel
from tf import TransformListener, TransformBroadcaster
from tf.transformations import quaternion_from_matrix, unit_vector
from timekeeper import TimeKeeper


class LocalizationNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LocalizationNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")

        # Initialize parameters
        rospy.set_param('~x', 0.35)
        rospy.set_param('~y', 0.0)
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

        # Subscribers
        self.sub_camera_info = self.subscriber('~camera_info', CameraInfo, self.cb_camera_info, queue_size=1)
        self.sub_reset = self.subscriber('~reset', Bool, self.cb_reset, queue_size=1)

        # Publishers
        self.pub_keypoints = self.publisher('~verbose/keypoints/compressed', CompressedImage, queue_size=1)
        self.pub_intersection = self.publisher('~verbose/intersection/compressed', CompressedImage, queue_size=1)
        self.pub_stopline_prediction = self.publisher('~verbose/stopline_prediction/compressed', CompressedImage, queue_size=1)
        self.pub_red_filter = self.publisher('~verbose/red_filter/compressed', CompressedImage, queue_size=1)
        self.pub_stopline_poses = self.publisher('~verbose/stopline_poses', PoseArray, queue_size=1)
        self.pub_stopline_poses_predicted = self.publisher('~verbose/stopline_poses_predicted', PoseArray, queue_size=1)
        self.pub_pose_estimates = self.publisher('~verbose/pose_estimates', PoseArray, queue_size=1)

        self.bridge = CvBridge()
        self.feature_tracker = FeatureTracker(400, 50, self.log)

        self.start_position = np.matrix([640/2.0, 480/4.0]).T
        self.start_angle = 180.0
        self.x = rospy.get_param('~x')
        self.cnt = 0
        self.last_seq = None

        self.tf = TransformListener()
        self.br = TransformBroadcaster()

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
            self.model = Intersection4wayModel(self.pcm, self.scaled_homography, self.tf)
            self.stopline_detector = StoplineDetector(self.pcm, self.scaled_homography, self.tf)

            # This topic subscription is only needed initially, so it can be unregistered.
            self.sub_camera_info.unregister()
            self.sub_camera_info = self.subscriber('~camera_info', CameraInfo, self.scaled_homography.cb_camera_info, queue_size=1)

            buffer_size = msg.width * msg.height * 3 * 2
            self.log('Buffer size set to {}.'.format(buffer_size), 'info')
            # Now the node can proceed to process images
            self.sub_image_in = self.subscriber('~image_in/compressed', CompressedImage, self.cb_image_in, queue_size=1, buff_size=buffer_size)

            self.log('Initialized.')


    def cb_reset(self, msg):
        do_reset = msg.data
        if do_reset:
            self.feature_tracker.reset()
            self.x = rospy.get_param('~x')
            self.cnt = 0
            self.last_seq = None


    def draw_ref_point(self, image, position, angle, length):
        # position in [px]
        # angle in [deg]

        angle_rad = np.deg2rad(angle)
        pt1 = (int(position[1]), int(position[0]))
        pt2 = (int(pt1[0] + length * np.sin(angle_rad)), int(pt1[1] + length * np.cos(angle_rad)))

        return cv2.arrowedLine(image, pt1, pt2, (0, 0, 255), 1)


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


            ##############################
            # Debug code to simulate motor commands
            seq = msg.header.seq
            if self.last_seq != None:
                seq_diff = seq - self.last_seq
                self.cnt += seq_diff
                if self.cnt > 50:
                    #pass
                    self.x -= 0.01 * seq_diff
            self.last_seq = seq

            x = self.x
            y = rospy.get_param('~y')
            z = rospy.get_param('~z')
            position = (x, y, z)
            m = np.matrix([
                [0,1,0,0],
                [-1,0,0,0],
                [0,0,1,0],
                [0,0,0,1]])
            orientation = unit_vector(quaternion_from_matrix(m))
            self.br.sendTransform(position, orientation, rospy.Time.now(), 'intersection', 'axle')
            # Debug code end
            ##############################


            stopline_poses_predicted = self.model.get_stopline_centers_pixel_prediction()
            self.publish_pose_array(self.pub_stopline_poses_predicted, 'axle', stopline_poses_predicted)
            tk.completed('predict poses')

            # Filtering red
            red_mask, dbg_image = self.stopline_detector.filter_red(img_original, verbose=self.verbose, tk=tk)
            if dbg_image is not None:
                utils.publish_image(self.bridge, self.pub_red_filter, dbg_image)

            # Clustering red points
            clusters, cluster_means = self.stopline_detector.cluster_mask(red_mask, self.parameters['~eps'], self.parameters['~min_samples'], tk=tk)
            # TODO Handle case if there are no clusters found

            if len(clusters) > 0:
                # Calculate stopline poses from clusters
                stopline_poses_measured = []
                for cluster_points in clusters:
                    pose = self.stopline_detector.calculate_pose_from_points(cluster_points)
                    stopline_poses_measured.append(pose)
                tk.completed('stopline pose calculations')

                labels = self.stopline_detector.classify_poses(stopline_poses_measured, stopline_poses_predicted)
                tk.completed('classifying poses')

                stopline_poses_corrected = []
                for pose_measured, label in zip(stopline_poses_measured, labels):
                    corrected = self.stopline_detector.correct_orientation(pose_measured, stopline_poses_predicted[label])
                    stopline_poses_corrected.append(corrected)
                self.publish_pose_array(self.pub_stopline_poses, 'axle', stopline_poses_corrected)
                tk.completed('corrected poses')

                if self.verbose:
                    img_debug = self.stopline_detector.draw_debug_image(img_original, stopline_poses_predicted, stopline_poses_corrected, clusters, labels)
                    utils.publish_image(self.bridge, self.pub_keypoints, img_debug)
                    tk.completed('debug image')

                for i, intersection_pose in enumerate(self.model.intersection_poses):
                    position, orientation = utils.pose_to_tuple(intersection_pose.pose)
                    transform = utils.get_transform(intersection_pose.header.frame_id, 'intersection_{}'.format(i), position, orientation)
                    self.tf.setTransform(transform)

                for stopline_pose, label in zip(stopline_poses_corrected, labels):
                    position, orientation = utils.pose_to_tuple(stopline_pose)
                    transform = utils.get_transform('axle', 'stopline_{}'.format(label), position, orientation)
                    self.tf.setTransform(transform)

                poses_estimated = []
                for i in range(len(self.model.intersection_poses)):
                    frame_origin_intersection = utils.get_pose('intersection_{}'.format(i), [0.0,0.0,0.0], [0.0,0.0,0.0,1])

                    pose_estimation = self.tf.transformPose('axle', frame_origin_intersection)
                    poses_estimated.append(pose_estimation.pose)

                self.publish_pose_array(self.pub_pose_estimates, 'axle', poses_estimated)
                tk.completed('pose estimations')

            self.log(tk.getall())
            return













            # NOT REACHABLE

            img_keypoints = img_original.copy()
            position, angle = self.feature_tracker.process_image(img_original, image_out_keypoints=img_keypoints)

            img_position = img_keypoints.copy()
            img_position = self.draw_ref_point(img_position, position, angle, 10)

            px_per_m_original = 500.0
            scale_x = 480.0 / 192
            scale_y = 640.0 / 256
            px_per_m_x = px_per_m_original / scale_x
            px_per_m_y = px_per_m_original / scale_y

            position_meter = np.array([0 + position[0] / px_per_m_x, 0 + position[1] / px_per_m_y])

            img_intersection = self.draw_position_on_intersection(position_meter, angle)

            #position_world = [position[0] / px_per_m_x, position[1] / px_per_m_y]

            if self.verbose:
                utils.publish_image(self.bridge, self.pub_keypoints, img_keypoints)
                utils.publish_image(self.bridge, self.pub_intersection, img_intersection)

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
