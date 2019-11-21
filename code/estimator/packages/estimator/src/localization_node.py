#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import rospkg

from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point, Quaternion, Pose
from std_msgs.msg import Bool
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

        # Load camera calibration
        self.homography = get_homography_for_robot(self.veh_name)

        # Subscribers
        buffer_size = 294912 # TODO Set this dynamically based on the image size.
        self.sub_image_in = self.subscriber('~image_in/compressed', CompressedImage, self.cb_image_in, queue_size=1, buff_size=buffer_size)
        self.sub_reset = self.subscriber('~reset', Bool, self.cb_reset, queue_size=1)

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


    def refresh_parameters(self):
        self.verbose = self.parameters['~verbose']


    def onShutdown(self):
        self.log("Stopping preprocessor_node.")

        super(LocalizationNode, self).onShutdown()


if __name__ == '__main__':
    # Initialize the node
    localization_node = LocalizationNode(node_name='localization_node') # Keep it spinning to keep the node alive
    rospy.spin()
