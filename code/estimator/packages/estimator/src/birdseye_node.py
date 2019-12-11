#!/usr/bin/env python

import cv2
import numpy as np
import rospy

from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import utils
from config_loader import get_camera_info_for_robot, get_homography_for_robot
from scaled_homography import ScaledHomography


class BirdseyeNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(BirdseyeNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")

        self.parameters['~verbose'] = None
        self.updateParameters()
        self.refresh_parameters()

        # Load camera calibration
        homography = get_homography_for_robot(self.veh_name)
        camera_info = get_camera_info_for_robot(self.veh_name)
        self.scaled_homography = ScaledHomography(homography, camera_info.height, camera_info.width)

        # Subscribers
        buffer_size = 294912 # TODO Set this dynamically based on the image size.
        self.sub_image_in = self.subscriber('~image_in/compressed', CompressedImage, self.cb_image_in, queue_size=1, buff_size=buffer_size)

        # Publishers
        self.pub_warped = self.publisher('~warped/compressed', CompressedImage, queue_size=1)
        self.pub_image_out = self.publisher('~image_out/compressed', CompressedImage, queue_size=1)

        self.bridge = CvBridge()

        self.log("Initialized")


    def cb_image_in(self, msg):
        if not self.is_shutdown:
            # TODO Change to debug level
            if self.verbose:
                self.log('Received image.', 'info')

            if self.parametersChanged:
                self.log('Parameters changed.', 'info')
                self.refresh_parameters()
                self.parametersChanged = False

            img_original = utils.read_image(msg)
            if img_original is None:
                return

            img_warped = self.camera_img_to_birdseye(img_original)

            if self.verbose:
                utils.publish_image(self.bridge, self.pub_warped, img_warped)

            # TODO Make these values parameters
            img_blurred = cv2.GaussianBlur(img_warped, (0,0), 3)
            img_sharpened = cv2.addWeighted(img_warped, 1.5, img_blurred, -0.5, 0)

            img_out = img_sharpened
            utils.publish_image(self.bridge, self.pub_image_out, img_out, msg.header)


    # TODO Make this function work if the horizon has been cut off.
    def camera_img_to_birdseye(self, cam_img):
        # Update homography based on image size
        height, width, _ = cam_img.shape
        self.scaled_homography.update_homography(height, width)
        scaled_homography = self.scaled_homography.for_image()

        # Apply image transformation
        # TODO Test different flags on duckiebot (https://docs.opencv.org/3.1.0/da/d54/group__imgproc__transform.html#ga5bb5a1fea74ea38e1a5445ca803ff121 )
        birdseye_img = cv2.warpPerspective(cam_img, scaled_homography, (width, height), flags=cv2.INTER_LINEAR+cv2.WARP_FILL_OUTLIERS)

        return birdseye_img


    def refresh_parameters(self):
        self.verbose = self.parameters['~verbose']


    def onShutdown(self):
        self.log("Stopping birdseye_node.")

        super(BirdseyeNode, self).onShutdown()


if __name__ == '__main__':
    # Initialize the node
    birdseye_node = BirdseyeNode(node_name='birdseye_node') # Keep it spinning to keep the node alive
    rospy.spin()
