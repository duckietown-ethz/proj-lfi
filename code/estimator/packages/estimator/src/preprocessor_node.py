#!/usr/bin/env python

import cv2
import numpy as np
import rospy

from duckietown import DTROS
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from math import floor
from image_geometry import PinholeCameraModel

import utils
from config_loader import get_camera_info_for_robot, get_homography_for_robot


class PreprocessorNode(DTROS):
    """
    Tasks of this node:
    - (done) Set parameters of camera_node (resolution)
    - (done) Cutting off horizon
    - (TODO) Improve image contrast
    - (done) Image rectification
    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(PreprocessorNode, self).__init__(node_name=node_name, parameters_update_period=10.0)
        self.veh_name = rospy.get_namespace().strip("/")

        # Initialize parameters
        self.parameters['~verbose'] = None
        self.parameters['~image_size'] = None
        self.parameters['~top_cutoff_percent'] = None
        self.updateParameters()
        self.refresh_parameters()

        # Load camera calibration
        self.camera_info = get_camera_info_for_robot(self.veh_name)
	self.pcm = None

        # Subscribers
        self.sub_camera_info = self.subscriber('~camera_info', CameraInfo, self.cb_camera_info, queue_size=1)

        # Publishers
        self.pub_cutoff = self.publisher('~verbose/cutoff/compressed', CompressedImage, queue_size=1)
        self.pub_rectified = self.publisher('~verbose/rectified/compressed', CompressedImage, queue_size=1)
        self.pub_image_out = self.publisher('~image_out/compressed', CompressedImage, queue_size=1)

        # Initialize rest of the node
        self.bridge = CvBridge()

        # Configure camera
        self.camera_width = int(640/2.5)
        self.camera_height = int(480/2.5)

        rospy.set_param('/{}/camera_node/exposure_mode'.format(self.veh_name), 'off')
        rospy.set_param('/{}/camera_node/res_w'.format(self.veh_name), self.camera_width)
        rospy.set_param('/{}/camera_node/res_h'.format(self.veh_name), self.camera_height)

        self.log('Waiting for camera to update its parameters.')


    def cb_camera_info(self, msg):
        if self.camera_width == msg.width and self.camera_height == msg.height:
            self.log('Received camera info.', 'info')
	    self.pcm = PinholeCameraModel()
            self.pcm.fromCameraInfo(msg)

            # This topic subscription is only needed initially, so it can be unregistered.
            self.sub_camera_info.unregister()

            buffer_size = msg.width * msg.height * 3 * 2
            self.log('Buffer size set to {}.'.format(buffer_size), 'info')
            # Now the node can proceed to process images
            self.sub_image_in = self.subscriber('~image_in/compressed', CompressedImage, self.cb_image_in, queue_size=1, buff_size=buffer_size)

            self.log('Initialized.')


    def cb_image_in(self, msg):
        if not self.is_shutdown and self.pcm != None:
            # TODO Change to debug level
            self.log('Received image.', 'info')

            if self.parametersChanged:
                self.log('Parameters changed.', 'info')
                self.refresh_parameters()
                self.parametersChanged = False

            img_original = utils.read_image(msg)
            if img_original == None:
                return

            img_rectified = img_original.copy()
            self.pcm.rectifyImage(img_rectified, img_rectified)

            if self.verbose:
                utils.publish_image(self.bridge, self.pub_rectified, img_rectified)

            height, width, _ = img_rectified.shape
            cutoff_absolute = int(floor(height * self.top_cutoff_percent / 100.0))
            img_cutoff = img_rectified[cutoff_absolute:,:]

            if self.verbose:
                utils.publish_image(self.bridge, self.pub_cutoff, img_cutoff)

            # TODO Improve image contrast

            img_out = img_cutoff
            utils.publish_image(self.bridge, self.pub_image_out, img_out)


    def refresh_parameters(self):
        self.verbose = self.parameters['~verbose']
        image_size = self.parameters['~image_size']
        self.image_size_height = image_size['height']
        self.image_size_width = image_size['width']
        self.top_cutoff_percent = self.parameters['~top_cutoff_percent']


    def onShutdown(self):
        self.log("Stopping preprocessor_node.")

        super(PreprocessorNode, self).onShutdown()


if __name__ == '__main__':
    # Initialize the node
    preprocessor_node = PreprocessorNode(node_name='preprocessor_node') # Keep it spinning to keep the node alive
    rospy.spin()
