#!/usr/bin/env python

import cv2
import numpy as np
import rospy

from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from math import floor

import utils


class PreprocessorNode(DTROS):
    """
    Tasks of this node:
    - (TODO) Set parameters of camera_node (resolution)
    - (done) Cutting off horizon
    - (TODO) Improve image contrast
    - (TODO) Image rectification
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

        # Subscribers
        self.sub_image_in = self.subscriber('~image_in/compressed', CompressedImage, self.cb_image_in, queue_size=1)

        # Publishers
        self.pub_cutoff = self.publisher('~verbose/cutoff/compressed', CompressedImage, queue_size=1)
        self.pub_rectified = self.publisher('~verbose/rectified/compressed', CompressedImage, queue_size=1)
        self.pub_image_out = self.publisher('~image_out/compressed', CompressedImage, queue_size=1)

        # Initialize rest of the node
        self.bridge = CvBridge()

        # TODO Set camera_node parameters

        self.log('Initialized')


    def cb_image_in(self, msg):
        if not self.is_shutdown:
            # TODO Change to debug level
            self.log('Received image.', 'info')

            if self.parametersChanged:
                self.log('Parameters changed.', 'info')
                self.refresh_parameters()
                self.parametersChanged = False

            img_original = utils.read_image(self.bridge, msg)
            if img_original == None:
                return

            height, width, _ = img_original.shape
            cutoff_absolute = int(floor(height * self.top_cutoff_percent / 100.0))
            img_cutoff = img_original[cutoff_absolute:,:]

            if self.verbose:
                utils.publish_image(self.bridge, self.pub_cutoff, img_cutoff)

            # TODO Rectify image using camera info
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
