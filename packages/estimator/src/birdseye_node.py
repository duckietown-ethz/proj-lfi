#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from duckietown_msgs.msg import BoolStamped
from duckietown import DTROS
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from math import floor
from image_geometry import PinholeCameraModel

import utils
from config_loader import get_camera_info_for_robot, get_homography_for_robot
from scaled_homography import ScaledHomography


class BirdseyeNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(BirdseyeNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")

        # XXX: do we really need a member variable for every field in the dict?
        self.verbose = None
        self.parameters['~verbose'] = None
        self.rectify = None
        self.parameters['~rectify'] = None
        self.parameters['~image_size'] = None

        self.updateParameters()
        self.refresh_parameters()

        self.pcm = None # initialized when CameraInfo is received
        self.bridge = CvBridge()

        self.active = True

        # Load default camera calibration
        H = get_homography_for_robot(self.veh_name)
        cam_info = get_camera_info_for_robot(self.veh_name)
        self.scaled_homography = ScaledHomography(H, cam_info.height, cam_info.width)

        # Scale the homography based on actual resolution requested from camera_node
        self.image_size_height = self.parameters['~image_size']['height']
        self.image_size_width = self.parameters['~image_size']['width']
        self.scaled_homography.update_homography(self.image_size_height, self.image_size_width)

        rospy.set_param('/{}/camera_node/res_w'.format(self.veh_name), self.image_size_width)
        rospy.set_param('/{}/camera_node/res_h'.format(self.veh_name), self.image_size_height)
        self.log('Waiting for CameraInfo matching requested resolution: %dx%d'%(self.image_size_width,self.image_size_height))

        # Subscribers
        self.sub_switch = self.subscriber("~switch", BoolStamped, self.cbSwitch, queue_size=1)
        self.sub_camera_info = self.subscriber('~camera_info', CameraInfo, self.cb_camera_info, queue_size=1)
        self.sub_image_in = None # initialized when CameraInfo is received

        # Publishers
        self.pub_rectified = self.publisher('~verbose/rectified/compressed', CompressedImage, queue_size=1)
        self.pub_image_out = self.publisher('~image_out/compressed', CompressedImage, queue_size=1)

    def cb_camera_info(self, msg):
        # wait for the camera_node to switch to the requested resolution
        if self.image_size_width != msg.width or self.image_size_height != msg.height:
           return
        self.log('Received updated camera info.', 'info')

        # This topic subscription is only needed initially, so it can be unregistered.
        self.sub_camera_info.unregister()
        self.pcm = PinholeCameraModel()
        self.pcm.fromCameraInfo(msg)

        buffer_size = msg.width * msg.height * 3 * 2 # 2 is a safety factor
        self.log('Buffer size set to {}.'.format(buffer_size), 'info')
        # Now the node can proceed to process images
        self.sub_image_in = self.subscriber('~image_in/compressed', CompressedImage, self.cb_image_in, queue_size=1, buff_size=buffer_size)

        self.log('Initialized.')


    def cbSwitch(self, switch_msg):
        self.log('Switch ' + str(switch_msg.data))
        self.active = switch_msg.data


    def cb_image_in(self, msg):
        if self.active and not self.is_shutdown:

            if self.verbose:
                self.log('Received image.', 'info')

            if self.parametersChanged:
                self.log('Parameters changed.', 'info')
                self.refresh_parameters()
                self.parametersChanged = False

            img = utils.read_image(msg)
            if img is None:
                self.log('Got empty image msg.')
            height, width, depth = img.shape

            if self.rectify:
                img_temp = img.copy()
                self.pcm.rectifyImage(img_temp, img_temp)
                img = img_temp
                if self.verbose:
                    utils.publish_image(self.bridge, self.pub_rectified, img)

            img_out = self.camera_img_to_birdseye(img)

            utils.publish_image(self.bridge, self.pub_image_out, img_out, msg.header)


    def camera_img_to_birdseye(self, cam_img):
        # Update homography based on image size
        height, width, _ = cam_img.shape
        if height != self.image_size_height:
            self.scaled_homography.update_homography(self.image_size_height, self.image_size_width)

        H = self.scaled_homography.for_image()
        # Apply image transformation
        # TODO: Test different flags (https://docs.opencv.org/3.1.0/da/d54/group__imgproc__transform.html#ga5bb5a1fea74ea38e1a5445ca803ff121 )
        birdseye_img = cv2.warpPerspective(cam_img, H, (width, height), flags=cv2.INTER_LINEAR+cv2.WARP_FILL_OUTLIERS)
        return birdseye_img


    def refresh_parameters(self):
        self.verbose = self.parameters['~verbose']
        self.rectify = self.parameters['~rectify']


    def onShutdown(self):
        self.log("Stopping birdseye_node.")
        super(BirdseyeNode, self).onShutdown()


if __name__ == '__main__':
    # Initialize the node
    birdseye_node = BirdseyeNode(node_name='birdseye_node') # Keep it spinning to keep the node alive
    rospy.spin()
