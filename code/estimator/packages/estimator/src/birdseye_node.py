#!/usr/bin/env python

import cv2
import numpy as np
import rospy

from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import utils


class BirdseyeNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(BirdseyeNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")

        self.parameters['~verbose'] = None
        self.updateParameters()
        self.refresh_parameters()

        # Subscribers
        self.sub_image_in = self.subscriber('~image_in/compressed', CompressedImage, self.cb_image_in, queue_size=1)

        # Publishers
        self.pub_warped = self.publisher('~warped/compressed', CompressedImage, queue_size=1)
        self.pub_image_out = self.publisher('~image_out/compressed', CompressedImage, queue_size=1)

        self.bridge = CvBridge()

        # duckymcduckface
        self.homography = np.matrix([
            [1.8886299808681545e-05, -0.0002247954313914998, -0.1783985372127643],
            [0.0008637636479351076, 1.107464752716367e-06, -0.26938728058395345],
            [5.021859748339636e-05,-0.006789974261768175, 1.0]
        ])
        #wlan
        self.homography = np.matrix([
            [-1.1907434447195475e-05, -0.00016985225547642657, -0.18018639992319468],
            [0.0008110438997760144, 2.9640247271729815e-07, -0.2609339693203626],
            [-5.837794811070778e-05, -0.006471722102967347, 1.0]
        ])


        self.log("Initialized")


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

            img_warped = self.camera_img_to_birdseye(img_original, self.homography)

            if self.verbose:
                utils.publish_image(self.bridge, self.pub_warped, img_warped)

            img_blurred = cv2.GaussianBlur(img_warped, (0,0), 3)
            img_sharpened = cv2.addWeighted(img_warped, 1.5, img_blurred, -0.5, 0)

            img_out = img_sharpened
            utils.publish_image(self.bridge, self.pub_image_out, img_out)


    # TODO Make this function work if the horizon has been cut off.
    def camera_img_to_birdseye(self, cam_img, homography):
        height, width, _ = cam_img.shape
        px_per_m_original = 500.0

        H = homography

        # Scale homography to the size of the camera image, as it assumes a size of (480,640)
        scale_x = 480.0 / height
        scale_y = 640.0 / width
        px_per_m_x = px_per_m_original / scale_x
        px_per_m_y = px_per_m_original / scale_y
        scaler_mat = np.hstack([np.ones((3,1))*scale_x, np.ones((3,1))*scale_y, np.ones((3,1))])
        H = np.multiply(scaler_mat, H)

        # Scale axle coordinates to pixels
        scaler_mat = np.vstack([np.ones((1,3))*px_per_m_x, np.ones((1,3))*px_per_m_y, np.ones((1,3))])
        H = np.multiply(scaler_mat, H)

        # Size (in pixels) of the resulting transformed image
        size = (height, width)

        # Center the axle x-axis in the image
        translation = np.eye(3, 3)
        translation[0, 2] += 0.0
        translation[1, 2] += size[1] / 2
        H = translation.dot(H)
        H /= H[2, 2]

        # Apply image transformation
        birdseye_img = cv2.warpPerspective(cam_img, H, size)

        # Rotate and transpose for correct orientation after transformation
        (h, w) = birdseye_img.shape[:2]
        center = (w / 2, h / 2)
        rotMat = cv2.getRotationMatrix2D(center, 180, 1.0)
        birdseye_img = cv2.warpAffine(birdseye_img, rotMat, (w, h))
        birdseye_img = cv2.transpose(birdseye_img)

        return birdseye_img


    def refresh_parameters(self):
        self.verbose = self.parameters['~verbose']


    def onShutdown(self):
        self.log("Stopping birdseye_node.")

        super(PreprocessorNode, self).onShutdown()



if __name__ == '__main__':
    # Initialize the node
    birdseye_node = BirdseyeNode(node_name='birdseye_node') # Keep it spinning to keep the node alive
    rospy.spin()
