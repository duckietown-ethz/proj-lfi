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

# takes kp in pixel coordinates for img
def overlay_kp(img, kpcoords, color = (100,100,100)):
    #out = np.copy(img)
    for kp in kpcoords:
        pix  = (int(kp[0]),int(kp[1]))
        cv2.drawMarker(img, pix, color,
                       markerType=cv2.MARKER_SQUARE,
                       markerSize=10, thickness=1, line_type=cv2.LINE_AA)

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
        self.camera_width = int(640)
        self.camera_height = int(480)

        rospy.set_param('/{}/camera_node/exposure_mode'.format(self.veh_name), 'off')
        rospy.set_param('/{}/camera_node/res_w'.format(self.veh_name), self.camera_width)
        rospy.set_param('/{}/camera_node/res_h'.format(self.veh_name), self.camera_height)

        self.fast = cv2.FastFeatureDetector_create(
                                    nonmaxSuppression = False,
                                    #type=cv2.FAST_FEATURE_DETECTOR_TYPE_9_16,
                                    #type=cv2.FAST_FEATURE_DETECTOR_TYPE_5_8,
                                    #type=cv2.FAST_FEATURE_DETECTOR_TYPE_7_12,
                                    #threshold = 254
                                    )

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
            hsv = cv2.cvtColor(img_original, cv2.COLOR_BGR2HSV)
            hsv_red1 = np.array([  0,140,100])
            hsv_red2 = np.array([ 15,255,255])
            hsv_red3 = np.array([165,140,100])
            hsv_red4 = np.array([180,255,255])
            bw1 = cv2.inRange(hsv, hsv_red1, hsv_red2)
            bw2 = cv2.inRange(hsv, hsv_red3, hsv_red4)
            red = cv2.bitwise_or(bw1, bw2)  # select red

            scale = 1.0/3
            red = cv2.resize(red,
                     (int(self.camera_width*scale), int(self.camera_height*scale)),
                      interpolation = cv2.INTER_LINEAR)
            kps = self.fast.detect(red, None)
            img_original = cv2.drawKeypoints(img_original, kps, None, color=(255,0,0))
            kpcoords = [(kp.pt[0]/scale, kp.pt[1]/scale) for kp in kps]
            bins = [] #list of lists of tuples
            threshold = 35**2#0.04**2

            for k in kpcoords:
                placed = False
                for b in bins:
                    if (k[0]-b[0][0])**2 + (k[1]-b[0][1])**2 < threshold:
                        b.append(k)
                        placed = True
                        break
                if not placed:
                    bins.append([k])
            #self.log(len(bins))

            col = [(255,0,0),(255,255,0),(0,255,0),(0,255,255),(0,0,255),(255,0,255)]
            i = 0
            for b in bins:
                #print("\tcol=",col,"n=",len(b),"\tmiddle=",end='')
                #print(np.mean(b,axis=0))

                #overlay_kp(img_original, b, color = col[i%6])
                i+=1
            #
            # img_rectified = img_original.copy()
            # self.pcm.rectifyImage(img_rectified, img_rectified)
            #
            # if self.verbose:
            #     utils.publish_image(self.bridge, self.pub_rectified, img_rectified)
            #
            # height, width, _ = img_rectified.shape
            # cutoff_absolute = int(floor(height * self.top_cutoff_percent / 100.0))
            # img_cutoff = img_rectified[cutoff_absolute:,:]
            #
            # if self.verbose:
            #     utils.publish_image(self.bridge, self.pub_cutoff, img_cutoff)

            # TODO Improve image contrast

            #img_out = img_cutoff
            utils.publish_image(self.bridge, self.pub_image_out, img_original)


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
