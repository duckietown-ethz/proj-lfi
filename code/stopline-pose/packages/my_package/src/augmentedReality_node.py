#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy
import yaml

from duckietown import DTROS
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import WheelsCmdStamped
from cv_bridge import CvBridge
from duckietown_utils import load_map, load_camera_intrinsics, load_homography, rectify
from collections import OrderedDict

from augmented_reality import Augmented


class AugmentedRealityNode(DTROS, Augmented):


    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(AugmentedRealityNode, self).__init__(node_name=node_name)
        Augmented(node_name = node_name)
        #self.veh_name = rospy.get_namespace().strip("/")
        self.veh_name = os.environ['VEHICLE_NAME']
        self.map_name = os.environ['MAP_NAME']

        self.intrinsics = load_camera_intrinsics('wlan')
        self.H = load_homography('wlan')
        # Wait for the automatic gain control
        # of the camera to settle, before we stop it
        rospy.sleep(2.0)
        name = self.veh_name
        print (name)
        imageName = self.map_name.split('.')[0]
        rospy.set_param('/%s/camera_node/exposure_mode' % name, 'off')

        # I added this change resolution
        rospy.set_param('/%s/camera_node/res_w' % name, 640)
        rospy.set_param('/%s/camera_node/res_h' % name, 480)
        # Up to here

        self.pub_image_ar = rospy.Publisher("~/%s/augmented_reality_node/%s/image/compressed" % (name, imageName), CompressedImage, queue_size=10)

        # print("wtf!!!")

        self.sub_cam_img = rospy.Subscriber("~/%s/camera_node/image/compressed" % name, CompressedImage, self.callback, queue_size=10)
        self.log("Initialized")


    def callback(self, msg):

        image = Augmented.process_image(self, msg)
        image_message = CvBridge().cv2_to_compressed_imgmsg(image)
        self.pub_image_ar.publish(image_message)


if __name__ == '__main__':
    # Initialize the node
    print('Starting CRA1: Augmented Reality ... ---------------------------------------------------------------------')
    camera_node = AugmentedRealityNode(node_name='augmented')
    print('... finishing Augmented Reality node ---------------------------------------------------------------------')
    # Keep it spinning to keep the node alive
    rospy.spin()
