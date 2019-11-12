#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy
import yaml

from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
from apriltags2_ros.msg import AprilTagDetectionArray
from cv_bridge import CvBridge, CvBridgeError


class AprilTagPoseNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(AprilTagPoseNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")

        rospy.set_param('~param1', 0)
        self.parameters['~param1'] = None

        self.updateParameters()

        # Subscribers
        self.sub_image = self.subscriber("~apriltag_detections", AprilTagDetectionArray, self.cbDetection, queue_size=1)

        # Publishers
        self.pub_image = self.publisher("~debug/compressed", CompressedImage, queue_size=1)

        self.bridge = CvBridge()
        self.stop = False

        self.log("Initialized")


    def cbDetection(self, msg):
        self.log("Got detection!")
        if not self.is_shutdown and not self.stop:
            self.log(msg)

            #try:
                #original = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding = "bgr8")
            #except CvBridgeError as e:
                #rospy.logerr(e)



            #try:
                #image_msg = self.bridge.cv2_to_compressed_imgmsg(output, dst_format = "png")
            #except CvBridgeError as e:
                #rospy.logerr(e)

            #image_msg.header.stamp = rospy.Time.now()
            #self.pub_image.publish(image_msg)


if __name__ == '__main__':
    # Initialize the node
    apriltag_pose_node = AprilTagPoseNode(node_name='apriltag_pose_node') # Keep it spinning to keep the node alive
    rospy.spin()
