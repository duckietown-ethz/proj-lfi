#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy
import rospkg
import yaml
import math

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
            #self.log(msg)

            #try:
                #original = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding = "bgr8")
            #except CvBridgeError as e:
                #rospy.logerr(e)


            rospack = rospkg.RosPack()
            self.pkg_path = rospack.get_path('apriltag_pose')
            path = self.pkg_path + "/src/four-way-intersection.png"
            output = cv2.imread(path)
            height, width, _ = output.shape
            height_m = 0.61
            width_m = 0.61

            detections = msg.detections

            if len(detections) > 0:
                detection = detections[0]
                # TODO Check tag id
                pose = detection.pose.pose.pose
                position = pose.position
                orientation = pose.orientation

                # TODO The april tag pose is in the camera frame and actually needs to be transformed into the axle coordinate frame
                # axle frame
                D_y = -position.x
                D_x = position.z
                self.log('axle: ({}, {}'.format(D_x, D_y))

                # image frame
                I_x = D_x
                I_y = width_m + D_y
                self.log('image: ({}, {}'.format(I_x, I_y))

                # pixel scaling
                px = int((I_x / height_m) * height)
                py = int((I_y / width_m) * width)
                self.log('pixel: ({}, {}'.format(px, py))

                if self.in_img(px, py, output):
                    output[px, py] = [255, 255, 255]

            try:
                image_msg = self.bridge.cv2_to_compressed_imgmsg(output, dst_format = "png")
            except CvBridgeError as e:
                rospy.logerr(e)

            image_msg.header.stamp = rospy.Time.now()
            self.pub_image.publish(image_msg)

    def in_img(self, x, y, image):
        height, width, _ = image.shape
        return 0 <= x and x < height and 0 <= y and y < width


if __name__ == '__main__':
    # Initialize the node
    apriltag_pose_node = AprilTagPoseNode(node_name='apriltag_pose_node') # Keep it spinning to keep the node alive
    rospy.spin()
