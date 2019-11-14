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
from duckietown_msgs.msg import SegmentList
from cv_bridge import CvBridge, CvBridgeError
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler


class EstimatorNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(EstimatorNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")

        rospy.set_param('~eps', 1.0)
        self.parameters['~eps'] = None

        rospy.set_param('~min_samples', 4)
        self.parameters['~min_samples'] = None

        self.updateParameters()

        # Subscribers
        self.sub_lines = self.subscriber("~linesegments_ground", SegmentList, self.cbSegments, queue_size=1)

        # Publishers
        self.pub_image = self.publisher("~debug/compressed", CompressedImage, queue_size=1)

        self.bridge = CvBridge()
        self.stop = False

        self.log("Initialized")

    def draw_segment(self, image, pt1, pt2, color, thickness):
        defined_colors = {
            0: ['rgb', [1, 1, 1]],
            1: ['rgb', [1, 1, 0]],
            2: ['rgb', [1, 0, 0]],

            3: ['rgb', [0, 1, 1]],
            4: ['rgb', [0, 0, 1]],
            5: ['rgb', [0, 1, 0]],
            -1: ['rgb', [0.1, 0, 0]],
            }
        _color_type, [r, g, b] = defined_colors[color]

        pt1 = (pt1[1], pt1[0])
        pt2 = (pt2[1], pt2[0])
        cv2.line(image, pt1, pt2, (b * 255, g * 255, r * 255), int(thickness*50))
        return image

    def pixel2ground(self, pixel):
        homography
        uv_raw = np.array([pixel.u, pixel.v])
#         if not self.rectified_input:
#             uv_raw = self.pcm.rectifyPoint(uv_raw)
        #uv_raw = [uv_raw, 1]
        uv_raw = np.append(uv_raw, np.array([1]))
        ground_point = np.dot(homography, uv_raw)
        point = Point()
        x = ground_point[0]
        y = ground_point[1]
        z = ground_point[2]
        point.x = x / z
        point.y = y / z
        point.z = 0.0
        return point

    def cbSegments(self, msg):
        self.log("Got segments!")
        if not self.is_shutdown and not self.stop:
            #self.log(msg)

            #try:
                #original = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding = "bgr8")
            #except CvBridgeError as e:
                #rospy.logerr(e)

            height = 500
            width = 500
            height_m = 1
            width_m = 1
            output = np.zeros((height, width, 3), np.uint8)

            segments = msg.segments

            if len(segments) > 0:
                red_points = None
                for seg in segments:
                    color_id = seg.color
                    D_p1 = seg.points[0]
                    D_p2 = seg.points[1]
                    D_p1 = np.matrix([D_p1.x, D_p1.y]).T
                    D_p2 = np.matrix([D_p2.x, D_p2.y]).T

                    if color_id == 2:
                        if red_points == None:
                            red_points = np.vstack([D_p1.T, D_p2.T])
                        else:
                            red_points = np.vstack([red_points, D_p1.T, D_p2.T])

                    # Filter lines that are too long
                    length = np.sqrt(np.sum(np.power(D_p1-D_p2, 2)))
                    #if length >= 5.0:
                        #continue


                    I_p1 = np.matrix([1, 0.5]).T - D_p1
                    I_p2 = np.matrix([1, 0.5]).T - D_p2

                    # pixel scaling
                    P_p1 = (I_p1 / height_m * height).astype(int)
                    P_p2 = (I_p2 / height_m * height).astype(int)

                    self.draw_segment(output, P_p1, P_p2, color_id, length)

                if red_points != None and red_points.shape[0] > 0:
                    red_points_normalized = StandardScaler().fit_transform(red_points)
                    db = DBSCAN(eps=self.parameters['~eps'], min_samples=self.parameters['~min_samples']).fit(red_points_normalized)
                    labels = db.labels_
                    unique_labels = set(labels)
                    self.log("{} points, {} labels".format(red_points.shape[0], len(set(labels))))

                    for label in unique_labels:
                        class_member_mask = (labels == label)
                        class_points = red_points[class_member_mask]
                        class_mean = np.mean(class_points, axis=0)
                        self.log(class_mean)
                        I_class_mean = np.matrix([1, 0.5]).T - class_mean.T
                        P_class_mean = (I_class_mean / height_m * height).astype(int)
                        self.draw_segment(output, P_class_mean, P_class_mean, 5, 0.1)

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
    estimator_node = EstimatorNode(node_name='estimator_node') # Keep it spinning to keep the node alive
    rospy.spin()
