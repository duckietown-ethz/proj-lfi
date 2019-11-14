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
        self.sub_camera = self.subscriber("~rectified_image", CompressedImage, self.cbImage, queue_size=1)

        # Publishers
        self.pub_image1 = self.publisher("~debug1/compressed", CompressedImage, queue_size=1)
        self.pub_image2 = self.publisher("~debug2/compressed", CompressedImage, queue_size=1)

        self.bridge = CvBridge()
        self.stop = False

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

        self.last_avg_angle = None
        self.cumulative_angle = 0.0
        self.last_avg_position = None
        self.cumulative_position = np.matrix([0.0, 0.0]).T

        self.log("Initialized")

    def draw_points(self, image, points, color):
        if points != None and len(points) > 0:
            for p in points:
                #point = point[0]
                #p = (corner[1], corner[0])
                self.draw_segment(image, p, p, 5, 0.1)

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
        return
        self.log("Got segments!")
        if not self.is_shutdown and not self.stop:

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
            self.pub_image1.publish(image_msg)

    def cbImage(self, msg):
            self.log("Got image!")

            try:
                original = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding = "bgr8")
            except CvBridgeError as e:
                rospy.logerr(e)

            upscaled = cv2.resize(original, (640, 480), None, 0, 0, cv2.INTER_LINEAR)
            # TODO Make camera_img_to_birdseye work with images of any size
            warped = self.camera_img_to_birdseye(upscaled, self.homography, 500.0)

            img = warped

            # TODO Play around with these parameters
            blurred = cv2.GaussianBlur(img, (0,0), 3)
            img = cv2.addWeighted(img, 1.5, blurred, -0.5, 0)


            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            max_corners = 10
            quality_level = 0.5
            min_distance = 0.0
            corners = cv2.goodFeaturesToTrack(gray, max_corners, quality_level, min_distance)

            # Convert to nx2-array of pixel coordinates (x,y)
            points_goodFeaturesToTrack = []
            if len(corners) > 0:
                try:
                    points = corners.squeeze()
                    points[:, 0], points[:, 1] = points[:, 1], points[:, 0].copy()
                    points_goodFeaturesToTrack = points
                except:
                    self.log("###################")
                    self.log(points.shape)

            orb = cv2.ORB_create(400)
            kp, des = orb.detectAndCompute(img, None)



            angles = []
            positions = []
            for keypoint in kp:
                angles.append(keypoint.angle)
                positions.append([keypoint.pt[0], keypoint.pt[1]])

            angles = np.array(angles)
            positions = np.matrix(positions).T

            avg_angle = np.mean(angles)
            if self.last_avg_angle != None:
                avg_angle_diff = avg_angle - self.last_avg_angle
                self.cumulative_angle += avg_angle_diff
                #self.log(self.cumulative_angle)

            self.last_avg_angle = avg_angle


            avg_position = np.mean(positions, axis=1)
            if self.last_avg_position != None:
                avg_position_diff = avg_position - self.last_avg_position
                self.cumulative_position += avg_position_diff
                self.log(self.cumulative_position)

            self.last_avg_position = avg_position




            output = img.copy()
            self.draw_points(output, points_goodFeaturesToTrack, 5)
            #output = cv2.drawKeypoints(output, kp, None, (255,255,255), 4)


            (h, w) = output.shape[:2]
            center = (w / 2, h / 2)
            #rotMat = cv2.getRotationMatrix2D(center, self.cumulative_angle, 1.0)
            rotMat = cv2.getRotationMatrix2D(center, 0, 1.0)
            transMat = np.hstack([np.zeros((2,2)), self.cumulative_position])
            #transMat = np.hstack([np.zeros((2,2)), np.ones((2,1))])
            M = rotMat + transMat
            output = cv2.warpAffine(output, M, (w, h))

            try:
                image_msg = self.bridge.cv2_to_compressed_imgmsg(output, dst_format = "jpg")
            except CvBridgeError as e:
                rospy.logerr(e)

            image_msg.header.stamp = rospy.Time.now()
            self.pub_image2.publish(image_msg)

    def pixel2ground(self, px):
        px_ext = np.vstack([px, 1])
        ground_point = self.homography * px_ext
        ground_x = ground_point[0] / float(ground_point[2])
        ground_y = ground_point[1] / float(ground_point[2])
        return np.vstack([ground_x, ground_y])

#     def ground2pixel(self, point):
#         if point.z != 0:
#             msg = 'This method assumes that the point is a ground point (z=0). '
#             msg += 'However, the point is (%s,%s,%s)' % (point.x, point.y, point.z)
#             raise ValueError(msg)
#
#         ground_point = np.array([point[0], point[1], 1.0])
#         image_point = np.linalg.solve(self.homography, ground_point)
#         image_x = image_point[0] / image_point[2]
#         image_y = image_point[1] / image_point[2]
#         return np.matrix([image_x, image_y]).T

    def in_img(self, x, y, image):
        height, width, _ = image.shape
        return 0 <= x and x < height and 0 <= y and y < width

    def camera_img_to_birdseye(self, cam_img, homography, px_per_m=500.0):
        height, width, _ = cam_img.shape

        H = homography

        # Scale axle coordinates to pixels
        offset = np.matrix([0.0,0.0]).T
        scaler_mat = np.vstack([np.ones((2,3))*px_per_m, np.ones((1,3))])
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


if __name__ == '__main__':
    # Initialize the node
    estimator_node = EstimatorNode(node_name='estimator_node') # Keep it spinning to keep the node alive
    rospy.spin()
