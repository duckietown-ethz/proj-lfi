#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy
import rospkg
import yaml
import math
import tf
import cv2

from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
from apriltags2_ros.msg import AprilTagDetectionArray
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_from_matrix


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

                # Transformation from camera coordinate system to root coordinate systems ------------------------------
                (pose_botFrame_pos, pose_botFrame_quat) = self.cam2bot_transform(position, orientation)
                self.log("Duckiebot position in bot frame: ")
                self.log(pose_botFrame_pos)
                self.log("Duckiebot orientation in bot frame: ")
                self.log(tf.transformations.euler_from_quaternion(pose_botFrame_quat))

                D_x = pose_botFrame_pos[0]
                D_y = pose_botFrame_pos[1]
                # ------------------------------------------------------------------------------------------------------

                # TODO The april tag pose is in the camera frame and actually needs to be transformed into the axle coordinate frame
                '''# axle frame
                D_y = -position.x
                D_x = position.z
                self.log('axle: ({}, {}'.format(D_x, D_y))
                '''

                # image frame
                I_x = D_x
                I_y = width_m + D_y
                #self.log('image: ({}, {}'.format(I_x, I_y))

                # pixel scaling
                px = int((I_x / height_m) * height)
                py = int((I_y / width_m) * width)
                pt1 = (px, py)
                self.log('pixel: ({}, {}'.format(px, py))

                # second point for arrow
                rotx = np.array([[1, 0, 0], [0, np.cos(pose_botFrame_quat[0]), -np.sin(pose_botFrame_quat[0])], [0, np.sin(pose_botFrame_quat[0]), np.cos(pose_botFrame_quat[0])]])
                roty = np.array([[np.cos(pose_botFrame_quat[1]), 0, np.sin(pose_botFrame_quat[1])], [0, 1, 0], [-np.sin(pose_botFrame_quat[1]), 0, np.cos(pose_botFrame_quat[1])]])
                rotz = np.array([[np.cos(pose_botFrame_quat[2]), -np.sin(pose_botFrame_quat[2]), 0], [np.sin(pose_botFrame_quat[2]), np.cos(pose_botFrame_quat[2]), 0] ,[0, 0, 1]])
                rot = np.matmul(rotx, roty)
                rot = np.matmul(rot, rotz)
                pt2 = np.matmul(rot, [[80], [0], [0]])
                pt2 = (int(-pt2[0]), -int(pt2[1]))
                pt2 = tuple(map(sum,zip(pt1,pt2)))

                self.log(pt1)
                self.log(pt2)

                if self.in_img(px, py, output):
                    #output[px-5:px+5, py-5:py+5] = [255, 255, 255]
                    #output[pt2[0] - 5:pt2[0] + 5, pt2[1] - 5:pt2[1] + 5] = [255, 255, 255]
                    output = cv2.arrowedLine(output, (pt1[1], pt1[0]), (pt2[1], pt2[0]), (255, 255, 255), 10)

            try:
                image_msg = self.bridge.cv2_to_compressed_imgmsg(output, dst_format = "png")
            except CvBridgeError as e:
                rospy.logerr(e)

            image_msg.header.stamp = rospy.Time.now()
            self.pub_image.publish(image_msg)


    def cam2bot_transform(self, position, orientation):
        '''
        transforms the position (x,y,z) and orientation (qx,qy,qz,qw) of the april tag (t) in the camera frame (c_) to
        the position and orientation of the april tag in the duckiebot frame

        :param position: (x,y,z) position coordinates of the april tag in the camera frame
        :param orientation: (qx,qy,qz,qw) quaternion orientation coordinates of the april tag in the camera frame
        :return:
        '''
        c_x_ct = position.x
        c_y_ct = position.y
        c_z_ct = position.z
        c_ox_ct = orientation.x
        c_oy_ct = orientation.y
        c_oz_ct = orientation.z
        c_ow_ct = orientation.w
        dist = np.linalg.norm([c_x_ct, c_y_ct, c_z_ct])
        if dist > 0.95:
            rospy.logwarn("Tag out of range (dist=%f)" % dist)

        # parameters for transformation from camera frame to duckiebot frame
        # source: https://github.com/duckietown/duckietown-intnav/blob/master/ros-intnav/launch/tf_tree.launch
        orient_camInbot = [-0.5495232, 0.5495232, -0.4449991, 0.4449991]
        rotMat_camInbot = tf.transformations.quaternion_matrix(orient_camInbot)[0:3, 0:3]
        pos_camInbot = np.array([[0.0382047], [0.0], [0.08479]])

        # transformation matrix from duckiebot from to camera frame
        abc1 = np.hstack((rotMat_camInbot, pos_camInbot))
        transformation_bot2cam = np.vstack((abc1, [0, 0, 0, 1]))
        #botframe_pos = np.matmul(transformation_bot2cam, np.vstack(([position.x], [position.y], [position.z], 1)))
        '''
        self.log("shape of transformation matrix")
        self.log(transformation.shape)
        self.log("shape of position vector")
        self.log(np.vstack(([position.x], [position.y], [position.z], 1)).shape)
        self.log("shape of new position vector")
        self.log(botframe_pos.shape)
        '''

        # transformation matrix from camera frame to april tag frame
        orient_aprilInCam = [c_ox_ct, c_oy_ct, c_oz_ct, c_ow_ct]
        rotMat_aprilInCam = tf.transformations.quaternion_matrix(orient_aprilInCam)[0:3, 0:3]
        pos_aprilInCam = np.array([[c_x_ct], [c_y_ct], [c_z_ct]])

        abc2 = np.hstack((rotMat_aprilInCam, pos_aprilInCam))
        transformation_cam2april = np.vstack((abc2, [0, 0, 0, 1]))

        # combined transformation from duckiebot to april tag
        transformation_bot2aprill = np.matmul(transformation_bot2cam, transformation_cam2april)
        pos_aprilInBot = transformation_bot2aprill[0:3,3]
        orient_aprilInBot = transformation_bot2aprill[0:3,0:3]
        quat_orient_aprilInBot = tf.transformations.quaternion_from_matrix(transformation_bot2aprill)


        return pos_aprilInBot, quat_orient_aprilInBot




    def in_img(self, x, y, image):
        height, width, _ = image.shape
        return 0 <= x and x < height and 0 <= y and y < width


if __name__ == '__main__':
    # Initialize the node
    apriltag_pose_node = AprilTagPoseNode(node_name='apriltag_pose_node') # Keep it spinning to keep the node alive
    rospy.spin()
