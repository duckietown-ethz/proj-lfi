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
from tf import TransformListener
from geometry_msgs.msg import PoseStamped


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
        self.pub_pose_intersection = self.publisher("~debug/intersection", PoseStamped, queue_size=1)
        self.pub_pose_apriltag = self.publisher("~debug/apriltag", PoseStamped, queue_size=1)
        self.pub_pose_camera = self.publisher("~debug/camera", PoseStamped, queue_size=1)
        self.pub_pose_axle = self.publisher("~debug/axle", PoseStamped, queue_size=1)

        self.bridge = CvBridge()
        self.stop = False

        self.tf = TransformListener()
        self.br = tf.TransformBroadcaster()

        self.log("Initialized")


    def get_pose(self, frame, position, orientation):
        pose = PoseStamped()
        #pose.header.stamp = stamp
        pose.header.frame_id = frame

        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]

        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]

        return pose

    def cbDetection(self, msg):
        self.log("Got detection!")
        if not self.is_shutdown and not self.stop:

            rospack = rospkg.RosPack()
            self.pkg_path = rospack.get_path('apriltag_pose')
            path = self.pkg_path + "/src/four-way-intersection.png"
            output = cv2.imread(path)
            height, width, _ = output.shape
            output = cv2.copyMakeBorder(output, height/2, height/2, width/2, width/2, cv2.BORDER_CONSTANT, None, (0,0,0))
            height_m = 0.61
            width_m = 0.61

            detections = msg.detections

            if len(detections) > 0:
                detection = detections[0]
                # TODO Check tag id
                pose = detection.pose.pose.pose
                position = pose.position
                orientation = pose.orientation

                pose_with_covar = detection.pose
                pose = PoseStamped()
                pose.header = pose_with_covar.header
                pose.header.frame_id = 'camera'
                pose.pose = pose_with_covar.pose.pose

                # Publish transformation from camera to apriltag based on received apriltag pose
                self.br.sendTransform(
                    (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
                    (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w),
                    pose.header.stamp,
                    'apriltag',
                    'camera'
                )

                frame_origin_intersection = self.get_pose('intersection', [0.0,0.0,0.0], [0.0,0.0,0.0,1])
                frame_origin_apriltag = self.get_pose('apriltag', [0.0,0.0,0.0], [0.0,0.0,0.0,1])
                frame_origin_camera = self.get_pose('camera', [0.0,0.0,0.0], [0.0,0.0,0.0,1])
                frame_origin_axle = self.get_pose('axle', [0.0,0.0,0.0], [0.0,0.0,0.0,1])
                axle_arrow_tip = self.get_pose('axle', [0.05,0.0,0.0], [0.0,0.0,0.0,1])

                # Transform all poses into intersection frame
                intersection = self.tf.transformPose('intersection', frame_origin_intersection)
                apriltag = self.tf.transformPose('intersection', frame_origin_apriltag)
                camera = self.tf.transformPose('intersection', frame_origin_camera)
                axle = self.tf.transformPose('intersection', frame_origin_axle)
                arrow_tip = self.tf.transformPose('intersection', axle_arrow_tip)

                # Publish poses for visualization
                self.pub_pose_intersection.publish(intersection)
                self.pub_pose_apriltag.publish(apriltag)
                self.pub_pose_camera.publish(camera)
                self.pub_pose_axle.publish(axle)

                # Convert from intersection frame to pixels in image
                image_size_meter = np.array([height_m, width_m])
                image_size_pixel = np.array([height, width])
                image_offset = np.array([height/2, width/2])

                axle_position_meter = np.array([axle.pose.position.x, axle.pose.position.y])
                axle_position_pixel = image_offset + axle_position_meter / image_size_meter * image_size_pixel
                arrow_tip_position_meter = np.array([arrow_tip.pose.position.x, arrow_tip.pose.position.y])
                arrow_tip_position_pixel = image_offset + arrow_tip_position_meter / image_size_meter * image_size_pixel

                pt1 = (int(axle_position_pixel[0]), int(axle_position_pixel[1]))
                pt2 = (int(arrow_tip_position_pixel[0]), int(arrow_tip_position_pixel[1]))

                output = cv2.arrowedLine(output, (pt1[1], pt1[0]), (pt2[1], pt2[0]), (255, 0, 255), 5)


            try:
                image_msg = self.bridge.cv2_to_compressed_imgmsg(output, dst_format = "png")
            except CvBridgeError as e:
                rospy.logerr(e)

            image_msg.header.stamp = rospy.Time.now()
            self.pub_image.publish(image_msg)


if __name__ == '__main__':
    # Initialize the node
    apriltag_pose_node = AprilTagPoseNode(node_name='apriltag_pose_node') # Keep it spinning to keep the node alive
    rospy.spin()
