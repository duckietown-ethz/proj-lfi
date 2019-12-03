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
from numpy import linalg as LA


class TrajectoryControl(DTROS):


    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(TrajectoryControl, self).__init__(node_name=node_name)

        x_track = self.rospy.get_param('/xCoords')
        y_track = self.rospy.get_param('/yCoords')
        x_rate = self.rospy.get_param('/xRate')
        y_rate = self.rospy.get_param('/yRate')
        tangent_angle = self.rospy.get_param('/tangentAngle')

        # self.pub_image_ar = rospy.Publisher("~/%s/augmented_reality_node/%s/image/compressed" % (name, imageName), CompressedImage, queue_size=10)
        self.pub_image_ar = rospy.Publisher("~/.../lane_pose" % (name, imageName), LanePose, queue_size=1)

        # print("wtf!!!")

        # self.sub_cam_img = rospy.Subscriber("~/%s/camera_node/image/compressed" % name, CompressedImage, self.callback, queue_size=1)
        self.sub_cam_img = rospy.Subscriber("~/.../pose_stamped" % name, PoseStamped, self.callback, queue_size=1)
        self.log("Initialized")


    def callback(self, msg):
        d, phi = self.run_pid(msg)

        # image_ar_msg = Augmented.process_image(self, msg)
        self.pub_image_ar.publish(msg_pose_stamped)


    def run_pid(self, msg):
        '''
        PID controller which follows the optimal trajectory generated offline and stored in yaml files
        :param msg: PoseStamed() message: current duckiebot position and orientation in stopline frame ?
        :return: motor commands
        '''
        car_state = msg.pose.pose.pose
        # TODO: find velocity of duckiebot
        look_ahead_x = car_state.position.x + car_state.velocity.x * look_ahead
        look_ahead_y = car_state.position.y + car_state.velocity.y * look_ahead

        idx_min_dist, min_dist = self.closest_track_point(look_ahead_x, look_ahead_y)

        track_pt1 = np.array([x_track[idx_min_dist], y_track[idx_min_dist]])
        track_pt2 = np.array([x_track[idx_min_dist + 1], y_track[idx_min_dist + 1]])
        car_pt = np.array([look_ahead_x, look_ahead_y])
        side = self.track_side(track_pt1, track_pt2, car_pt)

        position_error = side * min_dist
        integral_error += position_error

        # TODO: finish PID on line 185



    def closest_track_point(self, x, y):
        '''
        Function finds the closest distance from the current robot position to the optimal trajectory and the index of
        the x_ or y_coordinates which corresponds to the closest distance
        :param x: current duckiebot position x in stopline coordinate frame
        :param y: current duckiebot position y in stopline coordinate frame
        :return: minimal distance and its index
        '''
        xdist = np.arrange(x_track, (-1, 1)) - x
        ydist = np.arrange(y_track, (-1, 1)) - y
        dist = np.hstack((xdist, ydist))
        distances = LA.norm(dist, axis=1)
        idx_min_dist = np.argmin(distances)
        min_dist = np.min(distances)
        # TODO: more efficient search, e.g. first rough search, then binary search

        return idx_min_dist, min_dist


    def track_side(self, pt1, pt2, pt3):
        '''
        Function determines if the current car position is on the left or right side of the optimal trajectory
        :param pt1: first point of optimal trajectory (x,y)
        :param pt2: second point of optimal trajectory (x,y)
        :param pt3: current car position (x,y)
        :return a: -1 if car is on the left of the track, +1 if the car is on the right side of the track
        '''
        if (pt2[0] - pt1[0])*(pt3[1] - pt1[1]) - (pt2[1] - pt1[1])*(pt3[0] - pt1[0]) > 0:
            a = -1
        else:
            a = 1
        return a





if __name__ == '__main__':
    # Initialize the node
    print('Starting trajectory controller node ... ---------------------------------------------------------------------')
    camera_node = TrajectoryControl(node_name='trajectory')
    print('... finishing trajectory controller node ---------------------------------------------------------------------')
    # Keep it spinning to keep the node alive
    rospy.spin()
