#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy
import yaml

from duckietown import DTROS
from duckietown_msgs.msg import WheelsCmdStamped, LanePose
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from duckietown_utils import load_map, load_camera_intrinsics, load_homography, rectify
from collections import OrderedDict
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class TrajectoryControl(DTROS):


    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(TrajectoryControl, self).__init__(node_name=node_name)

        self.x_track = rospy.get_param('/trajectory_following_node/xCoords')
        self.y_track = rospy.get_param('/trajectory_following_node/yCoords')
        self.x_rate = rospy.get_param('/trajectory_following_node/xRate')
        self.y_rate = rospy.get_param('/trajectory_following_node/yRate')
        self.tangent_angle = rospy.get_param('/trajectory_following_node/tangentAngle')

        self.pub_image_ar = rospy.Publisher("~/.../lane_pose" % (name, imageName), LanePose, queue_size=1)

        self.sub_cam_img = rospy.Subscriber("~/.../pose_stamped" % name, PoseStamped, self.callback, queue_size=1)
        self.log("Initialized")


    def callback(self, msg):
        '''
        Callback Function, that subscribes to the incoming LanePose (position and orientation of duckiebot) message and
        publishes the PoseStamped message with the entries d and phi, that can be used for the controller
        :param msg: LanePose ros message
        :return: None
        '''
        # compte motor commands for lefta nd right motor with a simple line following pid controller
        # --> this function is INCOMPLETE!
        # u_l, u_r = self.run_pid(msg)

        # compute distance d and angle phi, same as in line following
        d, phi = self.relative_pose(msg)

        # convert to LanePose() message
        d, phi = self.relative_pose(message_pose)
        radius = self.y_track[-1]
        print("distance: " + str(d))
        print("angle: " + str(phi))
        # convert to LanePose() message
        msg_pub = LanePose()
        msg_pub.header.stamp = msg.header.stamp
        msg_pub.d = d
        msg_pub.phi = phi
        msg_pub.curvature = 1 / radius


        # publish lane pose msg
        self.pub_image_ar.publish(msg_pub)


    def relative_pose(self, msg):
        '''
        This function uses the current position of the duckiebot in the intersection-stop-line frame to find the closest
        distance to the optimal trajectory and the angle difference between the closest point of the optimal trajectory
        and current orientation of the duckiebot.
        :param msg: current position of the duckiebot in the intersection-stop-line frame
        :return: distance d and angle phi compared to the optimal trajectory --> same as for line following
        '''

        db_position = msg.pose.position
        db_orientation = msg.pose.orientation
        orientation_list = [db_orientation.x, db_orientation.y, db_orientation.z, db_orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        idx_min_dist, min_dist = self.closest_track_point(db_position.x, db_position.y)

        track_pt1 = np.array([self.x_track[idx_min_dist], self.y_track[idx_min_dist]])
        track_pt2 = np.array([self.x_track[idx_min_dist + 1], self.y_track[idx_min_dist + 1]])
        car_pt = np.array([db_position.x, db_position.y])
        side = self.track_side(track_pt1, track_pt2, car_pt)

        closest_pt = np.array([self.x_track[idx_min_dist], self.y_track[idx_min_dist]])
        closest_angle = self.tangent_angle[idx_min_dist]

        print("closest pt = " + str(closest_pt) + "; ang phi = " + str(closest_angle))

        d = min_dist * side
        phi = abs(closest_angle - yaw) * side

        return d, phi


    def closest_track_point(self, x, y):
        '''
        Function finds the closest distance from the current robot position to the optimal trajectory and the index of
        the x_ or y_coordinates which corresponds to the closest distance
        :param x: current duckiebot position x in stopline coordinate frame
        :param y: current duckiebot position y in stopline coordinate frame
        :return: minimal distance and its index
        '''
        xdist = np.reshape(self.x_track, (-1, 1)) - x
        ydist = np.reshape(self.y_track, (-1, 1)) - y
        # print("xdist = " + str(xdist))
        # print("ydist = " + str(ydist))
        dist = np.hstack((xdist, ydist))
        distances = LA.norm(dist, axis=1)
        # print("LA.norm = " + str(distances))
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


    def run_pid(self, msg):
        '''
        PID controller which follows the optimal trajectory generated offline and stored in yaml files
        --> This function is incomplete: some parameters need to be defined, the velocity needs to be computed and a
        mapping from steer to torque left and right
        :param msg: PoseStamed() message: current duckiebot position and orientation in stopline frame ?
        :return: motor commands
        '''
        car_state = msg.pose.pose.pose
        # TODO: find velocity of duckiebot
        look_ahead_x = car_state.position.x + car_state.velocity.x * look_ahead
        look_ahead_y = car_state.position.y + car_state.velocity.y * look_ahead

        idx_min_dist, min_dist = self.closest_track_point(look_ahead_x, look_ahead_y)

        track_pt1 = np.array([self.x_track[idx_min_dist], self.y_track[idx_min_dist]])
        track_pt2 = np.array([self.x_track[idx_min_dist + 1], self.y_track[idx_min_dist + 1]])
        car_pt = np.array([look_ahead_x, look_ahead_y])
        side = self.track_side(track_pt1, track_pt2, car_pt)

        position_error = side * min_dist
        integral_error += position_error

        u_steer = kp*position_error + kd*(position_error-prev_position_error) + ki*integral_error
        prev_position_error = position_error

        u_torque = target_velocity

        # TODO: find mapping from (torque, steer) to (torque left, torque right)
        torque_left = fcn(u_torque, u_steer)
        torque_right = fcn(u_torque, u_steer)

        return torque_left, torque_right




if __name__ == '__main__':
    # Initialize the node
    print('Starting trajectory controller node ... ---------------------------------------------------------------------')
    camera_node = TrajectoryControl(node_name='trajectory')
    print('... finishing trajectory controller node ---------------------------------------------------------------------')
    # Keep it spinning to keep the node alive
    rospy.spin()
