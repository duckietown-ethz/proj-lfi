#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy
import yaml

from duckietown import DTROS
from duckietown_msgs.msg import WheelsCmdStamped, LanePose, BoolStamped
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from duckietown_utils import load_map, load_camera_intrinsics, load_homography, rectify
from collections import OrderedDict
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import utils


class VirtualLaneNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(VirtualLaneNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")

        # Initialize parameters
        self.trajectories = self.load_trajectories(['straight', 'left', 'right'])

        self.parameters['~verbose'] = None
        self.parameters['~trajectory'] = None
        self.parameters['~end_condition_distance'] = None
        self.parameters['~end_condition_angle_deg'] = None
        self.updateParameters()
        self.refresh_parameters()

        # Publishers
        self.pub_lanepose = rospy.Publisher("~lane_pose", LanePose, queue_size=1)
        self.pub_trajectory = rospy.Publisher("~verbose/trajectory", Marker, queue_size=1)
        self.pub_closest = rospy.Publisher("~verbose/closest_point", Marker, queue_size=1)
        self.pub_switch2lanefollow = rospy.Publisher("~intersection_done", BoolStamped, queue_size=1)

        # Subscribers
        self.sub_intpose = rospy.Subscriber("~intersection_pose", PoseStamped, self.cb_intpose, queue_size=1)

        self.i = 0

        self.log("Initialized")


    def load_trajectories(self, trajectory_names):
        result = {}

        for name in trajectory_names:
            track_x = rospy.get_param('~xCoords_{}'.format(name))
            track_y = rospy.get_param('~yCoords_{}'.format(name))
            tangent_angle = rospy.get_param('~tangentAngle_{}'.format(name))
            curvature = rospy.get_param('~curvature_{}'.format(name))

            track = np.vstack([np.array(track_x), np.array(track_y)]).T
            tangent_angle = np.array(tangent_angle)
            curvature = np.array(curvature)

            trajectory = {
                'track': track,
                'tangent_angle': tangent_angle,
                'curvature': curvature,
            }

            result[name] = trajectory
            self.log("Loaded trajectory '{}'".format(name))

        return result


    def cb_intpose(self, int_pose):
        '''
        cb_intpose Function, that subscribes to the incoming PoseStamped (position and orientation of duckiebot) message and
        publishes the LanePose message with the entries d and phi, that can be used for the controller
        :param int_pose: PoseStamped ros message
        :return: None
        '''

        # TODO Change to debug level
        if self.verbose:
            self.log('Received intersection pose.')

        if self.parametersChanged:
            self.log('Parameters changed.', 'info')
            self.refresh_parameters()
            self.parametersChanged = False

        trajectory = self.trajectories[self.trajectory]
        track = trajectory['track']
        tangent_angle = trajectory['tangent_angle']
        curvature = trajectory['curvature']

        if self.verbose:
            if self.i % 10 == 0:
                self.publish_track(track, tangent_angle)
            self.i += 1

        # compte motor commands for lefta nd right motor with a simple line following pid controller
        # --> this function is INCOMPLETE!
        # u_l, u_r = self.run_pid(int_pose)

        # compute distance d and angle phi, same as in lane following
        d, phi, curv = self.relative_pose(int_pose, track, tangent_angle, curvature)
        if self.verbose:
            self.log("distance: " + str(d))
            self.log("angle: " + str(phi))

        # convert to LanePose() message
        radius = track[-1][1]
        lane_pose = LanePose()
        lane_pose.header.stamp = int_pose.header.stamp
        lane_pose.d = d
        lane_pose.phi = phi
        lane_pose.curvature = curv

        # publish lane pose msg
        self.pub_lanepose.publish(lane_pose)


    def relative_pose(self, pose, track, tangent_angle, curvature):
        '''
        This function uses the current position of the duckiebot in the intersection-stop-line frame to find the closest
        distance to the optimal trajectory and the angle difference between the closest point of the optimal trajectory
        and current orientation of the duckiebot.
        :param pose: current position of the duckiebot in the intersection-stop-line frame
        :return: distance d and angle phi compared to the optimal trajectory --> same as for line following
        '''

        position, orientation = utils.pose_to_tuple(pose.pose)
        _, _, yaw = euler_from_quaternion(orientation)
        car_pos = np.array(position[0:2])

        idx_min_dist, min_dist = self.closest_track_point(track[:-1], car_pos)

        '''dist2end = LA.norm(track[-1, :] - car_pos)
        ang2end = abs(tangent_angle[-1]-yaw)*180/np.pi
        self.log("distance to endpoint: {}".format(dist2end))
        self.log("position of endpoint: {}".format(track[-1,:]))
        self.log("yaw angle of car: {}".format(yaw*180/np.pi))
        self.log("angle of endpoint: {}".format(tangent_angle[-1]*180/np.pi))
        self.log("angle to endpoint: {}".format(ang2end))'''
        # if the distance from the car position to the end of the optimal trajectory is less than 28 cm
        # and the difference between the angle of the car and end of the optimal trajectory is less than 15 degrees
        # switch back to lane following
        distance_to_end = LA.norm(track[-1, :] - car_pos)
        angle_to_end = abs(tangent_angle[-1]-yaw)

        if self.verbose:
            self.log("distance to end: {}".format(distance_to_end))
            self.log("angle to end: {}".format(angle_to_end))

        end_condition_angle_rad = np.deg2rad(self.end_condition_angle_deg)
        distance_good_enough = distance_to_end < self.end_condition_distance
        angle_good_enough = angle_to_end < end_condition_angle_rad

        switch = BoolStamped()
        switch.header.stamp = rospy.Time(0)
        switch.data = distance_good_enough and angle_good_enough

        self.pub_switch2lanefollow.publish(switch)

        if switch.data:
            self.log("SWITCH BACK TO LANE FOLLOWING!!")
            
        # publish lane pose msg
        closest_pos = track[idx_min_dist]
        closest_angle = tangent_angle[idx_min_dist]
        next_pos = track[idx_min_dist + 1]

        if self.verbose:
            self.publish_closest(closest_pos, car_pos)

        side = self.track_side(closest_pos, next_pos, car_pos)
        d = -min_dist * side
        phi = yaw - closest_angle
        curv = curvature[idx_min_dist]

        return d, phi, curv


    def closest_track_point(self, track, car_pos):
        '''
        Function finds the closest distance from the current robot position to the optimal trajectory and the index of
        the x_ or y_coordinates which corresponds to the closest distance
        :param x: current duckiebot position x in stopline coordinate frame
        :param y: current duckiebot position y in stopline coordinate frame
        :return: minimal distance and its index
        '''
        diffs = track - car_pos
        distances = LA.norm(diffs, axis=1)
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
        if (pt2[0]-pt1[0]) * (pt3[1]-pt1[1]) - (pt2[1]-pt1[1]) * (pt3[0]-pt1[0]) > 0:
            a = -1.0
        else:
            a = 1.0
        return a


    def publish_closest(self, pos1, pos2):
        namespace = 'trajectory'
        now = rospy.Time.now()

        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = 'intersection'
        marker.ns = namespace
        marker.id = 1
        marker.action = 0
        marker.lifetime = rospy.Time(0)
        marker.scale.x = 0.005
        marker.scale.y = 0.005
        marker.scale.z = 0.005

        marker.type = 5
        marker.pose = utils.origin_pose('intersection').pose
        marker.points = [self.pos_to_point(pos1), self.pos_to_point(pos2)]
        marker.color = self.gen_color(0,1,0)
        marker.colors = []

        self.pub_closest.publish(marker)

        return marker


    def publish_track(self, track, tangent_angle):
        namespace = 'trajectory'
        now = rospy.Time.now()

        def gen_marker():
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = 'intersection'
            marker.ns = namespace
            marker.id = 2
            marker.action = 0
            marker.lifetime = rospy.Time(0)
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01

            return marker

        track_marker = gen_marker()
        track_marker.type = 7
        track_marker.pose = utils.origin_pose('intersection').pose
        track_marker.points = list([self.pos_to_point(pos) for pos in track])
        track_marker.colors = list([self.gen_color(0,0,1) for _ in track])

        self.pub_trajectory.publish(track_marker)


    def gen_color(self, r, g, b):
        c = ColorRGBA()
        c.r = float(r)
        c.g = float(g)
        c.b = float(b)
        c.a = 1.0
        return c


    def pos_to_point(self, pos):
        return Point(pos[0], pos[1], 0.0)


    def refresh_parameters(self):
        self.verbose = self.parameters['~verbose']
        self.trajectory = self.parameters['~trajectory']
        self.end_condition_distance = self.parameters['~end_condition_distance']
        self.end_condition_angle_deg = self.parameters['~end_condition_angle_deg']




if __name__ == '__main__':
    # Initialize the node
    camera_node = VirtualLaneNode(node_name='trajectory')
    # Keep it spinning to keep the node alive
    rospy.spin()
