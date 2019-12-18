#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import rospkg

from duckietown import DTROS
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Point, Quaternion, PoseStamped, PoseArray
from std_msgs.msg import Bool, Float64
from duckietown_msgs.msg import Pose2DStamped, FSMState, BoolStamped
from cv_bridge import CvBridge, CvBridgeError


import utils
from config_loader import get_camera_info_for_robot, get_homography_for_robot
from intersection_model import Intersection4wayModel
from scaled_homography import ScaledHomography
from stopline_detector import StoplineDetector
from stopline_filter import StoplineFilter
from image_geometry import PinholeCameraModel
from tf.transformations import unit_vector, quaternion_from_euler, euler_from_quaternion
from timekeeper import TimeKeeper


class LocalizationNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LocalizationNode, self).__init__(node_name=node_name)
        self.veh_name = rospy.get_namespace().strip("/")

        # Initialize parameters
        self.parameters['~verbose'] = False
        self.parameters['~demo'] = True
        self.parameters['~stop_time'] = 2
        self.parameters['~show_time_keeping'] = False

        self.parameters['~integration_assisted_clustering'] = True
        self.parameters['~integration_enabled'] = True

        self.parameters['~omega_factor'] = 1
        self.parameters['~damping'] = False

        self.parameters['~omega_max'] = 4.2
        self.parameters['~v_bar'] = 0.15

        # XXX:
        self.parameters['/{}/birdseye_node/image_size'.format(self.veh_name)] = None
        self.parameters['~start_x'] = None
        self.parameters['~start_y'] = None
        self.parameters['~start_z'] = None
        self.parameters['~dbscan_eps'] = None
        self.parameters['~dbscan_min_samples'] = None

        self.parameters['~min_quality'] = 0.5

        self.updateParameters()
        self.refresh_parameters()

        self.active = True
        self.resetting = False

        self.bridge = CvBridge()

        self.pose = None # wrt intersection frame
        self.integrated_pose = None # wrt intersection frame
        self.pose_in = None # wrt integrator frame
        self.integrator_offset = None # pose of integrator frame wrt intersection frame

        self.model = Intersection4wayModel()

        homography = get_homography_for_robot(self.veh_name)
        camera_info = get_camera_info_for_robot(self.veh_name)
        self.scaled_homography = ScaledHomography(homography, camera_info.height, camera_info.width)
        self.stopline_detector = StoplineDetector(self.scaled_homography, point_reduction_factor=1)

        # Subscribers
        self.sub_reset = self.subscriber('~reset', Bool, self.cb_reset, queue_size=1)
        self.sub_pose_in = self.subscriber('~open_loop_pose_estimate', Pose2DStamped, self.cb_pose_in, queue_size=1)
        self.sub_switch = self.subscriber("~switch", BoolStamped, self.cbSwitch, queue_size=1)
        self.sub_fsm = self.subscriber('~mode', FSMState, self.cb_mode, queue_size=1)
        self.sub_camera_info = self.subscriber('~camera_info', CameraInfo, self.cb_camera_info, queue_size=1)
        self.sub_image_in = None # will be initialized once CameraInfo is received

        # Publishers
        self.pub_clustering = self.publisher('~verbose/clustering/compressed', CompressedImage, queue_size=1)
        self.pub_red_filter = self.publisher('~verbose/red_filter/compressed', CompressedImage, queue_size=1)
        self.pub_stoplines_measured = self.publisher('~stoplines_measured', PoseArray, queue_size=1)
        self.pub_stoplines_predicted = self.publisher('~stoplines_predicted', PoseArray, queue_size=1)
        self.pub_pose_estimates = self.publisher('~pose_estimates', PoseArray, queue_size=1)
        self.pub_best_pose_estimate = self.publisher('~best_pose_estimate', PoseStamped, queue_size=1)
        self.pub_estimate_quality = self.publisher('~best_estimate_quality', Float64, queue_size=1)
        # TODO: remap
        self.intersection_go = self.publisher('/{}/coordinator_node/intersection_go'.format(self.veh_name), BoolStamped, queue_size=1)

        self.reset()
        self.log('Waiting for camera to update its parameters.')

    def cb_camera_info(self, msg):
        # must set image subscriber first time around,
        # next time check height to see if resolution has changed
        if self.sub_image_in is None or self.scaled_homography.current_height != msg.height:
            self.scaled_homography.update_homography(msg.height, msg.width)
            buffer_size = msg.width * msg.height * 3 * 2 # 2 is a safety factor
            self.log('Buffer changed to {}.'.format(buffer_size), 'info')
            # Now the node can proceed to process images
            self.sub_image_in = self.subscriber('~image_in/compressed', CompressedImage, self.cb_image_in, queue_size=1, buff_size=buffer_size)


    def cb_mode(self,fsm_state_msg):
        if fsm_state_msg.state == "INTERSECTION_COORDINATION":
            self.log('Setting up for intersection')
            rospy.sleep(self.stop_time)
            self.reset()
            # For this task other duckiebots on the same road are excluded so it
            #  just waits 2 seconds then this node gives the GO signal
            if self.parameters['~demo']:
                go_msg = BoolStamped()
                go_msg.data = True
                self.intersection_go.publish(go_msg)


    def cbSwitch(self, switch_msg):
        self.log('Switch ' + str(switch_msg.data))
        self.active = switch_msg.data


    # the received pose is wrt the initial position when car_interface started
    def cb_pose_in(self, pose2d_in):
        if self.integrator_offset is not None:
            # correct for drift
            self.integrated_pose = self.correct(pose2d_in)
        self.pose_in = pose2d_in # keep it to update offset


    def pub_pose(self, pose):
        self.pub_best_pose_estimate.publish(pose)


    # metrics for the clustering algoruthm
    def pub_quality(self, qual):
        msg = Float64()
        msg.data = qual
        self.pub_estimate_quality.publish(msg)


    # useful for debugging
    def cb_reset(self, msg):
        do_reset = msg.data
        self.log('got reset command: '+str(do_reset))
        if do_reset:
            self.reset()


    # reset the pose to the ideal pose before entering an intersection
    def reset(self):
        self.log('Intersection localization is resetting')
        # XXX: replace the following mechanism with mutex or workaround
        self.resetting = True

        rospy.sleep(.2) # wait for current image to finish processing
        position = [self.start_x, self.start_y, self.start_z]
        heading = np.pi/2.0
        orientation = unit_vector(quaternion_from_euler(0, 0, heading, axes='sxyz'))
        self.pose = utils.pose_stamped('intersection', position, orientation)
        self.update_integrator_offset()
        self.pub_pose(self.pose)
        self.resetting = False



    def correct(self, pose_in):
        # assume integrator_offset is initialized
        # pose_in is Pose2DStamped (x,y,heading) in the integrator frame
        # pose_out is PoseStamped (3d + quat) in the intersection frame

        xin = pose_in.x
        yin = pose_in.y
        thetain = pose_in.theta # needs some inertia

        xoff = self.integrator_offset[0]
        yoff = self.integrator_offset[1]
        thetaoff = self.integrator_offset[2]

        # TODO: stash the transformation
        x = xin*np.cos(thetaoff) - yin*np.sin(thetaoff) + xoff
        y = xin*np.sin(thetaoff) + yin*np.cos(thetaoff) + yoff

        theta = thetain + thetaoff

        if self.parameters['~damping']:
            # Ideally damping theta should be where the integration happens, even
            #   better, the integration could use a basic dynamic model.
            # Slow it down by taking the difference, scaling it and re-adding it
            prev_theta = euler_from_quaternion(utils.quat_to_tuple(self.integrated_pose.pose.orientation))[2]

            # for reference: theta upon entering intersection is pi/2
            # constrain it with continuity up to a U-turn (-pi/2, 3pi/2)
            prev_theta = ( (prev_theta + np.pi/2) % (2*np.pi) ) - np.pi/2
            theta = ( (theta + np.pi/2) % (2*np.pi) ) - np.pi/2

            dtheta = theta - prev_theta

            theta = prev_theta + self.parameters['~omega_factor']*dtheta

        quat = unit_vector(quaternion_from_euler(0, 0, theta, axes='sxyz'))

        out = utils.pose_stamped('intersection', (x,y,0), quat)
        return out


    def update_integrator_offset(self):
        # Integrator offset is the pose of the integrator frame wrt intersection frame.
        # It is just a tuple (x,y,theta)
        # We get it by comparing self.pose_in and self.pose, which are assumed
        # to be synchronised before this method is called
        self.integrated_pose = self.pose

        # latest pose from the integrator is needed to find the offset
        if self.pose_in is None: return

        xin = self.pose_in.x
        yin = self.pose_in.y
        thetain = self.pose_in.theta

        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        theta = euler_from_quaternion(utils.quat_to_tuple(self.pose.pose.orientation))[2]

        thetaoff = theta - thetain
        # align axes first
        xoff = x - xin*np.cos(thetaoff) + yin*np.sin(thetaoff)
        yoff = y - xin*np.sin(thetaoff) - yin*np.cos(thetaoff)

        self.integrator_offset = (xoff, yoff, thetaoff)

        if self.verbose:
            self.log('odom x=%.3f\ty=%.3f\tth=%.3f'%(xin,yin,thetain))
            self.log('pose x=%.3f\ty=%.3f\tth=%.3f'%(x,y,theta))
            self.log('offs x=%.3f\ty=%.3f\tth=%.3f'%(xoff,yoff,thetaoff))


    # If the node is active, not resetting or shutting down a pose will be
    # published every time this callback is run.
    # The pose is estimated with the image. Plain integration is a fallback
    def cb_image_in(self, msg):
        if self.is_shutdown: return
        if self.resetting == True: return

        if self.parametersChanged:
            self.log('Parameters changed.', 'info')
            self.refresh_parameters()
            self.parametersChanged = False

        if not self.active: return

        if self.verbose:
            self.log('Received image.', 'info')
        tk = TimeKeeper(msg)

        img_original = utils.read_image(msg)
        if img_original is None:
            return
        tk.completed('decode image')

        # helps to avoid mismatch if we had a frame without stopline detections
        if self.parameters['~integration_assisted_clustering']:
            if self.verbose:
                self.log('using integrated pose to predict cluster centers','info')
            self.pose = self.integrated_pose
        stopline_poses_predicted = self.model.get_stopline_poses_reference(self.pose.pose)
        if self.verbose:
            # To see them in rviz: (they are also in colour on the frame)
            self.publish_pose_array(self.pub_stoplines_predicted, 'axle', stopline_poses_predicted)
        tk.completed('predict poses')

        # Filtering red
        red_mask, dbg_image = self.stopline_detector.filter_red(img_original, verbose=self.verbose, tk=tk)
        if self.verbose and dbg_image is not None:
            utils.publish_image(self.bridge, self.pub_red_filter, dbg_image)

        # Clustering red points
        clusters = self.stopline_detector.cluster_mask(red_mask, self.dbscan_eps, self.dbscan_min_samples, tk=tk)
        # Calculate quality indicators of clusters
        cluster_qualities = list([self.stopline_detector.cluster_quality(c) for c in clusters])

        # Calculate stopline poses from clusters
        stopline_poses_measured = []
        for cluster_points in clusters: # may be empty
            pose = self.stopline_detector.calculate_pose_from_points(cluster_points)
            stopline_poses_measured.append(pose)
        tk.completed('stopline pose calculations')

        # Classify measured poses based on the predicted poses
        labels = self.stopline_detector.classify_poses(stopline_poses_measured, stopline_poses_predicted)
        tk.completed('classifying poses')

        # Correct orientation of measured poses based on the predicted poses
        stopline_poses_corrected = []
        for pose_measured, label in zip(stopline_poses_measured, labels):
            corrected = self.stopline_detector.correct_orientation(pose_measured, stopline_poses_predicted[label])
            stopline_poses_corrected.append(corrected)
        tk.completed('corrected poses')

        # Calculate pose estimate of axle in intersection coordinate frame
        poses_estimated = []
        for stopline_pose, label in zip(stopline_poses_corrected, labels):
            pose_estimation = self.model.get_axle_pose_from_stopline_pose(stopline_pose, label)
            poses_estimated.append(pose_estimation.pose)
        tk.completed('pose estimations')

        # Filter pose estimates
        best_pose_estimate = None
        if len(poses_estimated) > 0:
            self.stopline_filter.update_stopline_poses(poses_estimated, cluster_qualities)
            best_pose_estimate = self.stopline_filter.get_best_estimate()

        if best_pose_estimate is not None: # at least one is good enough
            if self.verbose:
                self.log('got a good pose from stoplines','info')

            self.pose.header.stamp = msg.header.stamp
            self.pose.pose = best_pose_estimate
            self.pub_pose(self.pose)
            if self.verbose:
                qual = self.stopline_filter.get_estimate_quality()
                self.pub_quality(qual)
            if self.integration_enabled:
                self.update_integrator_offset()

        elif self.integration_enabled and self.integrated_pose is not None:
            if self.verbose:
                self.log('fall back to integrated pose','info')
            self.integrated_pose.header.stamp = msg.header.stamp
            self.pub_pose(self.integrated_pose)

        if self.verbose:
            if len(stopline_poses_corrected) > 0:
                self.publish_pose_array(self.pub_stoplines_measured, 'axle', stopline_poses_corrected)
                self.publish_pose_array(self.pub_pose_estimates, 'intersection', poses_estimated)

            # Feature proposal: display quality on the cluster markers
            img_debug = self.stopline_detector.draw_debug_image(img_original, stopline_poses_predicted,
                                                                stopline_poses_corrected, clusters, labels)
            utils.publish_image(self.bridge, self.pub_clustering, img_debug)
            tk.completed('debug image')

        if self.parameters['~show_time_keeping']:
            self.log(tk.getall())

    def publish_pose_array(self, publisher, frame_id, poses):
        pose_array = PoseArray()
        pose_array.header.frame_id = frame_id
        pose_array.header.stamp = rospy.Time.now()
        pose_array.poses = poses
        publisher.publish(pose_array)


    def refresh_parameters(self):
        self.verbose = self.parameters['~verbose']
        self.start_x = self.parameters['~start_x']
        self.start_y = self.parameters['~start_y']
        self.integration_enabled = self.parameters['~integration_enabled']
        self.start_z = self.parameters['~start_z']
        self.stop_time = self.parameters['~stop_time']
        self.dbscan_eps = self.parameters['~dbscan_eps']
        self.dbscan_min_samples = self.parameters['~dbscan_min_samples']
        self.stopline_filter = StoplineFilter(min_quality=self.parameters['~min_quality'], policy='weighted_avg')


    def onShutdown(self):
        self.log("Stopping localization_node.")
        super(LocalizationNode, self).onShutdown()


if __name__ == '__main__':
    # Initialize the node
    localization_node = LocalizationNode(node_name='localization_node') # Keep it spinning to keep the node alive
    rospy.spin()
