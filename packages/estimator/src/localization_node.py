#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import rospkg

from duckietown import DTROS
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Point, Quaternion, PoseStamped, PoseArray
from std_msgs.msg import Bool
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
        self.active = True
        # Initialize parameters
        self.parameters['~verbose'] = None

        rospy.set_param('~omega_factor', 1)
        self.parameters['~omega_factor'] = 1

        rospy.set_param('~stop_time', 2)
        self.parameters['~stop_time'] = 2

        rospy.set_param('~integration_enabled', True)
        self.parameters['~integration_enabled'] = True

        self.parameters['/{}/birdseye_node/image_size'.format(self.veh_name)] = None
        self.parameters['~start_x'] = None
        self.parameters['~start_y'] = None
        self.parameters['~start_z'] = None
        self.parameters['~dbscan_eps'] = None
        self.parameters['~dbscan_min_samples'] = None
        self.updateParameters()
        self.refresh_parameters()

        # Subscribers
        self.sub_camera_info = self.subscriber('~camera_info', CameraInfo, self.cb_camera_info, queue_size=1)
        self.sub_reset = self.subscriber('~reset', Bool, self.cb_reset, queue_size=1)
        self.sub_pose_in = self.subscriber('~open_loop_pose_estimate', Pose2DStamped, self.cb_pose_in, queue_size=1)
        self.sub_switch = self.subscriber("~switch", BoolStamped, self.cbSwitch, queue_size=1)

        #TODO: listen to FSM mode and call reset when we enter NAVIGATION_COORDINATION
        self.sub_fsm = self.subscriber('~mode', FSMState, self.cb_mode, queue_size=1)

        # Publishers
        self.pub_clustering = self.publisher('~verbose/clustering/compressed', CompressedImage, queue_size=1)
        self.pub_red_filter = self.publisher('~verbose/red_filter/compressed', CompressedImage, queue_size=1)
        self.pub_stoplines_measured = self.publisher('~stoplines_measured', PoseArray, queue_size=1)
        self.pub_stoplines_predicted = self.publisher('~stoplines_predicted', PoseArray, queue_size=1)
        self.pub_pose_estimates = self.publisher('~pose_estimates', PoseArray, queue_size=1)
        self.pub_best_pose_estimate = self.publisher('~best_pose_estimate', PoseStamped, queue_size=1)
        # XXX:
        self.intersection_go = self.publisher('/{}/coordinator_node/intersection_go'.format(self.veh_name), BoolStamped, queue_size=1)

        self.bridge = CvBridge()

        self.pose = None
        self.pose_in = None
        self.integrator_offset = None
        self.integrated_pose = None

        self.last_thetain = None # for simulating inertia

        self.reset()

        self.scaled_homography = None

        self.resetting = False

        self.log('Waiting for camera to update its parameters.')

    def cb_mode(self,fsm_state_msg):
        if fsm_state_msg.state == "INTERSECTION_COORDINATION":
            self.log('Setting up for intersection')
            self.reset()
        # this is just a Hack so I don't have to do it manually
        msg = BoolStamped()
        msg.data = True
        rospy.sleep(self.stop_time)
        self.intersection_go.publish(msg)

    def cbSwitch(self, switch_msg):
        self.log('Switch ' + str(switch_msg.data))

        self.active = switch_msg.data

    def cb_camera_info(self, msg):
        # if self.image_size_width != msg.width or self.image_size_height != msg.height:
        #     return
        self.log('Received camera info.', 'info')
        self.pcm = PinholeCameraModel()
        self.pcm.fromCameraInfo(msg)
        homography = get_homography_for_robot(self.veh_name)
        camera_info = get_camera_info_for_robot(self.veh_name)
        self.scaled_homography = ScaledHomography(homography, camera_info.height, camera_info.width)
        self.model = Intersection4wayModel(self.pcm, self.scaled_homography)
        self.stopline_detector = StoplineDetector(self.scaled_homography, point_reduction_factor=1)
        self.stopline_filter = StoplineFilter(min_quality=0.5, policy='weighted_avg')

        # This topic subscription is only needed initially, so it can be unregistered.
        self.sub_camera_info.unregister()
        self.sub_camera_info = self.subscriber('~camera_info', CameraInfo, self.scaled_homography.cb_camera_info, queue_size=1)
        buffer_size = msg.width * msg.height * 3 * 2
        self.log('Buffer size set to {}.'.format(buffer_size), 'info')
        # Now the node can proceed to process images
        self.sub_image_in = self.subscriber('~image_in/compressed', CompressedImage, self.cb_image_in, queue_size=1, buff_size=buffer_size)

        self.log('Initialized.')

    def cb_pose_in(self, pose2d_in):
        self.pose_in = pose2d_in
        if self.integrator_offset is None:
            self.update_integrator_offset()
        else:
            self.integrated_pose = self.correct(pose2d_in)

    def pub_pose(self):
        self.pub_best_pose_estimate.publish(self.pose)

    def pub_integrated_pose(self):
        self.pub_best_pose_estimate.publish(self.integrated_pose)

    def cb_reset(self, msg):
        do_reset = msg.data
        self.log('got reset command: '+str(do_reset))
        if do_reset:
            self.reset()

    def reset(self):
        self.log('Intersection localization is resetting')
        self.resetting = True
        # TODO: replace with mutex or workaround
        rospy.sleep(.2) # wait for current image to finish processing
        position = [self.start_x, self.start_y, self.start_z]
        heading = np.pi/2.0 # TODO: make this a param as well
        orientation = unit_vector(quaternion_from_euler(0, 0, heading, axes='sxyz'))
        self.pose = utils.pose_stamped('intersection', position, orientation)
        self.pub_pose()
        self.integrator_offset = None
        self.pose_in = None
        self.resetting = False

    def correct(self, pose_in):
        # assume integrator_offset is initialized
        # pose_in is Pose2DStamped (x,y,heading) in the integrator frame
        # pose_out is PoseStamped (3d + quat) in the intersection frame

        # unpack stuff we'll use
        xin = pose_in.x
        yin = pose_in.y
        thetain = pose_in.theta # <--- needs some inertia, simulated by damping
        # ideally damping should be where the integration happens, even
        #   better, the integration could use a basic dynamic model
        prev_theta = euler_from_quaternion(utils.quat_to_tuple(self.integrated_pose.pose.orientation))[2]

        if self.last_thetain is not None:
            dtheta = thetain - self.last_thetain
            thetain = self.last_thetain + self.parameters['~omega_factor']*dtheta
        self.last_thetain = thetain

        xoff = self.integrator_offset[0]
        yoff = self.integrator_offset[1]
        thetaoff = self.integrator_offset[2]

        #TODO: stash the transformation
        x = xin*np.cos(thetaoff) - yin*np.sin(thetaoff) + xoff
        y = xin*np.sin(thetaoff) + yin*np.cos(thetaoff) + yoff
        theta = thetain + thetaoff
        quat = unit_vector(quaternion_from_euler(0, 0, theta, axes='sxyz'))

        out = utils.pose_stamped('intersection', (x,y,0), quat)
        return out

    def update_integrator_offset(self):
        # Integrator offset is the pose of the integrator frame wrt intersection frame.
        # It is just a tuple (x,y,theta)
        # We get it by comparing self.pose_in and self.pose, which are assumed
        # to be synchronised before this method is called
        if self.pose_in is None: return
        # self.pose doesn't need to be checked as it is always initialized
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
        self.integrated_pose = self.pose

        if self.verbose:
            self.log('odom x=%.3f\ty=%.3f\tth=%.3f'%(xin,yin,thetain))
            self.log('pose x=%.3f\ty=%.3f\tth=%.3f'%(x,y,theta))
            self.log('offs x=%.3f\ty=%.3f\tth=%.3f'%(xoff,yoff,thetaoff))

    def cb_image_in(self, msg):
        if self.resetting == True: return
        if self.is_shutdown: return
        # TODO Change to debug level
        if self.verbose:
            self.log('Received image.', 'info')
        tk = TimeKeeper(msg)

        if self.parametersChanged:
            self.log('Parameters changed.', 'info')
            self.refresh_parameters()
            self.parametersChanged = False
        if not self.active: return

        tk.completed('parameter check')
        img_original = utils.read_image(msg)
        if img_original is None:
            return

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

        if best_pose_estimate is not None:
            self.pose.header.stamp = msg.header.stamp
            self.pose.header.frame_id = 'intersection'
            self.pose.pose = best_pose_estimate
            self.pub_pose()
            self.update_integrator_offset()
        elif self.parameters['~integration_enabled'] and self.integrated_pose is not None:
            self.integrated_pose.header.stamp = msg.header.stamp
            self.pub_integrated_pose()

        if self.verbose:
            if len(stopline_poses_corrected) > 0:
                self.publish_pose_array(self.pub_stoplines_measured, 'axle', stopline_poses_corrected)
                self.publish_pose_array(self.pub_pose_estimates, 'intersection', poses_estimated)

            # TODO Write cluster quality in cluster center
            img_debug = self.stopline_detector.draw_debug_image(img_original, stopline_poses_predicted,
                                                                stopline_poses_corrected, clusters, labels)
            utils.publish_image(self.bridge, self.pub_clustering, img_debug)
            tk.completed('debug image')

            self.log(tk.getall())


    def draw_position_on_intersection(self, position_meter, angle):
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('estimator')
        path = self.pkg_path + "/src/four-way-intersection.png"
        image = cv2.imread(path)
        height, width, _ = image.shape
        image = cv2.copyMakeBorder(image, height/2, height/2, width/2, width/2, cv2.BORDER_CONSTANT, None, (0,0,0))
        height_m = 0.61
        width_m = 0.61

        offset = np.array([height/2 + height, width/2 + (width*3)/4])
        position_intersection = - position_meter * np.array([height / height_m, width / width_m])
        position_image = offset + position_intersection.astype(int)

        length = 100
        angle_rad = np.deg2rad(180.0 - angle)
        pt1 = (position_image[1], position_image[0])
        pt2 = (int(pt1[0] + length * np.sin(angle_rad)), int(pt1[1] + length * np.cos(angle_rad)))

        image = cv2.arrowedLine(image, pt1, pt2, (0,255,0), 5)

        return image


    def publish_pose_array(self, publisher, frame_id, poses):
        pose_array = PoseArray()
        pose_array.header.frame_id = frame_id
        pose_array.header.stamp = rospy.Time.now()
        pose_array.poses = poses
        publisher.publish(pose_array)


    def refresh_parameters(self):
        self.verbose = self.parameters['~verbose']
        image_size = self.parameters['/{}/birdseye_node/image_size'.format(self.veh_name)]
        self.image_size_height = image_size['height']
        self.image_size_width = image_size['width']
        self.start_x = self.parameters['~start_x']
        self.start_y = self.parameters['~start_y']
        self.integration_enabled = self.parameters['~integration_enabled']
        self.start_z = self.parameters['~start_z']
        self.stop_time = self.parameters['~stop_time']
        self.dbscan_eps = self.parameters['~dbscan_eps']
        self.dbscan_min_samples = self.parameters['~dbscan_min_samples']


    def onShutdown(self):
        self.log("Stopping localization_node.")
        super(LocalizationNode, self).onShutdown()


if __name__ == '__main__':
    # Initialize the node
    localization_node = LocalizationNode(node_name='localization_node') # Keep it spinning to keep the node alive
    rospy.spin()
