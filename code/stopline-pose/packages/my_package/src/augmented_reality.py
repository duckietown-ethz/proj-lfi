#!/usr/bin/env python

import cv2
import numpy as np
import os
import rospy
import yaml

from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
from cv_bridge import CvBridge
from numpy import linalg as LA



class Augmented:

    def __init__(self, node_name):

        print('initialized')
        # Initialize the DTROS parent class
        self.veh_name = os.environ['VEHICLE_NAME']
        self.map_name = os.environ['MAP_NAME']

        # Set parameters using a robot-specific yaml file if such exists
        self.readParamFromFile()
        print('initialized augmented')






    def process_image(self, data):
        cv_image = CvBridge().compressed_imgmsg_to_cv2(data, desired_encoding="passthrough")
        finalImage = cv_image.copy()

        self.readParamFromFile()

        self.parameters['~homography'] = np.reshape(np.array(rospy.get_param('~homography')), (3,3))
        self.parameters['~camera_matrix'] = np.array(rospy.get_param('~camera_matrix'))
        self.parameters['~distortion_coefficients'] = np.array(rospy.get_param('~distortion_coefficients'))
        self.parameters['~projection_matrix'] = np.array(rospy.get_param('~projection_matrix'))
        self.parameters['~rectification_matrix'] = np.array(rospy.get_param('~rectification_matrix'))

        self.parameters['~camera_matrix'] = np.reshape(self.parameters['~camera_matrix'], (3, 3))
        self.parameters['~distortion_coefficients'] = np.reshape(self.parameters['~distortion_coefficients'], (1, 5))
        self.parameters['~projection_matrix'] = np.reshape(self.parameters['~projection_matrix'], (3, 4))
        self.parameters['~rectification_matrix'] = np.reshape(self.parameters['~rectification_matrix'], (3, 3))


        imageHeight, imageWidth, channels = finalImage.shape

        # Used for rectification
        self.mapx = np.ndarray(shape=(imageHeight, imageWidth, 1), dtype='float32')
        self.mapy = np.ndarray(shape=(imageHeight, imageWidth, 1), dtype='float32')


        self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.parameters['~camera_matrix'],
            self.parameters['~distortion_coefficients'], self.parameters['~rectification_matrix'],
            self.parameters['~projection_matrix'], (imageWidth, imageHeight), cv2.CV_32FC1)

        undistorted_image = cv2.remap(finalImage, self.mapx, self.mapy, cv2.INTER_CUBIC)
        mask_image = self.color_red(undistorted_image)
        keypoints_pixel_img = self.corner_detector(mask_image)
        keypoint_image = cv2.drawKeypoints(mask_image, keypoints_pixel_img, outImage=np.array([]), color=(0, 0, 255))
        keypoints_ground_world = self.pixel2ground(keypoints_pixel_img, self.parameters['~homography'])
        keypoints_world_single = self.delete_duplicates(keypoints_ground_world)
        angle, long_dist, lat_dist = self.find_distances(keypoints_world_single)

        return keypoint_image

    def color_red(self, image1):
        # It converts the BGR color space of image to HSV color space
        hsv = cv2.cvtColor(image1, cv2.COLOR_BGR2HSV)

        # Threshold of red in HSV space
        hsv_red1 = np.array([0, 140, 100])
        hsv_red2 = np.array([15, 255, 255])
        hsv_red3 = np.array([165, 140, 100])
        hsv_red4 = np.array([180, 255, 255])

        # preparing the mask to overlay
        bw1 = cv2.inRange(hsv, hsv_red1, hsv_red2)
        bw2 = cv2.inRange(hsv, hsv_red3, hsv_red4)
        mask = cv2.bitwise_or(bw1, bw2)

        return mask

    def corner_detector(self, image1):
        # smoothing
        image1 = cv2.GaussianBlur(image1, (15, 15), 0)

        # Detect keypoints with non max suppression
        fast = cv2.FastFeatureDetector_create(threshold=7)
        keypoints = fast.detect(image1, None)

        # Print the number of keypoints detected in the training image
        #print("Number of Keypoints Detected In The Image With Non Max Suppression: ", len(keypoints))

        # delete keypoint in the upper 60% of image
        for i in range(len(keypoints) - 1, -1, -1):
            if keypoints[i].pt[1] < 480 * 0.6:
                del keypoints[i]

        #print("Number of Keypoints Detected In The Image Without upper image part: ", len(keypoints))

        return keypoints

    def pixel2ground(self, keypoints_pixel_img, homography):

        keypoints_cam = np.zeros((len(keypoints_pixel_img), 2))

        for i in range(len(keypoints_pixel_img)):
            u = keypoints_pixel_img[i].pt[0]
            v = keypoints_pixel_img[i].pt[1]
            # print([keypoints_pixel_img[i].pt[0], keypoints_pixel_img[i].pt[1]])
            temp = np.matmul(homography, np.array([u, v, 1]))
            keypoints_cam[i, 0] = temp[0] / float(temp[2])
            keypoints_cam[i, 1] = - temp[1] / float(temp[2])
            # print(round(keypoints_cam[i,0], 3), round(keypoints_cam[i,1], 3))

        return keypoints_cam

    def delete_duplicates(self, keypoints):
        no_duplicates = np.ones((len(keypoints),), dtype=int)
        # find duplicates
        for i in range(len(keypoints)):
            for j in range(i + 1, len(keypoints)):
                if abs(keypoints[i, 0] - keypoints[j, 0]) < 0.01 and abs(keypoints[i, 1] - keypoints[j, 1]) < 0.01:
                    no_duplicates[j] = 0

        # find points that are too far away
        for i in range(len(keypoints)):
            if np.sqrt(keypoints[i, 0] ** 2 + keypoints[i, 1] ** 2) > 0.35:
                no_duplicates[i] = 0

        # print(no_duplicates)

        # append only non-duplicate values
        keypoints_single = np.empty((0, 2))
        for i in range(0, len(keypoints)):
            if no_duplicates[i] == 1:
                keypoints_single = np.append(keypoints_single, keypoints[i, :])

        keypoints_single = np.reshape(keypoints_single, (-1, 2))
        print("Number of Keypoints Detected In The Image after deleting duplicates: ", len(keypoints_single))
        # print(keypoints_single)

        return keypoints_single

    def find_distances(self, keypoints):
        if len(keypoints) == 1 or len(keypoints) == 0:
            print("WARNING: ONLY ONE KEYPOINT DETECTED --> CAN'T CALCULATE THE ANGLE & DISTANCES!")
            return [], [], []

        # find the two "front" points of the stopline
        dist_abs = np.zeros(len(keypoints))
        dist_rel = np.zeros(len(keypoints))
        for i in range(1, len(keypoints)):
            dist_abs[i] = LA.norm(keypoints[0, :] - keypoints[i, :])
            dist_rel[i] = abs(dist_abs[i] - 0.205)
        dist_rel[0] = 0.205
        # print(dist_abs)

        point2 = np.min(dist_rel)
        if point2 > 0.05:
            print("WARNING: UNABLE TO DETECT 2 FRONT POINTS --> CAN'T CALCULATE THE ANGLE & DISTANCES!")
            return [], [], []

        idx_pt2 = np.argmin(dist_rel)

        angle = np.arctan((keypoints[0, 0] - keypoints[idx_pt2, 0]) / (keypoints[0, 1] - keypoints[idx_pt2, 1]))

        keypoints_tf = np.zeros((2, 2))
        keypoints_tf[0, 0] = np.cos(angle) * keypoints[0, 0] - np.sin(angle) * keypoints[0, 1]
        keypoints_tf[0, 1] = np.sin(angle) * keypoints[0, 0] + np.cos(angle) * keypoints[0, 1]
        keypoints_tf[1, 0] = np.cos(angle) * keypoints[idx_pt2, 0] - np.sin(angle) * keypoints[idx_pt2, 1]
        keypoints_tf[1, 1] = np.sin(angle) * keypoints[idx_pt2, 0] + np.cos(angle) * keypoints[idx_pt2, 1]

        long_dist = (keypoints_tf[0, 0] + keypoints_tf[1, 0]) / 2 - 0.05
        if keypoints_tf[0, 1] > keypoints_tf[1, 1]:
            lat_dist = keypoints_tf[0, 1]
        else:
            lat_dist = keypoints_tf[1, 1]

        print("angle: " + str(90 - angle * 180 / np.pi))
        print("longitudinal distance to stopline: " + str(long_dist))
        print("lateral distance to white street line: " + str(lat_dist))

        return angle, long_dist, lat_dist


    def readParamFromFile(self):
        """
        Reads the saved parameters from
        `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml` or
        uses the default values if the file doesn't exist. Adjsuts
        the ROS paramaters for the node with the new values.

        """
        # Check file existence
        fname = self.getFilePath(self.veh_name, 'extrinsic')
        # Use the default values from the config folder if a
        # robot-specific file does not exist.
        if not os.path.isfile(fname):
            self.log("Kinematics calibration file %s does not "
                     "exist! Using the default file." % fname, type='warn')
            fname = self.getFilePath('default')

        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict is None:
            # Empty yaml file
            return
        for param_name in ["homography"]:
            param_value = yaml_dict.get(param_name)
            if param_name is not None:
                rospy.set_param("~"+param_name, param_value)
            else:
                # Skip if not defined, use default value instead.
                pass

        fnameExt = self.getFilePath(self.veh_name, 'intrinsic')

        # Use the default values from the config folder if a
        # robot-specific file does not exist.
        if not os.path.isfile(fnameExt):
            self.log("Kinematics calibration file %s does not "
                     "exist! Using the default file." % fnameExt, type='warn')
            fnameExt = self.getFilePath('default')

        with open(fnameExt, 'r') as in_file:
            try:
                yaml_dict2 = yaml.load(in_file)
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fnameExt, exc), type='fatal')
                rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict2 is None:
            # Empty yaml file
            return
        for param_name2 in ["camera_matrix", "distortion_coefficients", "projection_matrix", "rectification_matrix"]:
            param_value2 = yaml_dict2.get(param_name2)['data']

            # "camera_matrix", "distortion_coefficients", "projection_matrix", "rectification_matrix"
            if param_name2 is not None:
                rospy.set_param("~"+param_name2, param_value2)
            else:
                # Skip if not defined, use default value instead.
                pass

    def getFilePath(self, name, cameraCalibration):
        """
        Returns the path to the robot-specific configuration file,
        i.e. `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`.

        Args:
            name (:obj:`str`): the Duckiebot name

        Returns:
            :obj:`str`: the full path to the robot-specific
                calibration file

        """
        if cameraCalibration == 'extrinsic':
            cali_file_folder = '/data/config/calibrations/camera_extrinsic/'

        if cameraCalibration == 'intrinsic':
            cali_file_folder = '/data/config/calibrations/camera_intrinsic/'

        cali_file = cali_file_folder + name + ".yaml"
        return cali_file
