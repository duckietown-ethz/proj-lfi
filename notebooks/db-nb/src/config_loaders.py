import os.path
from os import environ
import numpy as np
from duckietown_utils import logger, get_duckiefleet_root
from duckietown_utils.yaml_wrap import yaml_load_file

# the following is a rospy package, but we only use for the datatype, no ROS
# behaviour in this file.
# would be better to use image_geometry.PinholeCameraModel like all daffy code
# but I would have to include it in the package and have it built by catkin
from sensor_msgs.msg import CameraInfo

def get_robot_name():
    return environ['VEHICLE_NAME']

# these methods are constrained to return calibration values for the robot you
# are running the code on. Access to /data/ is required. Don't forget to
# mount it to your docker container.

def load_homography():
    '''Load homography (extrinsic parameters)'''
    robot_name = get_robot_name()
    filename = (get_duckiefleet_root() + "/calibrations/camera_extrinsic/" + robot_name + ".yaml")
    if not os.path.isfile(filename):
        logger.warn("no extrinsic calibration parameters for {}, trying default".format(robot_name))
        filename = (get_duckiefleet_root() + "/calibrations/camera_extrinsic/default.yaml")
        if not os.path.isfile(filename):
            logger.error("can't find default either, something's wrong")
        else:
            data = yaml_load_file(filename)
    else:
        logger.info("Using extrinsic calibration of " + robot_name)
        data = yaml_load_file(filename)
    logger.info("Loaded homography for {}".format(os.path.basename(filename)))
    H = np.array(data['homography']).reshape((3,3))
    return H

def load_camera_info():
    '''Load camera intrinsics'''
    robot_name = get_robot_name()
    filename = (get_duckiefleet_root() + "/calibrations/camera_intrinsic/" + robot_name + ".yaml")
    if not os.path.isfile(filename):
        filename = (os.environ['DUCKIEFLEET_ROOT'] + "/calibrations/camera_intrinsic/default.yaml")
    calib_data = yaml_load_file(filename)
    cam_info = CameraInfo()
    cam_info.width = calib_data['image_width']
    cam_info.height = calib_data['image_height']
    cam_info.K = np.array(calib_data['camera_matrix']['data']).reshape((3,3))
    cam_info.D = np.array(calib_data['distortion_coefficients']['data']).reshape((1,5))
    cam_info.R = np.array(calib_data['rectification_matrix']['data']).reshape((3,3))
    cam_info.P = np.array(calib_data['projection_matrix']['data']).reshape((3,4))
    cam_info.distortion_model = calib_data['distortion_model']

    return cam_info
