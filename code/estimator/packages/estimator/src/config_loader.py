import os
import yaml

import numpy as np
import duckietown_utils as dtu

from sensor_msgs.msg import CameraInfo


# Taken from https://github.com/duckietown/dt-core/blob/2db2abcd9e5d7d012f318b872d15b23682023450/packages/ground_projection/include/ground_projection/configuration.py
def get_homography_for_robot(robot_name):
    dtu.check_isinstance(robot_name, str)
    # find the file
    if robot_name == dtu.DuckietownConstants.ROBOT_NAME_FOR_TESTS:
        data = dtu.yaml_load(homography_default)
    else:
        fn = get_homography_info_config_file(robot_name)
        # load the YAML
        data = dtu.yaml_load_file(fn)

    # convert the YAML
    homography = homography_from_yaml(data)
    #check_homography_sane_for_DB17(homography)
    return homography

# Taken from https://github.com/duckietown/dt-core/blob/2db2abcd9e5d7d012f318b872d15b23682023450/packages/ground_projection/include/ground_projection/configuration.py
def homography_from_yaml(data):
    try:
        h = data['homography']
        res = np.array(h).reshape((3, 3))
        return res
    except Exception as e:
        msg = 'Could not interpret data:'
        msg += '\n\n' + dtu.indent(yaml.dump(data), '   ')
        dtu.raise_wrapped(InvalidHomographyInfo, e, msg)

# Taken from https://github.com/duckietown/dt-core/blob/2db2abcd9e5d7d012f318b872d15b23682023450/packages/ground_projection/include/ground_projection/configuration.py
def get_homography_info_config_file(robot_name):
    strict = False
    roots = [os.path.join(dtu.get_duckiefleet_root(), 'calibrations'),
             os.path.join(dtu.get_ros_package_path('duckietown'), 'config', 'baseline', 'calibration')]

    found = []
    for df in roots:
    # Load camera information
        fn = os.path.join(df, 'camera_extrinsic', robot_name + '.yaml')
        fn_default = os.path.join(df, 'camera_extrinsic', 'default.yaml')
        if os.path.exists(fn):
            found.append(fn)
            dtu.logger.info("Using filename %s" % fn)
        elif os.path.exists(fn_default):
            found.append(fn_default)
            dtu.logger.info("Using filename %s" % fn_default)

    if len(found) == 0:
        msg = 'Cannot find homography file for robot %r;\n%s' % (robot_name, roots)
        raise NoHomographyInfoAvailable(msg)
    elif len(found) == 1:
        return found[0]
    else:
        msg = 'Found more than one configuration file: \n%s' % "\n".join(found)
        msg += "\n Please delete one of those."
        if strict:
            raise Exception(msg)
        else:
            dtu.logger.error(msg)
            return found[0]


# Taken from https://github.com/duckietown/dt-core/blob/2db2abcd9e5d7d012f318b872d15b23682023450/packages/pi_camera/include/pi_camera/camera_info.py
class NoCameraInfoAvailable(dtu.DTException):
    pass


# Taken from https://github.com/duckietown/dt-core/blob/2db2abcd9e5d7d012f318b872d15b23682023450/packages/pi_camera/include/pi_camera/camera_info.py
class InvalidCameraInfo(dtu.DTException):
    pass

# Taken from https://github.com/duckietown/dt-core/blob/2db2abcd9e5d7d012f318b872d15b23682023450/packages/pi_camera/include/pi_camera/camera_info.py
def get_camera_info_for_robot(robot_name):
    """
        Returns a CameraInfo object for the given robot.
        This is in a good format to pass to PinholeCameraModel:
            self.pcm = PinholeCameraModel()
            self.pcm.fromCameraInfo(self.ci)
        The fields are simply lists (not array or matrix).
        Raises:
            NoCameraInfoAvailable  if no info available
            InvalidCameraInfo   if the info exists but is invalid
    """

    if robot_name == dtu.DuckietownConstants.ROBOT_NAME_FOR_TESTS:
        calib_data = dtu.yaml_load(default_camera_info)
    else:
        # find the file
        fn = get_camera_info_config_file(robot_name)

        # load the YAML

        calib_data = dtu.yaml_load_file(fn, plain_yaml=True)

    # convert the YAML
    try:
        camera_info = camera_info_from_yaml(calib_data)
    except InvalidCameraInfo as e:
        msg = 'Invalid data in file %s' % fn
        dtu.raise_wrapped(InvalidCameraInfo, e, msg)

    #check_camera_info_sane_for_DB17(camera_info)

    return camera_info

# Taken from https://github.com/duckietown/dt-core/blob/2db2abcd9e5d7d012f318b872d15b23682023450/packages/pi_camera/include/pi_camera/camera_info.py
def get_camera_info_config_file(robot_name):
    roots = [os.path.join(dtu.get_duckiefleet_root(), 'calibrations'),
             os.path.join(dtu.get_ros_package_path('duckietown'), 'config', 'baseline', 'calibration')]

    for df in roots:
    # Load camera information
        fn = os.path.join(df, 'camera_intrinsic', robot_name + '.yaml')
        fn_default = os.path.join(df, 'camera_intrinsic', 'default.yaml')
        if os.path.exists(fn):
            return fn
        elif os.path.exists(fn_default):
            return fn_default
        else:
            print('%s does not exist and neither does %s' % (fn, fn_default))

    msg = 'Cannot find intrinsic file for robot %r;\n%s' % (robot_name, roots)
    raise NoCameraInfoAvailable(msg)

# Taken from https://github.com/duckietown/dt-core/blob/2db2abcd9e5d7d012f318b872d15b23682023450/packages/pi_camera/include/pi_camera/camera_info.py
def camera_info_from_yaml(calib_data):
    try:
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
#         cam_info.K = np.matrix(calib_data['camera_matrix']['data']).reshape((3,3))
#         cam_info.D = np.matrix(calib_data['distortion_coefficients']['data']).reshape((1,5))
#         cam_info.R = np.matrix(calib_data['rectification_matrix']['data']).reshape((3,3))
#         cam_info.P = np.matrix(calib_data['projection_matrix']['data']).reshape((3,4))
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']

        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info
    except Exception as e:
        msg = 'Could not interpret data:'
        msg += '\n\n' + dtu.indent(yaml.dump(calib_data), '   ')
        dtu.raise_wrapped(InvalidCameraInfo, e, msg)
