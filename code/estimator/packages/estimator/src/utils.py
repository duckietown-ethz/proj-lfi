import numpy as np
import rospy
from duckietown_utils.jpg import bgr_from_jpg
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Quaternion, PoseStamped, TransformStamped
from tf.transformations import quaternion_inverse, quaternion_multiply
from tf import TransformerROS


def read_image(image_msg):
    try:
        image = bgr_from_jpg(image_msg.data)
        return image
    except ValueError as e:
        rospy.logerr(e)
        return None


def publish_image(bridge, publisher, image):
    try:
        image_msg = bridge.cv2_to_compressed_imgmsg(image, dst_format = "jpg")
    except CvBridgeError as e:
        rospy.logerr(e)
        return

    image_msg.header.stamp = rospy.Time.now()
    publisher.publish(image_msg)


def tuple_to_quat(tup):
    quaternion = Quaternion()
    quaternion.x = tup[0]
    quaternion.y = tup[1]
    quaternion.z = tup[2]
    quaternion.w = tup[3]
    return quaternion


def quat_to_tuple(quaternion):
    return (quaternion.x, quaternion.y, quaternion.z, quaternion.w)


# TODO Rename to pose_to_tuples
def pose_to_tuple(pose):
    position = (pose.position.x, pose.position.y, pose.position.z)
    orientation = quat_to_tuple(pose.orientation)
    return position, orientation


# Taken from https://stackoverflow.com/questions/23260939/distance-or-angular-magnitude-between-two-quaternions?noredirect=1
# def quat_distance(quaternion1, quaternion2):
#     quat_diff = quaternion_multiply(quaternion_inverse(quaternion1), quaternion2)
#     angle = 2 * np.arctan2(np.linalg.norm(quat_diff[0:2]), quat_diff[3])
#     return angle


# Taken from https://github.com/KieranWynn/pyquaternion/blob/446c31cba66b708e8480871e70b06415c3cb3b0f/pyquaternion/quaternion.py#L772
def quat_distance(q0, q1):
    q0_minus_q1 = q0 - q1
    q0_plus_q1  = q0 + q1
    d_minus = np.linalg.norm(q0_minus_q1)
    d_plus  = np.linalg.norm(q0_plus_q1)
    if d_minus < d_plus:
        return d_minus
    else:
        return d_plus

# Change this to get_pose_stamped and add another function get_pose
def get_pose(frame, position, orientation):
    p = PoseStamped()
    p.header.stamp = rospy.Time(0)
    p.header.frame_id = frame

    p.pose.position.x = position[0]
    p.pose.position.y = position[1]
    p.pose.position.z = position[2]

    p.pose.orientation.x = orientation[0]
    p.pose.orientation.y = orientation[1]
    p.pose.orientation.z = orientation[2]
    p.pose.orientation.w = orientation[3]

    return p

def stamp_pose(frame, pose):
    p = PoseStamped()
    p.header.stamp = rospy.Time(0)
    p.header.frame_id = frame

    p.pose.position = pose.position
    p.pose.orientation = pose.orientation

    return p


def get_transform(frame_parent, frame_child, position, orientation):
    t = TransformStamped()
    t.header.frame_id = frame_parent
    now = rospy.Time.now()
    t.header.stamp = now
    print('Setting transformation for ' + str(now))
    t.child_frame_id = frame_child
    t.transform.translation.x = position[0]
    t.transform.translation.y = position[1]
    t.transform.translation.z = position[2]

    t.transform.rotation.x = orientation[0]
    t.transform.rotation.y = orientation[1]
    t.transform.rotation.z = orientation[2]
    t.transform.rotation.w = orientation[3]

    return t

def invert_pose(pose):
    position, orientation = pose_to_tuple(pose)

    transformer = TransformerROS()
    transform = get_transform('frame1', 'frame2', position, orientation)
    transformer.setTransform(transform)
    frame1_origin_frame1 = get_pose('frame1', [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    frame1_origin_frame2 = transformer.transformPose('frame2', frame1_origin_frame1)

    return frame1_origin_frame2.pose



#     inverted = PoseStamped()
#     inverted.header.stamp = pose.header.stamp
#     inverted.header.frame_id = frame_id
# 
#     inverted.pose.position.x = -pose.pose.position.x
#     inverted.pose.position.y = -pose.pose.position.y
#     inverted.pose.position.z = -pose.pose.position.z
# 
#     inverted.pose.orientation = tuple_to_quat(quaternion_inverse(quat_to_tuple(pose.pose.orientation)))
# 
#     return inverted


def apply_homogeneous_transform(point, matrix, invert=False):
    point_x = point[0]
    point_y = point[1]

    point_homegeneous = np.array([point_x, point_y, 1.0])
    if invert:
        result_homogeneous = np.linalg.solve(matrix, point_homegeneous)
    else:
        result_homogeneous = np.dot(matrix, point_homegeneous)

    result = (result_homogeneous[0]/result_homogeneous[2], result_homogeneous[1]/result_homogeneous[2])
    return result
