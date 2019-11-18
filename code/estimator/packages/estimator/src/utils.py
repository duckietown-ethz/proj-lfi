import rospy
from duckietown_utils.jpg import bgr_from_jpg
from cv_bridge import CvBridge, CvBridgeError


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
