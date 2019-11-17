import rospy
from cv_bridge import CvBridge, CvBridgeError

def read_image(bridge, image_msg):
    try:
        image = bridge.compressed_imgmsg_to_cv2(image_msg, desired_encoding = "bgr8")
        return image
    except CvBridgeError as e:
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
