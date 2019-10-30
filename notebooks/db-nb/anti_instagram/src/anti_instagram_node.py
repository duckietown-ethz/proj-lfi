#!/usr/bin/env python

import rospy
import threading

from anti_instagram.AntiInstagram import AntiInstagram
from cv_bridge import CvBridge
from duckietown_utils.jpg import bgr_from_jpg
from sensor_msgs.msg import CompressedImage

class AntiInstagramNode():
    def __init__(self):
        self.node_name = rospy.get_name()
        self.image_lock = threading.Lock()

        self.ai = AntiInstagram()

        # XXX: read parameters
        # XXX: parameters need to go inside config file, ask aleks about heirarchy 
        self.interval = self.setup_parameter("~ai_interval", 10)
        self.color_balance_percentage = self.setup_parameter("~cb_percentage", 0.8) # XXX: change in all launch files
        self.output_scale = self.setup_parameter("~scale_percent", 0.4) # XXX: change in all launch files 
        self.calculation_scale = self.setup_parameter("~resize", 0.2)

        self.bridge = CvBridge()

        self.image = None

        rospy.Timer(rospy.Duration(self.interval), self.calculate_new_parameters)
        
        self.uncorrected_image_subscriber = rospy.Subscriber(
                                                '~uncorrected_image/compressed', 
                                                CompressedImage, 
                                                self.process_image,
                                                buff_size=921600, 
                                                queue_size=1)

        self.corrected_image_publisher = rospy.Publisher(
                                             "~corrected_image/compressed", 
                                             CompressedImage, 
                                             queue_size=1)
        

    def process_image(self, image_msg):        
        try:
            self.image_lock.acquire()
            image = bgr_from_jpg(image_msg.data)
            self.image = image
            self.image_lock.release()
        except ValueError as e:
            rospy.loginfo('Anti_instagram cannot decode image: %s' % e)
            self.image_lock.release()
            return

        color_balanced_image = self.ai.apply_color_balance(image, 
                                   self.output_scale)

        if color_balanced_image is None:
            self.calculate_new_parameters(None)
            return

        corrected_image = self.bridge.cv2_to_compressed_imgmsg(
                              color_balanced_image)
        corrected_image.header.stamp = image_msg.header.stamp           
        self.corrected_image_publisher.publish(corrected_image)        
        

    def calculate_new_parameters(self, event):
        self.image_lock.acquire()
        image = self.image
        self.image_lock.release()

        if image is None:
            rospy.loginfo("[%s] Waiting for first image!" % self.node_name)
            return

        self.ai.calculate_color_balance_thresholds(image, 
            self.calculation_scale,
            self.color_balance_percentage)
        
        rospy.loginfo("[%s] New parameters computed" % self.node_name)
        
        


    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

if __name__ == '__main__':
    rospy.init_node('anti_instagram_node', anonymous=False)
    node = AntiInstagramNode()
    rospy.spin()

