#!/usr/bin/env python

import rospy
import threading

from duckietown import DTROS
from anti_instagram.AntiInstagram import AntiInstagram
from cv_bridge import CvBridge
from duckietown_utils.jpg import bgr_from_jpg
from sensor_msgs.msg import CompressedImage

class AntiInstagramNode(DTROS):
    def __init__(self, node_name):
        super(AntiInstagramNode, self).__init__(node_name=node_name, parameters_update_period=10.0)
        self.veh_name = rospy.get_namespace().strip("/")

        # Initialize parameters
        self.parameters['~ai_interval'] = None
        self.parameters['~cb_percentage'] = None
        self.parameters['~scale_percent'] = None
        self.parameters['~resize'] = None
        self.updateParameters()
        self.refresh_parameters()

        self.image = None
        self.image_lock = threading.Lock()

        self.ai = AntiInstagram()
        self.bridge = CvBridge()

        self.uncorrected_image_subscriber = self.subscriber(
                                                '~uncorrected_image/compressed',
                                                CompressedImage,
                                                self.process_image,
                                                buff_size=921600,
                                                queue_size=1)

        self.corrected_image_publisher = self.publisher(
                                             "~corrected_image/compressed",
                                             CompressedImage,
                                             queue_size=1)

        rospy.Timer(rospy.Duration(self.interval), self.calculate_new_parameters)

    def refresh_parameters(self):
        self.interval                   = self.parameters["~ai_interval"]
        self.color_balance_percentage   = self.parameters["~cb_percentage"]
        self.output_scale               = self.parameters["~resize"]
        self.calculation_scale          = self.parameters["~scale_percent"]

    def process_image(self, image_msg):
        if self.parametersChanged:
            self.log('Parameters changed.', 'info')
            self.refresh_parameters()
            self.parametersChanged = False

        try:
            self.image_lock.acquire()
            image = bgr_from_jpg(image_msg.data)
            self.image = image
            self.image_lock.release()
        except ValueError as e:
            self.log('Anti_instagram cannot decode image: %s' % e)
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


    def calculate_new_parameters(self, event=None):
        self.image_lock.acquire()
        image = self.image
        self.image_lock.release()

        if image is None:
            self.log("[%s] Waiting for first image!" % self.node_name)
            return

        self.ai.calculate_color_balance_thresholds(image,
            self.calculation_scale,
            self.color_balance_percentage)

        self.log("[%s] New parameters computed" % self.node_name)

        def onShutdown(self):
            self.log("Stopping preprocessor_node.")

            super(PreprocessorNode, self).onShutdown()

if __name__ == '__main__':
    node = AntiInstagramNode(node_name='anti_instagram_node')
    rospy.spin()
