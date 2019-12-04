#!/usr/bin/env python
from os import environ
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from duckietown_utils.yaml_wrap import yaml_load_file
from augmented_reality import Augmenter
from config_loaders import load_homography, load_camera_info
from duckietown import DTROS

class AugmentedRealityNode(DTROS):

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(AugmentedRealityNode, self).__init__(node_name=node_name)

        # Ideally I should use the param server instead
        self.veh_name = environ['VEHICLE_NAME']
        # self.veh_name = rospy.get_param("~veh_name", None)
        #self.node_name = rospy.get_param("~node_name", None)
        self.node_name = node_name

        #self.map_file = "./intersection_4way.yaml"
        # From argument to roslaunch.
        self.map_file = environ['PWD']+'/packages/augmented_reality/map/'+rospy.get_param("~map", None) + '.yaml'

        H = load_homography()
        cam_info = load_camera_info()

        map_features = yaml_load_file(self.map_file)

        self.augmenter = Augmenter(map_features, H, cam_info)

        input_topic  = "/%s/camera_node/image/compressed"%self.veh_name
        output_topic = "/%s/%s/image/compressed"%(self.veh_name, self.node_name)

        self.image_sub = rospy.Subscriber(input_topic, CompressedImage, self.image_cb)
        print("will publish to" + output_topic)
        self.image_pub = rospy.Publisher(output_topic, CompressedImage, queue_size=1)

        self.log("Initialized")

    def image_cb(self, packet):
        #  type(packet) == CompressedImage
        # decode
        np_arr = np.fromstring(packet.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # process
        image = self.augmenter.process_image(image)
        image = self.augmenter.render_segments(image)
        # encode
        packet.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        # send
        self.image_pub.publish(packet)

    def onShutdown(self):
        self.image_sub.unregister()
        super(AugmentedRealityNode, self).onShutdown()
        rospy.loginfo("[AugmentedRealityNode] Shutdown.")

if __name__ == '__main__':
    print("current path: "+environ['PWD'])
    print("Creating node: augmented_reality_node")
    # create the node
    node = AugmentedRealityNode(node_name='augmented_reality_node')
    # keep spinning
    rospy.spin()
