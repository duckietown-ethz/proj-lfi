import cv2
import rospy
import numpy as np
from tf import TransformerROS
from geometry_msgs.msg import PoseStamped, TransformStamped

class Intersection4wayModel():

    def __init__(self, pcm, tf):
        self.pcm = pcm
        self.tf = tf

        # Origin of ther intersection coordinate system is the center of stopline 0.
        tile_side_length = 0.61
        lane_width = 0.205
        stopline_thickness = 0.048
        center_markings_thickness = 0.025

        q = lane_width
        p = stopline_thickness
        s = center_markings_thickness

        self.stopline_centers = []
        self.stopline_centers.append(self.get_pose('intersection', [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]))
        self.stopline_centers.append(self.get_pose('intersection', [-(q/2.0+s+q+p/2.0), p/2.0+q/2.0, 0.0], [0.0, 0.0, 0.0, 1.0]))
        self.stopline_centers.append(self.get_pose('intersection', [-(q/2.0+s+q/2.0), p/2.0+2*q+s+p/2.0, 0.0], [0.0, 0.0, 0.0, 1.0]))
        self.stopline_centers.append(self.get_pose('intersection', [q/2.0+p/2.0, p/2.0+q+s+q/2.0, 0.0], [0.0, 0.0, 0.0, 1.0]))
        print(self.stopline_centers)


    def get_stopline_centers_pixel_prediction(self):
        self.stopline_centers_predicted = []

        self.tf.waitForTransform('camera', 'intersection', rospy.Time(0), rospy.Duration(10))

        for i in range(4):
            prediction_camera_frame = self.tf.transformPose('camera', self.stopline_centers[i])
            # TODO Filter points behind the camera
            position = prediction_camera_frame.pose.position
            prediction_pixel_rectified = self.pcm.project3dToPixel((position.x, position.y, position.z))
            prediction_pixel_unrectified = self.unrectifyPoint(prediction_pixel_rectified)

            rounded = (int(prediction_pixel_unrectified[0]), int(prediction_pixel_unrectified[1]))
            self.stopline_centers_predicted.append(rounded)

        return self.stopline_centers_predicted


    # Translated to python from https://docs.ros.org/api/image_geometry/html/c++/pinhole__camera__model_8cpp_source.html
    def unrectifyPoint(self, point_rectified):
        ray = self.pcm.projectPixelTo3dRay(point_rectified)
        # Project the ray on the image
        r_vec, _ = cv2.Rodrigues(self.pcm.R)
        t_vec = np.zeros((3,1))
        image_points, _ = cv2.projectPoints(np.array([ray]), r_vec, t_vec, self.pcm.K, self.pcm.D)
        point_unrectified = image_points[0][0]

        return (point_unrectified[0], point_unrectified[1]);


    def get_pose(self, frame, position, orientation):
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


    def get_transform(self, frame_parent, frame_child, position, orientation):
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
