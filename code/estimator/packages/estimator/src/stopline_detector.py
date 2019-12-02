import cv2
import rospy
import numpy as np

from tf.transformations import quaternion_from_euler, unit_vector, quaternion_multiply
from geometry_msgs.msg import PoseStamped, TransformStamped
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
from sklearn.neighbors import KNeighborsClassifier

import utils


class StoplineDetector():
    '''
    This class assumes images to be in birdseye perspective.
    '''

    def __init__(self, scaled_homography):
        self.scaled_homography = scaled_homography


    def filter_red(self, image, verbose=False, tk=None):
        # Convert to HSV space for easier processing
        img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Filter red color ranges
        # Taken from https://github.com/duckietown/dt-core/blob/5576fc097e4c88421735d7ef5c6fb83194c51fac/packages/line_detector/include/line_detector/line_detector1.py#L43
        # TODO Make it configurable as well
        mask1 = cv2.inRange(img_hsv, np.array([0,140,100]), np.array([15,255,255]))
        mask2 = cv2.inRange(img_hsv, np.array([165,140,100]), np.array([180,255,255]))
        mask_red = cv2.bitwise_or(mask1, mask2)
        tk.completed('filter_red->inRange')

        # TODO Test if this would improve performance
        #kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        #bw = cv2.dilate(bw, kernel)

        img_red_highlight = None
        if verbose:
            # Highlight red pixels for visualization
            img_hsv[:,:,2][mask_red == 255] = 255
            img_hsv[:,:,2][mask_red == 0] /= 2
            img_red_highlight = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)
            tk.completed('filter_red->verbose')

        return mask_red, img_red_highlight


    def cluster_mask(self, mask, dbscan_eps, dbscan_min_samples, tk=None):
        # Get all coordinates where mask is set
        x, y = np.where(mask == 255)
        points = np.vstack([x,y]).T

        clusters = []

        if points.shape[0] > 0:
            # Normalize points so the eps parameter doesn't depend on the magnitude of the data
            points_normalized = StandardScaler().fit_transform(points.astype(float))
            tk.completed('clustering->normalization')

            # Clustering
            # TODO Can this be run in parallel (multi-core) somehow?
            db_scan = DBSCAN(eps=dbscan_eps, min_samples=dbscan_min_samples)
            db_scan_result = db_scan.fit(points_normalized)

            labels = db_scan_result.labels_
            unique_labels = set(labels)
            tk.completed('clustering->fitting')

            # Process results
            for label in unique_labels:
                # Do not include noise in the result
                if label == -1:
                    continue

                cluster_mask = (labels == label)
                cluster_points = points[cluster_mask]
                clusters.append(cluster_points)
            tk.completed('clustering->means')

        return clusters


    # Points must be in matrix coordinates
    # Returns poses in axle coordinate frame
    def calculate_pose_from_points(self, points):
        center_pixel = np.mean(points, axis=0)
        # The homography works with image coordinates and the points are supposed to be in matrix coordinates
        center_pixel = (center_pixel[1], center_pixel[0])

        # Position
        # Transform center point into axle coordinates
        center_pixel_rectified = utils.apply_homogeneous_transform(center_pixel, self.scaled_homography.for_image(), invert=True)
        center_point_axle = utils.apply_homogeneous_transform(center_pixel_rectified, self.scaled_homography.for_axle())
        # Position can be set directly
        position = [center_point_axle[0], center_point_axle[1], 0.0]

        # Orientation
        # Because we assume the points represent something that has its x-axis along its 'long' side,
        # we can fit a line to the points and interpret the direction of the line as the direction of the x-axis
        # vx and vy indicate the direction of the line (in pixel coordinates) going from x,y (or equivalently the center point)
        # TODO Test if it still works when using x,y instead of the center point
        # TODO Test if other distance metrics yield better performance with similar results (https://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html?highlight=cv2.fitline#cv2.fitLine )
        [vx, vy, x, y] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
        # The direction vector needs to be in image coordinates
        x_dir = (np.asscalar(vy), np.asscalar(vx))

        # Transform x direction point into axle coordinates
        # TODO Is the *10 here really improving results?
        x_dir_abs = (center_pixel[0] + x_dir[0]*10, center_pixel[1] + x_dir[1]*10)
        x_dir_abs_pixel_rectified = utils.apply_homogeneous_transform(x_dir_abs, self.scaled_homography.for_image(), invert=True)
        x_dir_abs_point_axle = utils.apply_homogeneous_transform(x_dir_abs_pixel_rectified, self.scaled_homography.for_axle())
        x_dir_point_axle = (x_dir_abs_point_axle[0] - center_point_axle[0], x_dir_abs_point_axle[1] - center_point_axle[1])

        # Angle needs to be calculated based on the x-axis direction vector
        x = np.array([x_dir_point_axle[0], x_dir_point_axle[1]])
        x = x / np.linalg.norm(x)
        if x[0] > 0:
            alpha = np.arcsin(x[1])
        else:
            alpha = np.pi - np.arcsin(x[1])
        # TODO Is unit_vector really necessary here?
        orientation = unit_vector(quaternion_from_euler(0, 0, alpha, axes='sxyz'))

        pose = utils.get_pose('axle', position, orientation).pose
        return pose


    # poses and poses_predicted need to be in the same coordinate frame
    # Classification is done with nearest positional neighbor
    def classify_poses(self, poses_measured, poses_reference):
        positions_measured = np.array([(p.position.x, p.position.y) for p in poses_measured])
        positions_reference = np.array([(p.position.x, p.position.y) for p in poses_reference])
        labels_known = np.arange(positions_reference.shape[0])

        knn = KNeighborsClassifier(1)
        knn.fit(positions_reference, labels_known)

        labels_predicted = knn.predict(positions_measured)
        return labels_predicted


    # poses and poses_predicted need to be in the same coordinate frame and they should be ordered such that the entries match
    def correct_orientation(self, pose, poses_reference):
        # There are two possible orientations based on the direction of the x-axis. The one that is closer to the prediction will be chosen.
        orientation1 = np.array(utils.quat_to_tuple(pose.orientation))
        # Alternative orientation is the current orientation rotated 180 degree around the z axis
        rot180 = [0.0, 0.0, 1.0, 0.0]
        orientation2 = quaternion_multiply(orientation1, rot180)
        orientation_reference = np.array(utils.quat_to_tuple(poses_reference.orientation))

        orientation1_error = utils.quat_distance(orientation1, orientation_reference)
        orientation2_error = utils.quat_distance(orientation2, orientation_reference)

        if orientation2_error < orientation1_error:
            pose.orientation = utils.tuple_to_quat(orientation2)

        return pose


    # Determines the quality of a point cluster taking into account the image resolution
    # Quality is given in a [0,1] interval (1 is best)
    def cluster_quality(self, cluster_points):
        # A stopline occupies at most 4356 pixels at resolution 480x640 (number was determined experimentally)
        max_percentage = 4356 / float(640 * 480)

        # Calculate percentage of image occupied by cluster
        px_cnt = self.scaled_homography.current_height * self.scaled_homography.current_width
        percentage = cluster_points.shape[0] / float(px_cnt)

        # Constrain to [0,1] since max_percentage was only determined emperically
        quality = percentage / float(max_percentage)

        return np.clip(quality, 0.0, 1.0)


    def draw_debug_image(self, image, stopline_poses_reference, stopline_poses_measured, pixel_clusters, labels):
        stopline_colors = {
            0: (255,0,0),
            1: (0,255,0),
            2: (0,0,255),
            3: (255,0,255)
        }

        # Color pixel clusters
        for cluster_points, label in zip(pixel_clusters, labels):
            color = stopline_colors[label]
            x = cluster_points[:,0]
            y = cluster_points[:,1]
            image[x, y] = color

        # Draw center points of measurements
        for pose in stopline_poses_measured:
            self.draw_axle_pose_in_image(image, pose, (255,255,255))

        # Draw center points of references
        for i, pose in enumerate(stopline_poses_reference):
            color = stopline_colors[i]
            self.draw_axle_pose_in_image(image, pose, color)

        return image


    # TODO Add arrow to indicate orientation
    def draw_axle_pose_in_image(self, image, pose, color):
        position = pose.position

        # Check if point is behind the camera
        if position.z < 0:
            return

        pt_camera_view = utils.apply_homogeneous_transform((position.x, position.y), self.scaled_homography.for_axle(), invert=True)
        pt_birdseye_view = utils.apply_homogeneous_transform(pt_camera_view, self.scaled_homography.for_image())

        pt = (int(pt_birdseye_view[0]), int(pt_birdseye_view[1]))
        cv2.circle(image, pt, 5, color, -1)
