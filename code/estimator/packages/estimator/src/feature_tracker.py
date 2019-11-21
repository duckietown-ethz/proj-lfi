import cv2
import numpy as np

class FeatureTracker():
    # TODO Try out different matchers
    # TODO Try out matching based on predicted positions (based on motor commands) instead of descriptors, or a combination of position- and descriptor-based matching.
    # TODO Try out combining the off-the-shelf feature-detectors with custom ones (maybe separate between position and orientation feature detectors).

    def __init__(self, keypoint_count, matches_to_consider, log):
        self.detector = cv2.ORB_create(keypoint_count)
        self.matcher = cv2.BFMatcher_create(cv2.NORM_HAMMING, crossCheck=True)

        self.matches_to_consider = matches_to_consider
        self.log = log

        self.reset()


    def reset(self):
        self.last_descriptors = None
        self.last_keypoints = None

        self.cumulative_position = np.matrix([0.0, 0.0]).T
        self.cumulative_angle = 0.0

    def draw_match_lines(self, image, matches):
        for kp_pair in matches:
            kp1 = kp_pair[0]
            kp2 = kp_pair[1]
            pt1 = (int(kp1.pt[0]), int(kp1.pt[1]))
            pt2 = (int(kp2.pt[0]), int(kp2.pt[1]))
            cv2.line(image, pt1, pt2, (255,255,255), 1)
            cv2.circle(image, pt1, 1, (0,0,255), -1)

    def process_image(self, image, image_out_keypoints=None):
        # TODO Apply mask for every black pixel
        keypoints, descriptors = self.detector.detectAndCompute(image, None)

        if self.last_descriptors != None and self.last_keypoints != None:
            matches = self.matcher.match(descriptors, self.last_descriptors)
            matches = sorted(matches, key = lambda x:x.distance)

            matches_to_consider = matches[:self.matches_to_consider]

            idx = [m.queryIdx for m in matches_to_consider]
            last_idx = [m.trainIdx for m in matches_to_consider]

            keypoints_to_consider = np.array(keypoints)[np.array(idx)]
            keypoints_to_consider_last = np.array(self.last_keypoints)[np.array(last_idx)]

            positions = np.array([[k.pt[0], k.pt[1]] for k in keypoints_to_consider])
            positions_last = np.array([[k.pt[0], k.pt[1]] for k in keypoints_to_consider_last])

            position_diff = positions - positions_last
            diff_lengths = np.linalg.norm(position_diff, axis=1)

            # Filter impossible values, i.e. zero difference, since that would mean we didn't move at all
            position_diff = position_diff[diff_lengths > 0.001]
            if position_diff.shape[0] > 0:
                avg_position_diff = np.mean(position_diff, 0)
                self.cumulative_position += np.matrix(avg_position_diff).T

            angles = np.array([k.angle for k in keypoints])[np.array(idx)]
            angles_last = np.array([k.angle for k in self.last_keypoints])[np.array(last_idx)]

            angle_diff = angles - angles_last
            diff_abs = np.abs(angle_diff)
            # Filter impossible values, i.e.:
            # -> zero difference, since that would mean we didn't move at all
            # -> angle larger than 45 degree
            angle_diff = angle_diff[np.logical_and(diff_abs > 0.001, diff_abs < 45.0)]
            if angle_diff.shape[0] > 0:
                avg_angle_diff = np.mean(angle_diff, 0)
                self.cumulative_angle += avg_angle_diff

            p = [np.asscalar(self.cumulative_position[1]), np.asscalar(self.cumulative_position[0])]

            if image_out_keypoints != None:
                # TODO Visualize by drawing lines from last position to current position
                # TODO Add flag cv2.DRAW_MATCHES_FLAGS_DRAW_OVER_OUTIMG
                #cv2.drawKeypoints(image_out_keypoints, keypoints_to_consider, image_out_keypoints, (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                kp_pairs = np.vstack([keypoints_to_consider, keypoints_to_consider_last]).T
                self.draw_match_lines(image_out_keypoints, kp_pairs)

        self.last_keypoints = keypoints
        self.last_descriptors = descriptors

        return p, -self.cumulative_angle
