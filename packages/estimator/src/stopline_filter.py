import cv2
import numpy as np

import utils


class StoplineFilter():
    '''
    Possible policies: weighted_avg, max_quality
    '''

    def __init__(self, min_quality=0.5, policy='weighted_avg'):
        self.policy = policy
        self.min_quality = min_quality

        self.current_estimate = None
        self.current_estimate_quality = 0.0

    def update_stopline_poses(self, pose_estimates, estimate_qualities):
        if self.policy == 'max_quality':
            idx_max = np.argmax(estimate_qualities)
            self.current_estimate = pose_estimates[idx_max]
            self.current_estimate_quality = estimate_qualities[idx_max]
        elif self.policy == 'weighted_avg':
            # Check if there is at least one estimate of sufficient quality
            if max(estimate_qualities) > self.min_quality:
                # Filter only estimates with high enough quality
                positions = [utils.pos_to_tuple(p.position) for p, qual in zip(pose_estimates, estimate_qualities) if qual > self.min_quality]
                orientations = [p.orientation for p, qual in zip(pose_estimates, estimate_qualities) if qual > self.min_quality]
                qualities = list([qual for qual in estimate_qualities if qual > self.min_quality])

                # Generate weights (that sum up to 1)
                sum_qualities = sum(qualities)
                weights = list([qual / sum_qualities for qual in qualities])

                avg_position = np.mean(np.array(positions), axis=0)
                idx_max = np.argmax(estimate_qualities)
                #avg_orientation = utils.quat_to_tuple(pose_estimates[idx_max].orientation)
                avg_orientation = utils.average_quaternion(orientations, weights)
                self.current_estimate = utils.pose(avg_position, avg_orientation)
                self.current_estimate_quality = np.sum(np.array(qualities)*np.array(weights))
            else:
                self.current_estimate = None # Let integration fill the gaps!
                self.current_estimate_quality = 0.0 # Let integration fill the gaps!

    def get_best_estimate(self):
        return self.current_estimate

    def get_estimate_quality(self):
        return self.current_estimate_quality
