import cv2
import numpy as np

class ScaledHomography():
    def __init__(self, original_homography, original_height, original_width):
        self.original_homography = original_homography
        self.original_height = original_height
        self.original_width = original_width

        self.recalculate_homography(original_height, original_width)


    def for_image(self):
        return self.current_homography_image


    def for_image_inv(self):
        return self.current_homography_image_inverse


    def for_axle(self):
        return self.current_homography_axle


    def for_axle_inv(self):
        return self.current_homography_axle_inverse


    def scale_homography_for_axle(self, height, width):
        H = self.original_homography

        # Scale homography to the size of the camera image, as it assumes a size of (480,640)
        scale_x = self.original_height / float(height)
        scale_y = self.original_width / float(width)
        scaler_mat = np.hstack([np.ones((3,1))*scale_x, np.ones((3,1))*scale_y, np.ones((3,1))])
        H = np.multiply(scaler_mat, H)

        return H

    def scale_homography_for_image(self, height, width):
        H = self.scale_homography_for_axle(height, width)
        # TODO Remove code duplication. This is also done in scale_homography_for_axle()
        scale_x = self.original_height / float(height)
        scale_y = self.original_width / float(width)

        # Scale axle coordinates to pixels
        px_per_m_original = 600.0
        px_per_m_x = px_per_m_original / scale_x
        px_per_m_y = px_per_m_original / scale_y
        scaler_mat = np.vstack([np.ones((1,3))*px_per_m_x, np.ones((1,3))*px_per_m_y, np.ones((1,3))])
        H = np.multiply(scaler_mat, H)

        # Size (in pixels) of the resulting transformed image
        size = (height, width)

        # Center the axle x-axis in the image
        translation = np.eye(3, 3)
        translation[0, 2] += 0.0
        translation[1, 2] += size[1] / 2
        H = translation.dot(H)
        H /= H[2, 2]

        # Rotate and transpose for correct orientation after transformation
        # NOTE: These transformations are NOT necessary for calculations, but they orient the image so that it is easier interpreted by a human when visualizing the birdseye view.
        # Transpose
        row = np.copy(H[0,:])
        H[0,:] = H[1,:]
        H[1,:] = row
        # Rotate
        center = (width / 2, height / 2)
        rot_mat_affine = cv2.getRotationMatrix2D(center, 180, 1.0)
        rot_mat = np.vstack([rot_mat_affine, np.array([0.0, 0.0, 1.0])])
        H = rot_mat.dot(H)

        return H


    def update_homography(self, height, width):
        # TODO Test performance on duckiebot to determine if this check actually has an impact
        if height != self.current_height or width != self.current_width:
            self.recalculate_homography(height, width)

    def recalculate_homography(self, height, width):
        self.current_homography_image = self.scale_homography_for_image(height, width)
        # TODO Use linsolve where the inverse is used and don't provide the inverse in this class
        self.current_homography_image_inverse = np.linalg.inv(self.current_homography_image)

        self.current_homography_axle = self.scale_homography_for_axle(height, width)
        self.current_homography_axle_inverse = np.linalg.inv(self.current_homography_axle)

        self.current_height = height
        self.current_width = width


    def cb_camera_info(self, msg):
        self.update_homography(msg.height, msg.width)
