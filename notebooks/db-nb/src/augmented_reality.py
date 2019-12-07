#!/usr/bin/env python
import numpy as np
import cv2
from geometry_msgs.msg import Point

# works on absolute image pixel values
def draw_segment(image, pt0, pt1, color):
    defined_colors = {
        'red': ['rgb', [1, 0, 0]],
        'green': ['rgb', [0, 1, 0]],
        'blue': ['rgb', [0, 0, 1]],
        'yellow': ['rgb', [1, 1, 0]],
        'magenta': ['rgb', [1, 0, 1]],
        'cyan': ['rgb', [0, 1, 1]],
        'white': ['rgb', [1, 1, 1]],
        'black': ['rgb', [0, 0, 0]]}
    _color_type, [r, g, b] = defined_colors[color]
    cv2.line(image, pt0, pt1, (b * 255, g * 255, r * 255), 5)
    return image

class Augmenter:

    def __init__(self, map_features, H, ci):
        self.Homo = H

        # build rectification map using pinhole camera model calibration
        self.W = ci.width
        self.H = ci.height
        mapx = np.ndarray(shape=(self.H, self.W, 1), dtype='float32')
        mapy = np.ndarray(shape=(self.H, self.W, 1), dtype='float32')
        mapx, mapy = cv2.initUndistortRectifyMap(ci.K, ci.D, ci.R, ci.P, 
                             (self.W, self.H), cv2.CV_32FC1, mapx, mapy)
        self.mapx = mapx
        self.mapy = mapy

        # define coordinates for point names, various reference frames
        pts = map_features.values()[0]
        # convert all points to rectified pixel coordinates
        self.pxls = self.any_to_pixel(pts)

        # define segments to be overlaid (using point names)
        self.sgs = map_features.values()[1]

    def any_to_pixel(self, pts):
        pxls = dict()
        # loop over keys (point names)
        for k in pts:
            pt = pts[k]
            ref_frame = pt[0]
            coords = pt[1]
            if ref_frame == 'image01':
                u = (self.W-1)*coords[0]
                v = (self.H-1)*coords[1]
                pxls[k] = (int(u), int(v))
            elif ref_frame == 'axle':
                pxls[k] = self.ground2pixel(coords)
            else:
                print("Unhandled reference frame for point ",k)
                pxls[k] = None
        return pxls

    # takes RECTIFIED image and overlays previosly defined map features
    def render_segments(self, img):
        for seg in self.sgs:
            pt0 = self.pxls[seg['points'][0]]
            pt1 = self.pxls[seg['points'][1]]
            col = seg['color']
            img = draw_segment(img, pt0, pt1, col)

        return img

    # rectifies image using pinhole camera model
    def process_image(self, cv_image_raw):
        cv_image_rectified = np.zeros(np.shape(cv_image_raw))
        return cv2.remap(cv_image_raw, self.mapx, self.mapy, cv2.INTER_CUBIC, cv_image_rectified)

    # transform points from axle frame of reference to rectified pixels
    def ground2pixel(self, point):
        gp = np.array([point[0], point[1], 1.0])
        pix = np.linalg.solve(self.Homo, gp)
        pix = pix / pix[2]
        # output a tuple cause that's what openCV likes
        return (int(pix[0]), int(pix[1]))

    # transform points from camera frame of reference to rectified pixels
    def camera2pixel(self, point):
        # YAY: camera frame is mentioned in the exercise but not actually needed
        return None
