#!/usr/bin/env python

import cv2
import numpy as np
from numpy import linalg as LA


def camera_img_to_birdseye(cam_img, homography):
    height, width, _ = cam_img.shape
    px_per_m_original = 500.0

    H = homography

    # Scale homography to the size of the camera image, as it assumes a size of (480,640)
    scale_x = 480.0 / height
    scale_y = 640.0 / width
    px_per_m_x = px_per_m_original / scale_x
    px_per_m_y = px_per_m_original / scale_y
    scaler_mat = np.hstack([np.ones((3, 1)) * scale_x, np.ones((3, 1)) * scale_y, np.ones((3, 1))])
    H = np.multiply(scaler_mat, H)

    # Scale axle coordinates to pixels
    scaler_mat = np.vstack([np.ones((1, 3)) * px_per_m_x, np.ones((1, 3)) * px_per_m_y, np.ones((1, 3))])
    H = np.multiply(scaler_mat, H)

    # Size (in pixels) of the resulting transformed image
    size = (int(height / 2), int(width / 2))
    print(size)

    # Center the axle x-axis in the image
    translation = np.eye(3, 3)
    translation[0, 2] += 0.0
    translation[1, 2] += size[1] / 2
    H = translation.dot(H)
    H /= H[2, 2]

    # Apply image transformation
    birdseye_img = cv2.warpPerspective(cam_img, H, size)

    # Rotate and transpose for correct orientation after transformation
    (h, w) = birdseye_img.shape[:2]
    center = (w / 2, h / 2)
    # TODO Convert this into a 3x3 matrix and combine with H
    rotMat = cv2.getRotationMatrix2D(center, 180, 1.0)
    birdseye_img = cv2.warpAffine(birdseye_img, rotMat, (w, h))
    birdseye_img = cv2.transpose(birdseye_img)

    cv2.imshow('birdseye_img', birdseye_img)
    if cv2.waitKey(0) & 0xff == 27:
        cv2.destroyAllWindows()

    return birdseye_img



def resize_crop(image1):

    # crop image
    cropped_image = image1[235:image1.shape[0]-33, 1:image1.shape[1]-615]

    # resize image
    resized = cv2.resize(cropped_image, (640, 480), interpolation=cv2.INTER_AREA)
    print('Original Dimensions : ', cropped_image.shape)
    print('Resized Dimensions : ', resized.shape)

    # safe cropped images
    cv2.imwrite('stopline-poses/pose' + str(1) + '_cropped.png', resized)


def color_red(image1):
    # It converts the BGR color space of image to HSV color space
    hsv = cv2.cvtColor(image1, cv2.COLOR_BGR2HSV)

    # Threshold of blue in HSV space
    hsv_red1 = np.array([0, 140, 100])
    hsv_red2 = np.array([15, 255, 255])
    hsv_red3 = np.array([165, 140, 100])
    hsv_red4 = np.array([180, 255, 255])

    # preparing the mask to overlay
    bw1 = cv2.inRange(hsv, hsv_red1, hsv_red2)
    bw2 = cv2.inRange(hsv, hsv_red3, hsv_red4)
    mask = cv2.bitwise_or(bw1, bw2)

    return mask


def corner_detector(image1):
    # smoothing
    image1 = cv2.GaussianBlur(image1, (15, 15), 0)

    # Detect keypoints with non max suppression
    fast = cv2.FastFeatureDetector_create(threshold=13)
    keypoints = fast.detect(image1, None)
    #print("Threshold: ", fast.getThreshold())

    # Print the number of keypoints detected in the training image
    print("Number of Keypoints Detected In The Image With Non Max Suppression: ", len(keypoints))
    #print(keypoints[11].pt)
    #print(keypoints[11].pt[0])

    # delete keypoint in the upper 60% of image
    for i in range(len(keypoints)-1, -1, -1):
        if keypoints[i].pt[1] < 480*0.6:
            del keypoints[i]

    print("Number of Keypoints Detected In The Image Without upper image part: ", len(keypoints))

    '''
    # Draw keypoints on top of the input image
    img_pk = cv2.drawKeypoints(image1, keypoints, outImage=np.array([]), color=(0, 0, 255))

    cv2.imshow('dst', img_pk)
    if cv2.waitKey(0) & 0xff == 27:
        cv2.destroyAllWindows()
    '''

    return keypoints


def undistorted_image(image1, camera_matrix, distortion_coefficients, rectification_matrix, projection_matrix):
    imageHeight, imageWidth, channels = image1.shape

    # Used for rectification
    mapx = np.ndarray(shape=(imageHeight, imageWidth, 1), dtype='float32')
    mapy = np.ndarray(shape=(imageHeight, imageWidth, 1), dtype='float32')

    mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, distortion_coefficients, rectification_matrix,
                                             projection_matrix, (imageWidth, imageHeight), cv2.CV_32FC1)

    undistortedImage = cv2.remap(image1, mapx, mapy, cv2.INTER_CUBIC)

    return undistortedImage


def pixel2ground(keypoints_pixel_img, homography):
    # TODO ...
    keypoints_cam = np.zeros((len(keypoints_pixel_img),2))

    for i in range(len(keypoints_pixel_img)):
        u = keypoints_pixel_img[i].pt[0]
        v = keypoints_pixel_img[i].pt[1]
        #print([keypoints_pixel_img[i].pt[0], keypoints_pixel_img[i].pt[1]])
        temp = np.matmul(homography, np.array([u,v,1]))
        keypoints_cam[i,0] = temp[0] / float(temp[2])
        keypoints_cam[i,1] = - temp[1] / float(temp[2])
        #print(round(keypoints_cam[i,0], 3), round(keypoints_cam[i,1], 3))

    return keypoints_cam


def delete_duplicates(keypoints):
    no_duplicates = np.ones((len(keypoints),), dtype=int)
    # find duplicates
    for i in range(len(keypoints)):
        for j in range(i+1, len(keypoints)):
            if abs(keypoints[i, 0] - keypoints[j, 0]) < 0.01 and abs(keypoints[i, 1] - keypoints[j, 1]) < 0.01:
                no_duplicates[j] = 0

    # find points that are too far away
    for i in range(len(keypoints)):
        if np.sqrt(keypoints[i,0]**2 + keypoints[i,1]**2) > 0.35:
            no_duplicates[i] = 0

    #print(no_duplicates)

    # append only non-duplicate values
    keypoints_single = np.empty((0,2))
    for i in range(0, len(keypoints)):
        if no_duplicates[i] == 1:
            keypoints_single = np.append(keypoints_single, keypoints[i, :])

    keypoints_single = np.reshape(keypoints_single, (-1,2))
    print("Number of Keypoints Detected In The Image after deleting duplicates: ", len(keypoints_single))
    #print(keypoints_single)

    return keypoints_single


def find_distances(keypoints):
    if len(keypoints) == 1 or len(keypoints) == 0:
        print("WARNING: ONLY ONE KEYPOINT DETECTED --> CAN'T CALCULATE THE ANGLE & DISTANCES!")
        return [], [], []

    # find the two "front" points of the stopline
    dist_abs = np.zeros(len(keypoints))
    dist_rel = np.zeros(len(keypoints))
    for i in range(1,len(keypoints)):
        dist_abs[i] = LA.norm(keypoints[0,:]-keypoints[i,:])
        dist_rel[i] = abs(dist_abs[i]-0.205)
    dist_rel[0] = 0.205
    #print(dist_abs)

    point2 = np.min(dist_rel)
    if point2 > 0.05:
        print("WARNING: UNABLE TO DETECT 2 FRONT POINTS --> CAN'T CALCULATE THE ANGLE & DISTANCES!")
        return [], [], []

    idx_pt2 = np.argmin(dist_rel)

    angle = np.arctan((keypoints[0,0]-keypoints[idx_pt2,0])/(keypoints[0,1]-keypoints[idx_pt2,1]))

    keypoints_tf = np.zeros((2, 2))
    keypoints_tf[0, 0] = np.cos(angle)*keypoints[0, 0] - np.sin(angle)*keypoints[0, 1]
    keypoints_tf[0, 1] = np.sin(angle)*keypoints[0, 0] + np.cos(angle)*keypoints[0, 1]
    keypoints_tf[1, 0] = np.cos(angle)*keypoints[idx_pt2, 0] - np.sin(angle)*keypoints[idx_pt2, 1]
    keypoints_tf[1, 1] = np.sin(angle)*keypoints[idx_pt2, 0] + np.cos(angle)*keypoints[idx_pt2, 1]

    long_dist = (keypoints_tf[0, 0] + keypoints_tf[1, 0]) / 2 - 0.05
    if keypoints_tf[0, 1] > keypoints_tf[1, 1]:
        lat_dist = keypoints_tf[0, 1]
    else:
        lat_dist = keypoints_tf[1, 1]

    print("angle: " + str(90-angle*180/np.pi))
    print("longitudinal distance to stopline: " + str(long_dist))
    print("lateral distance to white street line: " + str(lat_dist))

    return angle, long_dist, lat_dist

def show_undistorted_img_keypoints(image, keypoints):
    # Draw keypoints on top of the input image
    img_pk = cv2.drawKeypoints(image, keypoints, outImage=np.array([]), color=(0, 0, 255))

    cv2.imshow('undistortedMaskedKeypointImage', img_pk)
    if cv2.waitKey(0) & 0xff == 27:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    print('Starting stop line poses ...')

    # Load the images
    '''
    image1 = cv2.imread('stopline-poses/screenshots/Screenshot from 2019-11-22 14-17-12_pose1.png')
    image2 = cv2.imread('stopline-poses/screenshots/Screenshot from 2019-11-22 14-19-46_pose2.png')
    image3 = cv2.imread('stopline-poses/screenshots/Screenshot from 2019-11-22 14-21-21_pose3.png')
    image4 = cv2.imread('stopline-poses/screenshots/Screenshot from 2019-11-22 14-21-48_pose4.png')
    image5 = cv2.imread('stopline-poses/screenshots/Screenshot from 2019-11-22 14-22-49_pose5.png')
    image6 = cv2.imread('stopline-poses/screenshots/Screenshot from 2019-11-22 14-23-16_pose6.png')
    image7 = cv2.imread('stopline-poses/screenshots/Screenshot from 2019-11-22 14-24-37_pose7.png')
    image8 = cv2.imread('stopline-poses/screenshots/Screenshot from 2019-11-22 14-25-00_pose8.png')
    image9 = cv2.imread('stopline-poses/screenshots/Screenshot from 2019-11-22 14-27-06_pose9.png')
    image10 = cv2.imread('stopline-poses/screenshots/Screenshot from 2019-11-22 14-27-42_pose10.png')
    image11 = cv2.imread('stopline-poses/screenshots/Screenshot from 2019-11-22 14-28-56_pose11.png')
    image12 = cv2.imread('stopline-poses/screenshots/Screenshot from 2019-11-22 14-29-20_pose12.png')
    image13 = cv2.imread('stopline-poses/screenshots/Screenshot from 2019-11-22 14-30-26_pose13.png')
    '''

    # camera intrinsic and extrinsic parameters
    camera_matrix = np.reshape(np.array([383.56872831063896, 0.0, 306.29202372658284, 0.0, 375.2944318113526, 277.4882151139124, 0.0, 0.0, 1.0]), (3,3))
    distortion_coefficients = np.reshape(np.array([-0.3969146750560185, 0.13898585541278483, -0.02400133922589981, 0.007263687357386502, 0.0]), (1,5))
    projection_matrix = np.reshape(np.array([294.7282409667969, 0.0, 314.18058144816314, 0.0, 0.0, 308.80792236328125, 271.973485013772, 0.0, 0.0, 0.0, 1.0, 0.0]), (3,4))
    rectification_matrix = np.reshape(np.array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]), (3,3))
    homography = np.reshape(np.array([-1.1907434447195475e-05, -0.00016985225547642657, -0.18018639992319468,
                                      0.0008110438997760144, 2.9640247271729815e-07, -0.2609339693203626,
                                      -5.837794811070778e-05, -0.006471722102967347, 1.0]), (3,3))

    # Initialize the node

    # choose input image: pose1-pose13 possible
    image = cv2.imread('stopline-poses/pose9_cropped.png')

    image = undistorted_image(image, camera_matrix, distortion_coefficients, rectification_matrix, projection_matrix)

    mask = color_red(image)

    keypoints_pixel_img = corner_detector(mask)

    keypoints_ground_world = pixel2ground(keypoints_pixel_img, homography)

    keypoints_ground_world_single = delete_duplicates(keypoints_ground_world)

    phi, lat_dist, long_dist = find_distances(keypoints_ground_world_single)

    show_undistorted_img_keypoints(mask, keypoints_pixel_img)

    #camera_img_to_birdseye(image, homography)

    print('... finishing stop line poses')
