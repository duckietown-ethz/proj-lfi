import cv2
import numpy as np
from matplotlib import pyplot as plt
from scipy.ndimage.filters import gaussian_filter1d, gaussian_filter


# Load the image
image1 = cv2.imread('intersectionStopLine.jpg')
scale_percent = 40 # percent of original size
width = int(image1.shape[1] * scale_percent / 100)
height = int(image1.shape[0] * scale_percent / 100)
dim = (width, height)
# resize image
resized = cv2.resize(image1, dim, interpolation = cv2.INTER_AREA)
print('Original Dimensions : ',image1.shape)
print('Resized Dimensions : ',resized.shape)

# Convert image to gray scale
gray = cv2.cvtColor(resized, cv2.COLOR_RGB2GRAY)

cv2.imshow('dst',gray)
if cv2.waitKey(0) & 0xff == 27:
    cv2.destroyAllWindows()

# smoothing
gray = cv2.GaussianBlur(gray,(15, 15),0)

fast = cv2.FastFeatureDetector_create()

# Detect keypoints with non max suppression

keypoints_with_nonmax = fast.detect(gray, None)

image_with_nonmax = np.copy(resized)

# Draw keypoints on top of the input image
cv2.drawKeypoints(gray, keypoints_with_nonmax, image_with_nonmax, color=(0,255,0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Print the number of keypoints detected in the training image
print("Number of Keypoints Detected In The Image With Non Max Suppression: ", len(keypoints_with_nonmax))

cv2.imshow('dst',image_with_nonmax)
if cv2.waitKey(0) & 0xff == 27:
    cv2.destroyAllWindows()