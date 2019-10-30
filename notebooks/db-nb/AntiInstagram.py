#!/usr/bin/env python

import cv2
import numpy as np
import math
import threading

class AntiInstagram():

    def __init__(self):

        self.threshold_lock = threading.Lock()

        self.lower_threshold = None
        self.higher_threshold = [255,255,255]

    def calculate_color_balance_thresholds(self, image, scale, percentage):

        resized_image = cv2.resize(image, (0, 0), fx=scale, fy=scale)
        H = resized_image.shape[0]
        cropped_image = resized_image[int(H * 0.3):(H - 1), :, :]

        half_percent = percentage / 2
        channels = cv2.split(cropped_image)

        lower_threshold = []
        higher_threshold = []
        for idx, channel in enumerate(channels):
            # find the low and high precentile values (based on the input percentile)
            height, width = channel.shape
            num_pixels = width * height
            flattened = channel.reshape(num_pixels)

            # sort entries
            flattened = np.sort(flattened)
            
            # calculate thresholds
            lower_threshold.append(
                flattened[int(math.floor(num_pixels * half_percent))]) 
            
            higher_threshold.append(
                flattened[int(math.ceil(num_pixels * (1.0 - half_percent)))])

        self.threshold_lock.acquire()
        self.lower_threshold = lower_threshold
        # Note: higher_threshold is not being changed
        self.threshold_lock.release()

    def apply_color_balance(self, image, scale):

        self.threshold_lock.acquire()
        lower_threshold = self.lower_threshold
        higher_threshold = self.higher_threshold
        self.threshold_lock.release()

        if lower_threshold == None:
            return None

        resized_image = cv2.resize(image, (0, 0), fx=scale, fy=scale)

        channels = cv2.split(resized_image)
        out_channels = []

        for idx, channel in enumerate(channels):
            thresholded = self.apply_threshold(channel, lower_threshold[idx], higher_threshold[idx])
            normalized = cv2.normalize(thresholded, thresholded.copy(), 0, 255, cv2.NORM_MINMAX)
            out_channels.append(normalized)

        return cv2.merge(out_channels)
    
    def apply_threshold(self, matrix, low_value, high_value):
    
        low_mask = matrix < low_value
        matrix = self.apply_mask(matrix, low_mask, low_value)

        high_mask = matrix > high_value
        matrix = self.apply_mask(matrix, high_mask, high_value)

        return matrix

    def apply_mask(self, matrix, mask, fill_value):
        
        masked = np.ma.array(matrix, mask=mask, fill_value=fill_value)
        return masked.filled()

