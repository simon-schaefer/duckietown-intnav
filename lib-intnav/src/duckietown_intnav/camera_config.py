#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Load and make availabe camera configuration data from configuration file 
# or camera configuration message string.  
# K - intrinsics. 
# H - homography. 
# R - rectification matrix. 
# P - projjection matrix. 
###############################################################################
__all__ = [
    'CameraConfig',
]

import cv2
from image_geometry import PinholeCameraModel
from numpy import asarray, reshape
import numpy as np
import os.path
import yaml

class CameraConfig: 

    def __init__(self, K, D, H, dim, P, R): 
        # Initialize camera properties. 
        self.K = reshape(asarray(K), (3,3))
        self.D = asarray(D)
        self.width = dim[0]
        self.height = dim[1]
        self.P = reshape(asarray(P), (3,4))               
        self.R = reshape(asarray(R), (3,3))
        self.H = None
        if not H is None: 
            self.H = reshape(asarray(H), (3,3))
        # Initialize camera model and undistortion model. 
        mapx = np.ndarray(shape=(self.height, self.width, 1), dtype='float32')
        mapy = np.ndarray(shape=(self.height, self.width, 1), dtype='float32')
        self._mapx, self._mapy = cv2.initUndistortRectifyMap(
            self.K, self.D, self.R, self.P, (self.width, self.height), 
            cv2.CV_32FC1, mapx, mapy)

    @classmethod
    def from_file(CameraConfig, file_path="../duckietown_intnav_tests/cam_config/camera.yaml"):
        ''' Load camera intrinsics and distortion parameter from yaml file, 
        which is stored in data directory by default. '''
        params = {}
        try: 
            config_file = os.path.dirname(os.path.realpath(__file__))
            config_file = os.path.join(config_file, file_path)
            with open(config_file, 'r') as stream:
                params = yaml.load(stream)
        except (IOError, yaml.YAMLError): 
            raise IOError("Unknown or invalid parameters file !")   
        # Assign parameters.
        K = params['camera_matrix']['data']
        dist_params = params['distortion_coefficients']
        img_width = params['image_width']
        img_height = params['image_height']
        projection_matrix = params['projection_matrix']['data']
        rect_matrix = params['rectification_matrix']['data']
        homography = params['homography']
        return CameraConfig(K, dist_params, homography, (img_width, img_height), 
                            projection_matrix, rect_matrix)

    @classmethod
    def from_camera_info(CameraConfig, message):
        ''' Load camera intrinsics from ROS camera info topic, initialize
        camera extrinsics with None value. '''
        model = PinholeCameraModel()
        model.fromCameraInfo(message)
        return CameraConfig(model.K, model.D, None, (model.width, model.height), 
                            model.P, model.R)

    def convert_pixel_to_world(self, pixel_coords): 
        ''' Convert pixel to world coordinates using homography. 
        @param[in]  pixel_coords        (u,v) pixel coordinates. '''
        pixel = np.array([pixel_coords[1], pixel_coords[0], 1])
        return np.matmul(self.H, pixel)

    def rectify_image(self, image):
        image_rectified = np.zeros(np.shape(image)) 
        return cv2.remap(image, self._mapx, self._mapy, cv2.INTER_CUBIC, image_rectified)
    