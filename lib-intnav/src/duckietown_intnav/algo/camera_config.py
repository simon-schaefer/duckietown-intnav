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

from numpy import asarray, reshape
import numpy as np
import os.path
import yaml

class CameraConfig: 

    def __init__(self, K, dist, H, dim, P, R): 
        self.K = K
        self.dist_params = dist
        self.width = dim[0]
        self.height = dim[1]
        self.P = P                 
        self.R = R
        self.H = H

    @classmethod
    def from_file(CameraConfig, file_path="../data/camera.yaml"):
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
        K = reshape(asarray(params['camera_matrix']['data']), (3,3))
        dist_params = asarray(params['distortion_coefficients'])
        img_width = params['image_width']
        img_height = params['image_height']
        projection_matrix = asarray(params['projection_matrix']['data'])
        rect_matrix = reshape(asarray(params['rectification_matrix']['data']), (3,3))
        homography = reshape(asarray(params['homography']), (3,3))
        return CameraConfig(K, dist_params, homography, (img_width, img_height), 
                            projection_matrix, rect_matrix)

    def convert_pixel_to_world(self, pixel_coords): 
        ''' Convert pixel to world coordinates using homography. 
        @param[in]  pixel_coords        (u,v) pixel coordinates. '''
        pixel = np.array([pixel_coords[1], pixel_coords[0], 1])
        return np.matmul(self.H, pixel)

        