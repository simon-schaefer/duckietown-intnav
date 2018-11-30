#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Load and make availabe camera configuration data from configuration file 
# or camera configuration message string.  
###############################################################################
__all__ = [
    'CameraConfig',
]

from numpy import asarray, reshape
import os.path
import yaml

class CameraConfig: 

    def __init__(self): 
        self.K = None
        self.dist_params = None
        self.img_width = None
        self.img_height = None
        self.projection_matrix = None
        self.rect_matrix = None

    @classmethod
    def from_file(self, file_path="../data/camera.yaml"):
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
        self.K = reshape(asarray(params['camera_matrix']['data']), (3,3))
        self.dist_params = asarray(params['distortion_coefficients'])
        self.img_width = params['image_width']
        self.img_height = params['image_height']
        self.projection_matrix = asarray(params['projection_matrix']['data'])
        self.rect_matrix = reshape(asarray(params['rectification_matrix']['data']), (3,3))
        return self

        