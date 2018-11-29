#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Keypoint detection and description given BGR image (numpy array). To exploit
# the color structure of the intersection first the Laplacian of the BGR is 
# determined and afterwards the FAST/ORB detector is used to extract keypoints. 
# The keypoints are described using ORB description in order to be rotation-
# and intensity-invariant but still be comp. efficient (binary descriptor). 
# Implemented based on Python Open CV2. 
###############################################################################
__all__ = [
    'Features',
]

import cv2
import os
import numpy as np
from scipy.misc import imresize
import warnings
import yaml

class Features: 

    detectors_options = ["ORB", "FAST"]
    descriptors_options = ["ORB", "PATCH"] 

    def __init__(self, detector="ORB", descriptor="ORB"): 
        ''' Initialize detector and descriptor algorithms. 
        @param[in]    detector    see options above. 
        @param[in]    descriptor  see options above. '''
        # Load program parameters. 
        params = {}
        try: 
            config_file = os.path.dirname(os.path.realpath(__file__))
            config_file = os.path.join(config_file, "../data/parameter.yaml")
            with open(config_file, 'r') as stream:
                params = yaml.load(stream)
        except (IOError, yaml.YAMLError): 
            raise IOError("Unknown or invalid parameters file !") 
        # Check input arguments validity. 
        if not detector in Features.detectors_options: 
            raise ValueError("Invalid detector %s !" % detector)
        if not descriptor in Features.descriptors_options: 
            raise ValueError("Invalid descriptor %s !" %descriptor)
        # Initialize feature detector. 
        self._detector = None
        if detector == "FAST": 
            self._detector = cv2.FastFeatureDetector_create(threshold=6, 
                                                            nonmaxSuppression=True)
        if detector == "ORB": 
            self._detector = cv2.ORB_create()
        # Initialize feature descriptor. 
        self._descriptor = None
        if descriptor == "ORB": 
            self._descriptor = cv2.ORB_create()
        if descriptor == "PATCH":
            self._descriptor = Features.PatchDescriptor()
        # Set debug parameters. 
        self._debug_mode = params['debug_mode']
        self._min_num_keypoints = params['features']['min_count']
        self._min_pixel_std_dev = params['features']['min_pixel_std_deviation']

    def process(self, data, resize_factor=4): 
        ''' Compute keypoints and descriptors from data BGR input image 
        by applying Laplacian operator and initialized detector and descriptor. 
        In order to accelerate computation the image is resized before computing
        the Laplacian using linear interpolation and transformed to original
        size afterwards. 
        @param[in]  data            BGR input image as numpy array. 
        @param[in]  resize_factor   image downsampling factor (accuracy dec. !).
        @param[out] keypoints       pixel coordinates of keypoints. 
        @param[out] descs           keypoints descriptors. '''
        assert np.shape(data)[2] == 3
        dim = (data.shape[0]/resize_factor, data.shape[1]/resize_factor)
        img = imresize(data, (dim[0],dim[1],3))
        # Compute the convolution as a 3d convolution. 
        gradients = cv2.Laplacian(img,cv2.CV_64F)
        gradients = imresize(gradients, data.shape)
        kp = self._detector.detect(gradients, None)
        keypoints, descs = self._descriptor.compute(data, kp)
        # Debug information. 
        if self._debug_mode: 
            num_kp = len(keypoints)
            if num_kp < self._min_num_keypoints:
                warnings.warn("Small detected features count = %d !" % num_kp,RuntimeWarning)
            else: 
                num_samples = min(10, num_kp)
                samples = np.random.choice(keypoints, num_samples, replace=False)
                distances = [point.pt[0] + point.pt[1]  for point in samples]
                if np.std(distances) < self._min_pixel_std_dev: 
                    warnings.warn("Bad detected feature distribution !", RuntimeWarning)
        return keypoints, descs

    @staticmethod
    def draw(image, keypoints):
        ''' Visualize keypoints in input image by overlaying it with red dots. '''
        overlayed = cv2.drawKeypoints(image, keypoints, image, color=(255,0,0))
        cv2.imshow("keypoints", overlayed)
        cv2.waitKey(0)

    class PatchDescriptor: 
        ''' Simple patch descriptor i.e. get adjacent pixel as descriptor 
        for each keypoint (colored patch !). '''
        def __init__(self, patch_radius=2):
            self.patch_radius = patch_radius

        def compute(self, image, keypoints): 
            r = self.patch_radius
            N = len(keypoints)
            patch_size = (2*r+1)**2
            descriptors = np.zeros((N,patch_size*3), dtype=np.uint8)
            keypoints_new = []
            k = 0
            for i in range(N):
                kp = keypoints[i]
                x = int(kp.pt[0])
                y = int(kp.pt[1])
                patch = image[x-r:x+r+1, y-r:y+r+1,:]
                patch = np.reshape(patch, (patch_size,3))
                if not self.check_patch(patch):
                    continue
                descriptors[k,:] = patch.flatten()
                keypoints_new.append(kp)
                k += 1
            descriptors = descriptors[:k,:]
            return keypoints_new, descriptors

        @staticmethod
        def check_patch(patch, min_num_colors=3): 
            ''' Check whether patch is an acceptable (e.g. unique)
            keypoint descriptor, since it contains at least 3 colors. 
            It turned out that HSV or clustering based approaches are 
            way too computationally expensive. 
            @param[in]  patch       patch BGR array. '''
            N = np.shape(patch)[0]
            color_max = np.max(patch, axis=0)
            color_min = np.min(patch, axis=0)
            # For the pixel to be unique at least 3 colors have to 
            # be contained in the patch. 
            #contains = {'red': False, 'yellow': False, 'white': False, 'gray': False}
            #return sum(contains.values()) >= min_num_colors
            #return (color_max[0] > 200 and color_min[0] < 100) \
            #    or (color_max[1] > 200 and color_min[1] < 100)
            return (all(color_max > 200))

            
            