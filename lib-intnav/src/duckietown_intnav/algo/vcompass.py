#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Visual compass - Orientation estimation by using monocular camera. Between
# subsequent frames keep track of a certain patch in the images horizon, i.e.
# in one frame fix a patch in the center and look for it in the horizon line 
# in the next image (planar movement !) by computing sum of squared distances 
# between fixed patch and all patches in horizon line.
###############################################################################
__all__ = [
    'VCompass',
]

import cv2
import os
import numpy as np
import yaml

class VCompass(object): 

    def __init__(self, camera_config): 
        # Load class parameters. 
        params = {}
        try: 
            config_file = os.path.dirname(os.path.realpath(__file__))
            config_file = os.path.join(config_file, "../data/parameter.yaml")
            with open(config_file, 'r') as stream:
                params = yaml.load(stream)
        except (IOError, yaml.YAMLError): 
            raise IOError("Unknown or invalid parameters file !") 
        self._num_patches = params['vcompass']['num_candidates']
        self._patch_radius = params['vcompass']['patch_radius']
        self._config_num_samples = params['vcompass']['num_config_samples']
        # Class variables.     
        self.pixel_diff = None
        self._camera_config = camera_config
        self._prev_patch = None
        self._config_ssds = []
        self.is_initialised = False
        self.is_configured = False

    def process(self, image):
        ''' Compare input image with center patch* in previous image, search for
        lowest SSD patch w.r.t. patch* in horizon (center) line in current image
        and estimate rigid transformation (rotation angle) from difference pixel 
        coordinate difference between patch and patch*. 
        @param[in]      image       current camera image (raw). 
        @param[in]      N           number of patches to check for SSD (N-2). 
        @param[in]      r           patch radius (such that patch size = (2r+1)^2).
        @param[out]     angle_diff  rotation angle between both images [rad] or 
                                    None if no overlaying patch found. 
        @param[out]     variance    estimation of variance of angle diff estimate. '''
        N = self._num_patches
        r = self._patch_radius
        if not self.is_initialised:
            self._prev_patch = self._cut_center_patch(image, r)
            self.pixel_diff = 0
            self.is_initialised = True
            return 0.0, 0.0
        # Compute center points to iterate (along horizon line).
        center_x = int(self._camera_config.K[0,2])
        center_y = int(self._camera_config.K[1,2])
        delta_x = float(self._camera_config.width-2*r)/N
        centers = [(int(r+x*delta_x),center_y) for x in range(1,N-1)]
        # Iterate over all center point and determine smallest SSD. 
        min_ssd = np.Infinity
        min_delta = -1
        for cx,cy in centers:
            patch = self._cut_patch(image, (cx,cy), r)
            ssd = np.sum((patch - self._prev_patch)**2)
            if ssd < min_ssd: 
                min_ssd = ssd
                min_delta = cx - center_x 
        self.pixel_diff = min_delta
        # Gather configuration samples if in configuration phase. 
        if not self.is_configured: 
            self._config_ssds.append(min_ssd)
            if len(self._config_ssds) >= self._config_num_samples: 
                self._config_ssds = np.mean(self._config_ssds)
                self.is_configured = True
        # TODO: Estimate inaccuracy based on ssd. 
        variance = min(10.0, min_ssd/np.mean(self._config_ssds)*1.0)
        # Based on minimal delta estimate rigid body rotation angle. 
        #wc = self._camera_config.convert_pixel_to_world((center_x, center_y))
        angle_diff = float(min_delta)/(self._camera_config.width/2)*60.0
        # Reset previous patch to current image. 
        self._prev_patch = self._cut_center_patch(image, r)
        return angle_diff, variance

    def _cut_center_patch(self, image, r): 
        ''' Cut patch at images center (convergence point). '''
        cx = int(self._camera_config.K[0,2])
        cy = int(self._camera_config.K[1,2])
        return self._cut_patch(image, (cx,cy), r)

    def _cut_patch(self, image, center, r): 
        ''' Cut patch with given radius and at given center point, 
        such that patch is of size (2*r + 1)^2. 
        @param[in]  center      (cx,cy) center in pixel coords.
        @param[in]  r           patch radius. '''
        return image[center[1]-r-1:center[1]+r, center[0]-r-1:center[0]+r]


# M = cv2.estimateRigidTransform(src_img, dst_img, False)
# angle = np.arctan2(M[1,0],M[0,0])
