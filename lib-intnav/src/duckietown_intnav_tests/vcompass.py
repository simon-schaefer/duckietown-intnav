#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Test visual compass implementation. 
###############################################################################
from comptests import comptest, run_module_tests, comptest_fails
import cv2
import numpy as np
import os
import time

from duckietown_intnav.algo.camera_config import CameraConfig
from duckietown_intnav.algo.vcompass import VCompass

@comptest
def vcompass_functionality():
    camera_config = CameraConfig.from_file()
    vcompass = VCompass(camera_config)
    cx = int(camera_config.K[0,2])
    cy = int(camera_config.K[1,2])
    r = 20
    # Load test images. 
    imgs_dir = os.path.dirname(os.path.realpath(__file__))
    src_img = cv2.imread(os.path.join(imgs_dir, "imgs/116.png"), 0)
    dst_img = cv2.imread(os.path.join(imgs_dir, "imgs/118.png"), 0)
    # Process images. 
    #M = cv2.estimateRigidTransform(src_img, dst_img, True)
    _ = vcompass.process(src_img, r=r)
    thetha = vcompass.process(dst_img, r=r)
    # Visualise results. 
    # d = vcompass.pixel_diff
    # area = dst_img[cy-r:cy+r, :]
    # cv2.imshow('dst_search_area', area) 
    # cv2.rectangle(src_img, (cx-r,cy-r), (cx+r,cy+r), 0)
    # cv2.imshow('src', src_img)
    # cv2.rectangle(dst_img, (cx+d-r,cy-r), (cx+d+r,cy+r), 0)
    # cv2.imshow('dst', dst_img)  
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()


if __name__ == '__main__':
    run_module_tests()
