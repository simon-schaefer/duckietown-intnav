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

from duckietown-intnav.algo.camera_config import CameraConfig
from duckietown-intnav.algo.vcompass import VCompass

R = 30

@comptest
def vcompass_functionality():
    camera_config = CameraConfig.from_file()
    vcompass = VCompass(camera_config)
    # Load test images. 
    imgs_dir = os.path.dirname(os.path.realpath(__file__))
    src_img = cv2.imread(os.path.join(imgs_dir, "imgs/116.png"), 0)
    dst_img = cv2.imread(os.path.join(imgs_dir, "imgs/118.png"), 0)
    # Process images. 
    vcompass.process(src_img)
    dth, var = vcompass.process(dst_img)
    print(dth, var)
    # Visualise results. 
    # cx = int(camera_config.K[0,2])
    # cy = int(camera_config.K[1,2])
    # d = vcompass.pixel_diff
    # area = dst_img[cy-R:cy+R, :]
    # cv2.imshow('dst_search_area', area) 
    # cv2.rectangle(src_img, (cx-R,cy-R), (cx+R,cy+R), 0)
    # cv2.imshow('src', src_img)
    # cv2.rectangle(dst_img, (cx+d-R,cy-R), (cx+d+R,cy+R), 0)
    # cv2.imshow('dst', dst_img)  
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

@comptest
def vcompass_looping(): 
    camera_config = CameraConfig.from_file()
    vcompass = VCompass(camera_config)
    # Load test images. 
    imgs_dir = os.path.dirname(os.path.realpath(__file__))
    src_img = cv2.imread(os.path.join(imgs_dir, "imgs/116.png"), 0)
    vcompass.process(src_img)
    # Iterate over all test images and estimate overall rotation 
    # change. 
    angle_sum = 0.0
    for i in range(208): 
        img = cv2.imread(os.path.join(imgs_dir, "imgs/"+ str(i)+".png"), 0)
        dth, var = vcompass.process(img)
        if var > 1.0: 
            continue
        angle_sum = angle_sum + dth
    assert 80 < angle_sum < 120

if __name__ == '__main__':
    run_module_tests()
