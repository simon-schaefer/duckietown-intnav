#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Test algorithm performance i.e. the timing required. 
###############################################################################
from comptests import comptest, run_module_tests, comptest_fails
import cv2
import numpy as np
import os
import time

from duckietown-intnav.algo.camera_config import CameraConfig
from duckietown-intnav.algo.imap import IMap
from duckietown-intnav.algo.vcompass import VCompass

@comptest
def imap_initialization_time():
    start_time = time.time()
    _ = IMap("4")
    assert (time.time() - start_time)*1000 < 300

@comptest
def imap_visualization_time():
    data = IMap("4")
    N = 400
    step = 0.05
    trajectory = []
    for k in range(N): 
        r = N*step
        x = r - k*step
        y = 15.0 - np.sqrt(r**2 - x**2) 
        trajectory.append([x,y])
    start_time = time.time()
    data.visualize_add_trajectory(trajectory)
    _ = data.visualize()
    assert (time.time() - start_time)*1000 < 20

@comptest
def vcompass_time(): 
    camera_config = CameraConfig.from_file()
    vcompass = VCompass(camera_config)
    # Load test images. 
    imgs_dir = os.path.dirname(os.path.realpath(__file__))
    src_img = cv2.imread(os.path.join(imgs_dir, "imgs/116.png"), 0)
    dst_img = cv2.imread(os.path.join(imgs_dir, "imgs/118.png"), 0)
    # Process images. 
    vcompass.process(src_img)
    start_time = time.time()
    dth, var = vcompass.process(dst_img)
    assert (time.time() - start_time)*1000 < 5

# use comptest_fails for a test that is supposed to fail
#@comptest_fails
#def test_supposed_to_fail():
#    raise Exception()


if __name__ == '__main__':
    run_module_tests()
