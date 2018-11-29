#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Test imap performance i.e. the timing required for imap calls such as 
# initialization, map coordinate transformation, etc. 
###############################################################################
import cv2
from comptests import comptest, run_module_tests, comptest_fails
import numpy as np
import os
import time

from duckietown_intnav.algo.imap import IMap
from duckietown_intnav.algo.features import Features
from duckietown_intnav.algo.matching import Matching

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
def feature_matching_time(): 
    # Get test and database image and initialize feature and 
    # matching algorithms. 
    img_db = IMap("3SL").data_colored
    img_file = os.path.dirname(os.path.realpath(__file__))
    img_file = os.path.join(img_file, "concept/test.png")
    img = cv2.imread(img_file)
    features = Features("ORB", "PATCH")
    matcher = Matching()
    # Determine keypoints and descriptors of database image
    # (merely once !).
    kp_db, descs_db = features.process(img_db)
    #features.draw(img_db, kp_db)
    # Find and match features in test image with database.
    start_time = time.time()
    kp, descs = features.process(img)
    #features.draw(img,kp)
    matches = matcher.process(kp, kp_db, descs, descs_db)
    print(time.time() - start_time)
    assert (time.time() - start_time)*1000 < 100
    matcher.draw(img, kp, img_db, kp_db, matches)

# use comptest_fails for a test that is supposed to fail
#@comptest_fails
#def test_supposed_to_fail():
#    raise Exception()


if __name__ == '__main__':
    run_module_tests()
