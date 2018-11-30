#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Test algorithm performance i.e. the timing required. 
###############################################################################
from comptests import comptest, run_module_tests, comptest_fails
import numpy as np
import os
import time

from duckietown_intnav.algo.imap import IMap

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

# use comptest_fails for a test that is supposed to fail
#@comptest_fails
#def test_supposed_to_fail():
#    raise Exception()


if __name__ == '__main__':
    run_module_tests()
