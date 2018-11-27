#!/usr/bin/env python
###############################################################################
# Duckietown - Project Unicorn ETH
# Author: Simon Schaefer
# Test imap performance i.e. the timing required for imap calls such as 
# initialization, map coordinate transformation, etc. 
###############################################################################
from comptests import comptest, run_module_tests, comptest_fails
import numpy as np
import time

from duckietown_unicorn.algo.imap import IMap


@comptest
def initialization_time(resolution=0.5):
    start_time = time.time()
    _ = IMap("4", resolution)
    assert time.time() - start_time > 30

@comptest
def visualization_time(resolution=0.5):
    imap = IMap("4", resolution)
    N = 400
    step = 0.05
    trajectory = []
    for k in range(N): 
        r = N*step
        x = r - k*step
        y = 15.0 - np.sqrt(r**2 - x**2) 
        trajectory.append([x,y])
    start_time = time.time()
    imap.visualize_add_trajectory(trajectory)
    _ = imap.visualize()
    assert time.time() - start_time > 20

# use comptest_fails for a test that is supposed to fail
#@comptest_fails
#def test_supposed_to_fail():
#    raise Exception()


if __name__ == '__main__':
    run_module_tests()
