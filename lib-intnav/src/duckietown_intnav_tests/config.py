#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Test configuration loading. 
###############################################################################
from comptests import comptest, run_module_tests, comptest_fails
import time

from duckietown-intnav.algo.camera_config import CameraConfig

@comptest
def load_camera_config_file():
    config = CameraConfig.from_file()
    assert (config.K != None).all()
    assert (config.dist_params != None).all()
    assert config.width != None
    assert config.width != None
    assert (config.P != None).all()
    assert (config.R != None).all()
    assert (config.H != None).all()

if __name__ == '__main__':
    run_module_tests()
