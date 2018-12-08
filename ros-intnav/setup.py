#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Intersection map setup.py
###############################################################################
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['duckietown-intnav'],
    package_dir={'': '../lib-intnav'}
)

setup(**setup_args)
