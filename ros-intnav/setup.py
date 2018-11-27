#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Intersection map setup.py
###############################################################################
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['duckietown_intnav'],
    package_dir={'': '../lib-intnav/src'}
)

setup(**setup_args)
