#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Keypoint detection and description given BGR image (numpy array). To exploit
# the color structure of the intersection first the Laplacian of the BGR is 
# determined and afterwards the FAST/ORB detector is used to extract keypoints. 
# The keypoints are described using ORB description in order to be rotation-
# and intensity-invariant but still be comp. efficient (binary descriptor). 
# Implemented based on Python Open CV2. 
###############################################################################
__all__ = [
    'Features',
]

class Features: 
    pass