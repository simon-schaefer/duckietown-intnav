#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Visualise iMap as image and display it for testing. 
###############################################################################
import cv2
import numpy as np

from duckietown_intnav.algo.imap import IMap

imap = IMap("4", 0.5)

cv2.imshow("intersection", imap.data_colored)
cv2.waitKey(0)