#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Visualise iMap as image and display it for testing. 
###############################################################################
import cv2
import numpy as np

from duckietown_intnav.imap import IMap

imap = IMap("4")
image = imap.visualize(pose=(0.2, 0.1, 1.5*np.pi/4))

cv2.imshow("intersection", image)
cv2.waitKey(0)