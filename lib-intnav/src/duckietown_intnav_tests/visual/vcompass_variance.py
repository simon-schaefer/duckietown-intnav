#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Visualize visual compass variance implementation. 
###############################################################################
import cv2
import matplotlib.pyplot as plt
import numpy as np
import os

from duckietown-intnav.algo.camera_config import CameraConfig
from duckietown-intnav.algo.vcompass import VCompass

R = 30

camera_config = CameraConfig.from_file()
vcompass = VCompass(camera_config)
# Load test images. 
imgs_dir = os.path.dirname(os.path.realpath(__file__))
src_img = cv2.imread(os.path.join(imgs_dir, "../imgs/116.png"), 0)
vcompass.process(src_img)
# Iterate over all test images and estimate overall rotation 
# change. 
angle_sum = 0.0
angle_mean = []
angle_lower = []
angle_upper = []
for i in range(208): 
    img = cv2.imread(os.path.join(imgs_dir, "../imgs/"+ str(i)+".png"), 0)
    dth, var = vcompass.process(img)
    if abs(var) >= 1.0: 
        continue
    angle_sum = angle_sum + dth
    angle_mean.append(angle_sum)
    angle_lower.append(angle_sum - var)
    angle_upper.append(angle_sum + var)
# Visualization. 
fig, ax = plt.subplots()
ax.fill_between(range(len(angle_mean)), angle_upper, angle_lower, color='gray', alpha=0.5)
ax.plot(angle_mean, 'r-', label='Mean')
ax.legend(loc='upper left')
plt.xlabel("Image index")
plt.ylabel("Overall angle")
plt.show()
