#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Proof of concept of seperated color gradient search. 
###############################################################################
import cv2
import matplotlib.pyplot as plt
import numpy as np
from os.path import isfile
from scipy.misc import imresize
from scipy.signal import fftconvolve
import time

def image_intersection_colors(): 
    img = np.zeros((640,640,3), dtype=np.uint8)
    img[200:,200:400,2] = 255
    img[200:,:200,2] = 255
    img[200:300,:200,1] = 255
    img[400:,:,0] = 255
    img[400:,:,1] = 255
    img[400:,:,2] = 255
    return img

def image_real(): 
    test_image_path = "test.png" 
    if not isfile(test_image_path): 
        raise IOError("Invalid input file path !")
    img = cv2.imread(test_image_path, cv2.IMREAD_COLOR)
    return img

def image_single_edge():
    img = np.zeros((640,640,3), dtype=np.uint8)
    img[:,200:,0] = 255
    img[:,:200,2] = 255
    return img

# Load image.
img = image_real()
fast = cv2.FastFeatureDetector_create(threshold=6, nonmaxSuppression=True)
orb = cv2.ORB_create()

# Processing data. 
start_time = time.time()
resize_factor = 2
dim = (img.shape[0]/resize_factor, img.shape[1]/resize_factor)
data = imresize(img, (dim[0],dim[1],3))

# compute the convolution as a 3d convolution. 
gradients = cv2.Laplacian(data,cv2.CV_64F)
#gradients = data
gradients = imresize(gradients, img.shape)
#kp = fast.detect(gradients,None)
kp = orb.detect(gradients,None)
kp, des = orb.compute(gradients, kp)
for K,D in zip(kp, des): 
    x, y = int(K.pt[0]), int(K.pt[1])
gradients = cv2.drawKeypoints(gradients, kp, gradients, color=(255,0,0))
print("Time usage = %s" % str(time.time() - start_time))

# Show output images. 
cv2.imshow('original', img)
cv2.imshow('gradients', gradients)
combined = cv2.addWeighted(img, 0.09, gradients, 0.91, 1.0)
cv2.imshow('combined', combined)
#cv2.imwrite('filter3.png', combined)

# Finish program - Destroy all created windows.
cv2.waitKey(0)
cv2.destroyAllWindows()