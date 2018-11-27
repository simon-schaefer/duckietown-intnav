#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Proof of concept of color transition filter.  
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
    #img[:,:200,2] = 255
    return img

def kernel3d(): 
    ''' Create 3D generalized (color) Sobel kernel.
    @param[out]     kernel1     vertical gradients kernel.
    @param[out]     kernel2     horizontal gradients kernel. '''
    kernel1 = np.zeros((3,3,1))
    kernel2 = np.zeros((3,3,1))
    kernel1[:,:,0] = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])
    kernel2[:,:,0] = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]])
    kernel1[:,:,0] = np.array([[-1, 0, 1], [2, 0, -2], [-1, 0, 1]])
    kernel2[:,:,0] = np.array([[2, -1, -1], [0, 0, 0], [-2, 1, 1]])
    return kernel1, kernel2

def convolve3d(x, threshold = 10): 
    kernel1, kernel2 = kernel3d()
    convolved = fftconvolve(x, kernel1, mode='valid') + fftconvolve(x, kernel2, mode='valid')
    convolved[convolved < threshold] = 0
    return convolved

# Load image.
img = image_real()

# Processing data. 
start_time = time.time()
resize_factor = 4
dim = (img.shape[0]/resize_factor, img.shape[1]/resize_factor)
data = imresize(img, (dim[0],dim[1],3))
#data = cv2.cvtColor(data, cv2.COLOR_BGR2YCrCb)
#data[:,:,1] = 0

# compute the convolution as a 3d convolution. 
gradients = convolve3d(data)
#gradients = np.max(gradients, axis=2)
#gradients = gradients[:,:,2]
gradients = imresize(gradients, img.shape)
print("Time usage = %s" % str(time.time() - start_time))

# Show output images. 
cv2.imshow('original', img)
cv2.imshow('gradients', gradients)
combined = cv2.addWeighted(img, 0.09, gradients, 0.91, 1.0)
cv2.imshow('combined', combined)
cv2.imwrite('filter_real.png', combined)

# Finish program - Destroy all created windows.
cv2.waitKey(0)
cv2.destroyAllWindows()


# def kernel3d(): 
#     ''' Create 3D generalized (color) Sobel kernel.
#     @param[out]     kernel1     vertical gradients kernel.
#     @param[out]     kernel2     horizontal gradients kernel. '''
#     kernel1 = np.zeros((3,3,1))
#     kernel2 = np.zeros((3,3,1))
#     kernel1[:,:,0] = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])
#     kernel2[:,:,0] = np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]])
#     kernel1[:,:,0] = np.array([[-1, 0, 1], [2, 0, -2], [-1, 0, 1]])
#     kernel2[:,:,0] = np.array([[2, -1, -1], [0, 0, 0], [-2, 1, 1]])
#     return kernel1, kernel2

# def convolve3d(x, threshold = 10): 
#     kernel1, kernel2 = kernel3d()
#     convolved = fftconvolve(x, kernel1, mode='valid') + fftconvolve(x, kernel2, mode='valid')
#     convolved[convolved < threshold] = 0
#     return convolved