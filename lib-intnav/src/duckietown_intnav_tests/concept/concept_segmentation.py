#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Proof of concept of image segmentation.  
###############################################################################
import cv2
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from os.path import isfile

test_image_path = "test.png"

def draw_hist2d(ax, x, y):
    r = np.array([[1, 255], [1,255]]) 
    hist, xedges, yedges = np.histogram2d(x.flatten(), y.flatten(), bins=20, range=r)
    xpos, ypos = np.meshgrid(xedges[:-1] + 0.25, yedges[:-1] + 0.25)
    xpos = xpos.flatten('F')
    ypos = ypos.flatten('F')
    zpos = np.zeros_like(xpos)
    dx = 0.5 * np.ones_like(zpos)
    dy = dx.copy()
    dz = hist.flatten()
    ax.bar3d(xpos, ypos, zpos, dx, dy, dz, color='r', zsort='average')
    plt.xlabel('x')
    plt.ylabel('y')

def white_balance(img):
    result = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    avg_a = np.average(result[:, :, 1])
    avg_b = np.average(result[:, :, 2])
    result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
    result = cv2.cvtColor(result, cv2.COLOR_LAB2BGR)
    return result

if not isfile(test_image_path): 
    raise IOError("Invalid input file path !")
img = cv2.imread(test_image_path, cv2.IMREAD_COLOR)
img = white_balance(img)
cv2.imshow('original', img)

R = img[:,:,0]
G = img[:,:,1]
B = img[:,:,2]
Rue = cv2.equalizeHist(img[:,:,0])
Gue = cv2.equalizeHist(img[:,:,1])
Bue = cv2.equalizeHist(img[:,:,2])
equ = np.stack((Rue, Gue, Bue), axis=2)
cv2.imshow('equalized_RGB', equ)

fig = plt.figure(figsize=(15,15))
subset = np.random.choice(img.size/3, 300)
Rue = Rue.flatten()[subset]
Gue = Gue.flatten()[subset]
Bue = Bue.flatten()[subset]
Req = equ[:,:,0].flatten()[subset]
Geq = equ[:,:,1].flatten()[subset]
Beq = equ[:,:,2].flatten()[subset]
ax1 = fig.add_subplot(211, projection='3d')
ax1.scatter(Rue, Gue, Bue, c='skyblue')
ax2 = fig.add_subplot(212, projection='3d')
ax2.scatter(Req, Geq, Beq, c='red')

ycc = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
crs = ycc[:,:,1]
cys = ycc[:,:,2]
crs_eq = cv2.equalizeHist(ycc[:,:,1])
cys_eq = cv2.equalizeHist(ycc[:,:,2])
ycc_equ = np.stack((ycc[:,:,0], crs_eq, cys_eq), axis=2)
cv2.imshow('ycc', ycc)
cv2.imshow('equalized_ycc', ycc_equ)

fig = plt.figure(figsize=(15,15))
ax1 = fig.add_subplot(211, projection='3d')
draw_hist2d(ax1, crs, cys)
ax2 = fig.add_subplot(212, projection='3d')
draw_hist2d(ax2, crs_eq, cys_eq)

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
hue = hsv[:,:,0]
sat = hsv[:,:,1]
hue_eq = cv2.equalizeHist(hsv[:,:,0])
sat_eq = cv2.equalizeHist(hsv[:,:,1])
hsv_equ = np.stack((hsv[:,:,0], hue_eq, sat_eq), axis=2)
cv2.imshow('hsv', hsv)
cv2.imshow('equalized_hsv', hsv_equ)

fig = plt.figure(figsize=(15,15))
ax1 = fig.add_subplot(211, projection='3d')
draw_hist2d(ax1, hue, sat)
ax2 = fig.add_subplot(212, projection='3d')
draw_hist2d(ax2, hue_eq, sat_eq)

lab = cv2.cvtColor(img, cv2.COLOR_BGR2Lab)
laba = lab[:,:,1]
labb = lab[:,:,2]
laba_eq = cv2.equalizeHist(lab[:,:,1])
labb_eq = cv2.equalizeHist(lab[:,:,2])
lab_equ = np.stack((lab[:,:,0], laba_eq, labb_eq), axis=2)
cv2.imshow('lab', lab)
cv2.imshow('equalized_lab', lab_equ)

fig = plt.figure(figsize=(15,15))
ax1 = fig.add_subplot(211, projection='3d')
draw_hist2d(ax1, laba, labb)
ax2 = fig.add_subplot(212, projection='3d')
draw_hist2d(ax2, laba_eq, labb_eq)

segmented = np.zeros(img.size/3, dtype=np.uint8)
a = cys.flatten()
for i in range(25): 
    segmented[np.where(np.logical_and(a < (i+1)*10, a > i*10))] = i*10
segmented = np.reshape(segmented, img[:,:,0].shape)
cv2.imshow('segmented', segmented)

# Finish program - Destroy all created windows.
plt.show()
cv2.waitKey(0)
cv2.destroyAllWindows()

