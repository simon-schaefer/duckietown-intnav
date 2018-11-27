#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Visualise iMap as image and display it for testing. 
###############################################################################
import matplotlib
from matplotlib import pyplot as plt
import numpy as np

from duckietown_intnav.algo.imap import IMap

inter = IMap("3LR", 0.1)
b = inter.visualize()

ca = np.array([[IMap.v_env,0,0,0],
               [IMap.v_str,200,200,200],
               [IMap.v_whi,255,255,255],
               [IMap.v_red,255,0,0], 
               [IMap.v_yel,255,255,0]])

colors = ca[ca[:,0].argsort()][:,1:]/255.
cmap = matplotlib.colors.ListedColormap(colors)
norm = matplotlib.colors.BoundaryNorm(np.arange(len(ca)+1)-0.5, len(ca))

plt.imshow(b, cmap=cmap, norm=norm)
plt.show()