#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Marta Tintore
# Visualise path as image and display it for testing. 
###############################################################################
import matplotlib.pyplot as plt

from duckietown_intnav.planner import path_generate

fig = plt.figure()
k = 1
for direction in [-1,0,1]: 
    path = path_generate(direction)
    xs = [z[0] for z in path]
    ys = [z[1] for z in path]
    ax = fig.add_subplot(3,1,k)
    ax.plot(xs, ys)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    k = k + 1
plt.show()