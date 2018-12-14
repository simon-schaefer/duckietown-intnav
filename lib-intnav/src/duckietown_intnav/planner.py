#!/usr/bin/env python
###############################################################################
# Duckietown - Project Unicorn ETH
# Author: Marta Tintore
# Plan optimal path from start to goal point, depending on intersection type.
###############################################################################
__all__ = [
    'path_generate',
]

import numpy as np
from scipy.misc import comb

def bernstein_poly(i, n, t):
    """ The Bernstein polynomial of n, i as a function of t. """
    return comb(n, i) * ( t**(n-i) ) * (1 - t)**i

def bezier_curve(points, n_steps=20):
    """ Given a set of control points, return the bezier curve defined
    by the control points. Points should be a list of lists, or list of tuples
    such as [ [1,1], [2,3], [4,5], ..[Xn, Yn] ].
    @param[in]  n_steps         number of time steps. """
    n_points = len(points)
    xPoints = np.array([p[0] for p in points])
    yPoints = np.array([p[1] for p in points])
    t = np.linspace(0.0, 1.0, n_steps)
    polynomial_array = np.array([bernstein_poly(i, n_points-1, t) for i in range(0,n_points)])
    xvals = np.dot(xPoints, polynomial_array)
    yvals = np.dot(yPoints, polynomial_array)
    return xvals, yvals

def path_generate(direction, n_steps=20):
    ''' Generate path given driving direction using bezier curves
    based on (hard coded) optimal path points.
    @param[in]  direction       l=-1, r=1, s=0. '''
    xs, ys = None, None
    # Case straight.
    if direction == 0 or direction==1:
        xs, ys = np.linspace(1,-0.15,60),-0.1225*np.ones((60,))
    # Case left.
    elif direction == -1 :
        xs, ys = bezier_curve([(-0.15,-0.1225),(0.04,-0.07),(0.285,0.1225),(0.37,0.32)], n_steps)
        xs, ys = np.insert(xs,0 , np.ones((20,))*0.37), np.insert(ys, 0, np.linspace(1,0.34,20))
    # Case right
    elif direction == +1:
        xs, ys = bezier_curve([(-0.14,-0.1225),(-0.06,-0.1225),(0.07,-0.30)], n_steps)
        xs, ys = np.insert(xs,0 , np.ones((20,))*0.10), np.insert(ys, 0, np.linspace(-1.2,-0.51,20))
    else:
        raise ValueError("Invalid path direction !")
    assert len(xs) == len(ys)
    xs, ys = np.insert(xs, len(xs), np.linspace(-0.16,-1,20)), np.insert(ys, len(ys), -0.1225*np.ones((20,)))
    path = np.zeros((len(xs),2))
    path[:,0] = np.asarray(np.flipud(xs))
    path[:,1] = np.asarray(np.flipud(ys))
    print("path: ", path)
    return path
