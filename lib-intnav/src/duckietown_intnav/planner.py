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
    if direction == 0:
        xs, ys = bezier_curve([(0,-0.1225),(0.715,-0.1225)], n_steps)
        xs, ys = np.insert(xs, 0, 0.865), np.insert(ys, 0, -0.1225)
    # Case left. 
    elif direction == -1: 
        xs, ys = bezier_curve([(0,-0.1225),(0.18,-0.1225),(0.36,-0.0785),(0.405,0.1025),(0.405,0.2825)], n_steps)
        xs, ys = np.insert(xs, 0, 0.405), np.insert(ys, 0, 0.4325)
    # Case right
    elif direction == +1: 
        xs, ys = bezier_curve([(0,-0.1225),(0.0675,-0.1225),(0.135,-0.145),(0.16,-0.21),(0.16,-0.2825)], n_steps)
        xs, ys = np.insert(xs, 0, 0.16), np.insert(ys, 0, -0.4325)
    else: 
        raise ValueError("Invalid path direction !")
    assert len(xs) == len(ys)
    xs, ys = np.insert(xs, len(xs), -0.15), np.insert(ys, len(ys), -0.1225)
    path = [(xs[i],ys[i]) for i in range(len(xs))]
    return path
