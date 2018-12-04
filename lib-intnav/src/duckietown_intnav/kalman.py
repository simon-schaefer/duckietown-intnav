#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Kalman filter implementation for localization postprocessing, based on 
# differential drive vehicle model.
# Differential Drive Vehicle Model: 
# x_dot = cos(theta)*v_A
# y_dot = sin(theta)*v_A
# theta_dot = omege = (v_R - v_L)/(2R)
# v_A = (v_R + v_L)/(2R)
###############################################################################
__all__ = [
    'VehicleModel',
    'KalmanFilter',
]

class VehicleModel(object): 
    pass 

class KalmanFilter(object): 
    pass