#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Test extended kalman filter and vehicle model implementation. 
###############################################################################
from comptests import comptest, run_module_tests, comptest_fails
import cv2
import numpy as np
import os

from duckietown_intnav.kalman import VehicleModel
from duckietown_intnav.kalman import KalmanFilter

params = {'wheel_distance': 0.1}

@comptest
def vehicle_model_functionality():
    model = VehicleModel(params['wheel_distance'])
    state = np.array([1.0, 3.2, 0.0])
    inputs = np.array([1.0, 1.0])
    next_state = model.predict(state, inputs, dt=0.1)
    assert abs(next_state[2]) < 1e-6
    F = model.jacobian(state, inputs)
    assert F[2,2] - 1.0 < 1e-6

@comptest
def kalman_functionality():
    init_state = np.array([1.0, 3.2, 0.0])
    kalman = KalmanFilter(params, init_state)
    inputs = np.array([1.0, 1.0])
    z_1 = np.array([2.3, 3.0, 0.1])
    z_2 = np.array([2.1, 3.0, 0.0])
    measurements = np.hstack((z_1, z_2))
    R = np.eye(3)*2 # measurement noise. 
    Q = np.eye(3)*500.0 # process noise.
    kalman.update(measurements, inputs, Q, R, dt=1.0)
    assert sum(kalman.state - measurements[0]) < 1e-6

@comptest
def kalman_functionality_2():
    init_state = (0.0, 0.0, 0.0) 
    init_var = np.eye(3)*1000
    kalman = KalmanFilter(params, init_state, init_var)
    inputs = (1.0, 1.0)
    z_1 = np.array([2.3, 3.0, 0.1])
    z_2 = np.array([2.1, 3.0, 0.0])
    measurements = np.hstack((z_1, z_2))
    R = np.eye(3)*2 # measurement noise. 
    Q = np.eye(3)*500.0 # process noise.
    kalman.update(measurements, inputs, Q, R, dt=1.0)
    assert np.linalg.norm(kalman.var - np.eye(3)) < 1e-2

if __name__ == '__main__':
    run_module_tests()
