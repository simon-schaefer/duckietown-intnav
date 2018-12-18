#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Kalman filter implementation for localization postprocessing, based on
# differential drive vehicle model.
# x_dot = cos(theta)*v_A
# y_dot = sin(theta)*v_A
# theta_dot = omege = (v_R - v_L)/(2R)
# v_A = (v_R + v_L)/(2R)
###############################################################################
__all__ = [
    'VehicleModel',
    'KalmanFilter',
]

import numpy as np

''' Vehicle model for differential drive duckiebot, with left wheel velocity v_L,
right wheel velocity v_R and wheel distance R. '''
class VehicleModel(object):

    def __init__(self, R):
        self.R = R

    def predict(self, state, inputs, dt,direction):
        ''' Predict next state based on inputs and current state.
        @param[in]  state       (x, y, thetha) current state description.
        @param[in]  inputs      (vr, vl) velocity of left and right wheel.
        @param[in]  dt          time interval to predict [s].
        @param[out] state_next  (xn, yn, thetan) next state description. '''
        x, y, theta = state[0], state[1], state[2]
        #vr, vl = inputs[0], inputs[1]
        #va = (vr + vl)/2.0
        va = inputs[0]
        #omega = (vr - vl)/(self.R)
        if(direction=='R'): omega = inputs[1]/2
        else: omega = inputs[1]
        vr = va + omega*self.R/2
        vl = va - omega*self.R/2
        xn = x + np.cos(theta)*va*dt
        yn = y + np.sin(theta)*va*dt
        thetan = theta + omega*dt
        print("Vehicle Model")
        print('VL: ',vl,'VR: ',vr)
        print('R: ',self.R)
        print(omega,dt,theta,thetan)

        return np.array([xn, yn, thetan])

    def jacobian(self, state, inputs):
        ''' Return system dynamics Jacobian, evaluated at the given
        state and inputs. '''
        theta = state[2]
        vr, vl = inputs[0], inputs[1]
        va = (vr + vl)/2.0
        F = np.eye(3)
        F[0,2] = -np.sin(theta)*va
        F[1,2] =  np.cos(theta)*va
        return F

''' Extended Kalman filter implementing vehicle dynamics of differential drive
duckiebot, using the following discrete-time predict and update equations:
Predict
- x_k| = f(x_k-1, u_k)
- P_k| = F_k P_k-1 F_k^T + Q_k
Update
- y_k = z_k - h(x_k|)
- S_k = H_k P_k| H_k^T + R_k
- K_K = P_k| H_k^T S_k^(-1)
- x_k = x_k| + K_k y_k
- P_k = (I - K_k H_k) P_k|
with F_k = df/dx|x_k-1,u_k and H_k = dh/dx|x_k|. '''
class KalmanFilter(object):

    def __init__(self, model_params, init_state, init_var=np.zeros((3,3))):
        ''' Access model class (= f, system dynamics) and initialize state
        as well as variance.
        @param[in]  model_params    tuple of model parameters.
        @param[in]  init_state      (x0,y0,theta0) initial state, np.array.
        @param[in]  init_variance   initial variance [3x3], np.array. '''
        self.model = VehicleModel(R=model_params['wheel_distance'])
        self.state = init_state
        self.var = init_var

    def update(self, z, u, Q, R, dt, direction):
        ''' Extended Kalman filter prediction and update step.
        @param[in]  z           [(xm, ym, thetham), ...] measurement vector, np.array.
        @param[in]  u           (vr, vl) velocity of left and right wheel.
        @param[in]  Q           process uncertainties [3x3], np.array.
        @param[in]  R           measurement uncertainties [3x3], np.array.
        @param[in]  dt          time interval to predict [s].
        Exclude prediction step (pure sensor fusion) for u = None by Q = inf.'''
        xlast = self.state
        Plast = self.var
        # Check if open loop control.
        if u is None:
            u = np.array([0.0, 0.0])
            Q = np.eye(3)*100.0
        # Prediction step.
        F = self.model.jacobian(xlast, u)
        xprior = self.model.predict(xlast, u, dt, direction)
        Pprior = F * Plast * np.transpose(F) + Q
        # Prepare measurements vector.
        #print('z: ', z)
        #print('shape z: ', np.shape(z))
        #print('shape z [0]: ', np.shape(z)[0])
        assert np.shape(z)[0] % 3 == 0
        num_measurements = np.shape(z)[0]/3
        H = np.zeros((3*num_measurements,3))
        Rnp = np.kron(np.eye(num_measurements), R)
        for i in range(num_measurements):
            H[3*i:3*i+3,:] = np.eye(3)
        # Update step (H = eye(3)).
        y = z - np.matmul(H,xprior)
        S = np.matmul(np.matmul(H,Pprior),np.transpose(H)) + Rnp
        K = np.matmul(np.matmul(Pprior,np.transpose(H)),np.linalg.inv(S))
        self.state = xprior + np.matmul(K,y)
        self.var = np.matmul(np.eye(3) - np.matmul(K,H),Pprior)

    def predict(self, u, dt,direction):
        ''' Extended Kalman filter prediction step (pure state update).
        @param[in]  u           (vr, vl) velocity of left and right wheel.
        @param[in]  dt          time interval to predict [s].
        For u == None assume to stand still, i.e. u = (0,0). '''
        xlast = self.state
        # Check if open loop control.
        if u is None:
            u = np.array([0.0, 0.0])
        # Prediction step.
        self.state = self.model.predict(xlast, u, dt,direction)
