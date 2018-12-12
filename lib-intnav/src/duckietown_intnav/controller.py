#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Marta Tintore
# Pure pursuit controller implementation.
###############################################################################
__all__ = [
    'pure_pursuit',
]

import numpy as np
from scipy.spatial.distance import cdist

def pure_pursuit(pose, path, wheel_distance,
                 adm_error=0.005, la_dis=0.03, t_step=0.5, vel=0.025):
    ''' Pure pursuit implementation determining control commands (v, tau)
    based on the current pose (x,y,thetha) and an target path.
    @param[in]  pose            (x,y,thetha) current pose [m,rad].
    @param[in]  path            [[x,y],...] target path, numpy array. 
    @param[in]  wheel_distance  differential drive vehicle baseline [m].
    @param[in]  adm_error       admissible error (perpendicular distance
                                of future point from the path) [m].
    @param[in]  la_dis          look ahead distance [m].
    @param[in]  t_step          time interval to calculate future point [s].
    @param[in]  vel             output velocity as PP merely controls
                                the steering angle [m/s].
    If no change of input is necessary return None, else (vr, vl). '''
    class Car(object):
        def __init__(self,x_pos,y_pos,thetha,velocity):
            self.x = x_pos
            self.y = y_pos
            self.a = thetha #With x axis (driving direction of the car)
            self.velx = velocity*np.cos(thetha)
            self.vely = velocity*np.sin(thetha)

    assert len(pose) == 3
    x, y, thetha = pose
    car = Car(x,y,thetha,vel)
    # Predict future point location.
    actual = np.zeros((1,2))
    #future = np.zeros((1,2))
    actual[0,0]=car.x
    actual[0,1]=car.y
    #future[0,0] = actual[0,0]+(t_step*car.velx)
    #future[0,1] = actual[0,1]+(t_step*car.vely)
    ## Determine projection of future point + distance.
    #projected = path.interpolate(path.project(actual))
    ## If future and projected point are (nearly) the same no change
    ## of control action is necessary. Therefore return None.
    #vector = (projected.coords[0][0]-future.coords[0][0],
    #          projected.coords[0][1]-future.coords[0][1])
    #distance = np.linalg.norm(vector)
    #if distance < adm_error:
    #    return None
    # Find goal point.
    dist_all = cdist(path,actual,'euclidean')
    #print('dist_all', dist_all)
    idx_shortest = np.where(dist_all==np.min(dist_all)) #what if 2 points at same distance
    #print('idx shortest', idx_shortest)
    projected_pt = path[idx_shortest[0],:]
    #print('projec point', projected_pt)
    dist_fromlookahead = cdist(path[int(idx_shortest[0]):,:],projected_pt,'euclidean')-la_dis
    neg_idx = np.where(dist_fromlookahead<0)
    dist_fromlookahead[neg_idx]=np.max(dist_fromlookahead)
    #print('dist lookaheah', dist_fromlookahead)
    idx_temp = np.where(dist_fromlookahead==np.min(dist_fromlookahead))
    idx_next = int(idx_temp[0]) + int(idx_shortest[0])
    #print('idx next', idx_next)
    goal = path[idx_next,:]
    #print('goal point', goal)
    # From goal point --> vehicle action (velocity & steering vector).
    sv = (goal[0]-actual[0,0],
          goal[1]-actual[0,1]) #Steering_vector
    #print('sv:',sv)
    # New orientation for the car.
    # cosang = np.dot(sv, (1,0))
    # ori = np.arccos(cosang) /np.#In radians
    ori = np.arctan2(sv[1],sv[0])
    #print('orientation: ',ori)
    # Compute omega (pure pursuit geometry).
    l = np.linalg.norm(sv)
    #print('l: ',l)
    al = (np.pi/2) - (ori - car.a)
    #print('al: ',al) #Complementary of current orientation and desired orientation
    xL = l*np.sin(al)
    yL = l*np.cos(al)
    r = (xL**2)/(2*yL) + (yL/2)
    #print('r: ',r)
    tau = vel/r
    #print('tau: ',tau)
    return (vel-0.5*tau*wheel_distance),(vel+0.5*tau*wheel_distance)
