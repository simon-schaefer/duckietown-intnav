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
from shapely.geometry import LineString, Point 

def pure_pursuit(pose, path_points, wheel_distance, 
                 adm_error=0.005, la_dis=0.1, t_step=0.5, vel=0.1):
    ''' Pure pursuit implementation determining control commands (v, tau)
    based on the current pose (x,y,thetha) and an target path. 
    @param[in]  pose            (x,y,thetha) current pose [m,rad]. 
    @param[in]  path            [(x1,y1),(x2,y2),...] target path.
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
    path = LineString(path_points)
    # Predict future point location. 
    actual = Point(car.x,car.y)
    future_x = actual.coords[0][0]+(t_step*car.velx)
    future_y = actual.coords[0][1]+(t_step*car.vely)
    future = Point(future_x, future_y)
    # Determine projection of future point + distance.
    projected = path.interpolate(path.project(actual))
    # If future and projected point are (nearly) the same no change
    # of control action is necessary. Therefore return None. 
    vector = (projected.coords[0][0]-future.coords[0][0],
              projected.coords[0][1]-future.coords[0][1])
    distance = np.linalg.norm(vector)
    if distance < adm_error:
        return None
    # Find goal point.
    in_dis = path.project(actual)
    goal = path.interpolate(in_dis + la_dis)
    # From goal point --> vehicle action (velocity & steering vector).
    sv = (goal.coords[0][0]-actual.coords[0][0],
          goal.coords[0][1]-actual.coords[0][1]) #Steering_vector
    # New orientation for the car.
    cosang = np.dot(sv, (1,0))
    sinang = np.linalg.norm(np.cross(sv, (1,0)))
    ori = np.arctan2(sinang, cosang) #In radians
    # Compute omega (pure pursuit geometry).
    l = np.linalg.norm(sv) 
    al = (np.pi/2) - (ori - car.a) #Complementary of current orientation and desired orientation 
    xL = l*np.sin(al) 
    yL = l*np.cos(al) 
    r = (xL**2)/(2*yL) + (yL/2) 
    tau = vel/r
    return vel + 0.5*tau*wheel_distance, vel - 0.5*tau*wheel_distance
