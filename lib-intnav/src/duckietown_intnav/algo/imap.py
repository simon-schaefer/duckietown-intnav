#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Intersection map representation core class - Contains map array, world-map-
# transformations, status and visualization methods. 
# Each grid cell is assigned to a value containing state information: 
# 0 -> environment
# 1 -> street
# 2 -> white line
# 3 -> red line
# 4 -> yellow line
###############################################################################
__all__ = [
    'IMap',
]

import os
import numpy as np
from PIL import Image, ImageDraw
import sys
import warnings
import yaml

class IMap(object):

    # Structure-based encodings.
    v_env = 0
    v_str = 150
    v_whi = 250
    v_red = 225
    v_yel = 200
    # Additional visualisation encodings (only in pre_image !).
    v_coord_system = 70
    v_trajectory = 50

    def __init__(self, itype, resolution): 
        ''' iMap class initialization: Build map of given type and given 
        resolution as internal numpy array. 
        @param[in]  itype       intersection type ("4","3LR","3SL","3SR"). 
                                with L = left, R = right, S = straight. 
        @param[in]  resolution  map grid cell width in cm/cell, assuming 
                                a quadratic grid cell structure, float. '''
        # Load duckietown street parameter from pkg config file. 
        self._dt_params = {}
        try: 
            config_file = os.path.dirname(os.path.realpath(__file__))
            config_file = os.path.join(config_file, "../data/imap_parameter.yaml")
            with open(config_file, 'r') as stream:
                self._dt_params = yaml.load(stream)
        except (IOError, yaml.YAMLError): 
            raise IOError("Unknown or invalid duckietown structure file !")      
        # Set internal iMap parameters. Map width is exactly three times the
        # width of a street (= 6 times the width of half street width s).
        # According to the duckietown norms the red line height is exactly 
        # equal to the width of the white line. 
        self.resolution = resolution
        s = int((self._dt_params['street']['white_line'] 
            + self._dt_params['street']['yellow_line_half']
            + self._dt_params['street']['lane'])/resolution)
        wl = int(self._dt_params['street']['white_line']/resolution)
        yl = int(self._dt_params['street']['yellow_line_half']/resolution)
        rh = int(self._dt_params['street']['white_line']/resolution)
        yd = int(self._dt_params['street']['yellow_line_distance']/resolution) 
        self.width, self.height = 6*s, 6*s
        self.data = IMap.v_env*np.ones((self.width, self.height), dtype=np.uint8)
        # Loading map type to internal array, by exploiting map's symmetry. 
        # Basically every map type can be represented by mirroring its smallest 
        # symmetry element which is a street. 
        street_img = IMap.v_str*np.ones((2*s,2*s), dtype=np.uint8)
        street_img[s-yl:s+yl,:] = IMap.v_yel
        street_img[0:wl,:] = IMap.v_whi
        street_img[2*s-wl:2*s,:] = IMap.v_whi
        red_line_img = IMap.v_red*np.ones((s+yl,rh), dtype=np.uint8)
        red_line_img[0:wl,:] = IMap.v_whi
        red_line_img[s-yl:s+yl,:] = IMap.v_yel
        yellow_line_segements = 2*s/yd
        for k in range(yellow_line_segements):
            if k % 2 == 0: 
                street_img[s-yl:s+yl,2*s-(k+1)*yd:2*s-k*yd] = IMap.v_str
        # Build up high level structure from basic structure. 
        if itype == "4":
            # Build basic structure. 
            self.data[2*s:4*s,2*s:4*s] = IMap.v_str
            self.data[2*s:4*s,0:2*s] = street_img
            self.data[2*s:4*s,4*s:6*s] = np.flip(street_img, 1)
            self.data[0:2*s,2*s:4*s] = np.transpose(street_img)
            self.data[4*s:6*s,2*s:4*s] = np.flip(np.transpose(street_img), 0)
            # Reconstruct white line edges and set red lines. s
            self.data[2*s:3*s+yl,2*s:2*s+rh] = red_line_img
            self.data[3*s-yl:4*s,4*s-rh:4*s] = np.flip(red_line_img, 0)
            self.data[2*s:2*s+rh,3*s-yl:4*s] = np.flip(np.transpose(red_line_img), 1)
            self.data[4*s-rh:4*s,2*s:3*s+yl] = np.transpose(red_line_img)
        elif itype == "3LR": 
            # Build basic structure. 
            self.data[2*s:4*s,2*s:4*s] = IMap.v_str
            self.data[2*s:4*s,0:2*s] = street_img
            self.data[0:2*s,2*s:4*s] = np.transpose(street_img)
            self.data[4*s:6*s,2*s:4*s] = np.flip(np.transpose(street_img), 0)
             # Reconstruct white line edges and set red lines. s
            self.data[2*s:3*s+yl,2*s:2*s+rh] = red_line_img
            self.data[2*s:2*s+rh,3*s-yl:4*s] = np.flip(np.transpose(red_line_img), 1)
            self.data[4*s-rh:4*s,2*s:3*s+yl] = np.transpose(red_line_img)
            self.data[2*s:4*s,4*s-wl:4*s] = IMap.v_whi
        elif itype == "3SL": 
            # Build basic structure. 
            self.data[2*s:4*s,2*s:4*s] = IMap.v_str
            self.data[2*s:4*s,0:2*s] = street_img
            self.data[2*s:4*s,4*s:6*s] = np.flip(street_img, 1)
            self.data[4*s:6*s,2*s:4*s] = np.flip(np.transpose(street_img), 0)
            # Reconstruct white line edges and set red lines. s
            self.data[2*s:3*s+yl,2*s:2*s+rh] = red_line_img
            self.data[3*s-yl:4*s,4*s-rh:4*s] = np.flip(red_line_img, 0)
            self.data[4*s-rh:4*s,2*s:3*s+yl] = np.transpose(red_line_img)
            self.data[2*s:2*s+wl,2*s:4*s] = IMap.v_whi   
        elif itype == "3SR": 
            # Build basic structure. 
            self.data[2*s:4*s,2*s:4*s] = IMap.v_str
            self.data[2*s:4*s,0:2*s] = street_img
            self.data[2*s:4*s,4*s:6*s] = np.flip(street_img, 1)
            self.data[0:2*s,2*s:4*s] = np.transpose(street_img)
            # Reconstruct white line edges and set red lines. s
            self.data[2*s:3*s+yl,2*s:2*s+rh] = red_line_img
            self.data[3*s-yl:4*s,4*s-rh:4*s] = np.flip(red_line_img, 0)
            self.data[2*s:2*s+rh,3*s-yl:4*s] = np.flip(np.transpose(red_line_img), 1)
            self.data[4*s-wl:4*s,2*s:4*s] = IMap.v_whi            
        else:
            raise ValueError("Unknown intersection type %s !" % itype)
        # Build colored map representation. 
        self.data_colored = np.zeros((self.width, self.height, 3), dtype=np.uint8)
        color_dict = {IMap.v_str: 'lane', 
                      IMap.v_whi: 'white_line', 
                      IMap.v_red: 'red_line', 
                      IMap.v_yel: 'yellow_line'}
        for i in range(self.width): 
            for j in range(self.height): 
                cell_type = self.data[i][j]
                if cell_type == IMap.v_env:
                    continue
                classifier = color_dict[cell_type]
                colors = self._dt_params['colors'][classifier]
                self.data_colored[i,j,0] = colors['blue']
                self.data_colored[i,j,1] = colors['green']
                self.data_colored[i,j,2] = colors['red']
        # Set origin position to midth of lower street. 
        self._origin_p = (3*s, 2*s)
        # Build special points dictionary. 
        self.special_points = {}
        self.special_points['start'] = self.transform_pixel_world(2.5*s,1.5*s)
        self.special_points['goal_left'] = self.transform_pixel_world(4.5*s,2.5*s)
        self.special_points['goal_straight'] = self.transform_pixel_world(2.5*s,4.5*s)
        self.special_points['goal_right'] = self.transform_pixel_world(1.5*s,1.5*s)
        # Prerender image for visualization, this image can be overlayed 
        # without changing the imap itself (e.g. by the trajectory). 
        # Change trajectory as well in order to not redraw the visualization
        # if updated trajectory happens to be the same already visualized
        # trajectory. 
        self._pre_image = None
        self._vis_point_rad = int(self._dt_params['vis']['point_rad']/resolution) 
        self._vis_car_w = int(self._dt_params['vis']['car_width']/resolution) 
        self._vis_car_h = int(self._dt_params['vis']['car_height']/resolution) 
        self._pre_trajectory = []
        self.visualize_init()

    def in_map_pixel(self, u, v): 
        ''' Check whether pixel coordinate is in map. '''
        return 0 <= u < self.width and 0 <= v < self.height

    def in_map_world(self, x, y): 
        ''' Check whether world coordinate is in map. '''
        u, v = self.transform_world_pixel(x,y)
        return self.in_map_pixel(u,v)

    def transform_pixel_world(self, u, v): 
        ''' Transform pixel coordinates to world coordinates. 
        Attention: Due to timing considerations no check is done 
        here whether the stated coordinates are within the imap !
        @param[in]  u       pixel width coordinate, int.  
        @param[in]  v       pixel height coordinate, int.
        @param[out] x       world coordiantes [cm]. 
        @param[out] y       world coordiantes [cm]. '''
        x = (v - self._origin_p[1])*self.resolution
        y = (u - self._origin_p[0])*self.resolution
        return float(x), float(y)

    def transform_world_pixel(self, x, y): 
        ''' Transform world coordinates to pixel coordinates. 
        Attention: Due to timing considerations no check is done 
        here whether the stated coordinates are within the imap !
        @param[out] x       world coordiantes [cm], float. 
        @param[out] y       world coordiantes [cm], float. 
        @param[in]  u       pixel width coordinate. 
        @param[in]  v       pixel height coordinate. '''  
        u = y/self.resolution + self._origin_p[0]
        v = x/self.resolution + self._origin_p[1]
        return int(u), int(v)

    def visualize_add_coord_system(self): 
        ''' Adding coordinate system to visualization. '''
        x,y = self._origin_p
        # Draw x-axis. 
        u,v = self.transform_world_pixel(2.0, 0.5)
        self._pre_image[x:u,y:v] = IMap.v_coord_system
        # Draw y-axis. 
        u,v = self.transform_world_pixel(0.5, 2.0)
        self._pre_image[x:u,y:v] = IMap.v_coord_system
        return True

    def visualize_init(self): 
        ''' (Re)Initialize visualization by appending basic
        structure and basic elements such as coordiate system. '''
        self._pre_image = np.copy(self.data)
        self.visualize_add_coord_system()

    def visualize_add_trajectory(self, trajectory): 
        ''' Adding trajectory data to visualization. 
        @param[in]  trajectory      trajectory world coordinates in
                                    format [[x1,y1],[x2,y2],...]. '''
        # Check whether redrawing is necessary, or already visualized
        # trajectory the same as new trajectory. 
        succeded = True
        if len(self._pre_trajectory) == len(trajectory): 
            diff = 0.0
            for k in range(len(trajectory)): 
                pp, pt = self._pre_trajectory[k], trajectory[k]
                diff += abs(pp[0]-pt[0]) + abs(pp[1]-pt[1])
            if diff < 1.0:
                return succeded
        # Reinitialize visualization in case of "old" trajectories. 
        self.visualize_init()
        # Add every trajectory point to visualization. 
        r = self._vis_point_rad
        for point in trajectory: 
            x,y = point
            u,v = self.transform_world_pixel(x,y)
            if self.in_map_pixel(u,v): 
                self._pre_image[u-r:u+r,v-r:v+r] = IMap.v_trajectory
            else: 
                warnings.warn("Trajectory point (%f,%f) not in iMap !" % (x,y), Warning)
                succeded = False
        # Update pre_rendered trajectory. 
        self._pre_trajectory = trajectory
        return succeded

    def visualize_add_robot(self, image, pose): 
        ''' Add rectangle as visualization of robot to image (not pre-
        rendered image !) using PIL library. 
        @param[in]  image           image to add robot to.
        @param[in]  pose            robot's pose description in world 
                                    coordinates [cm,rad] (x,y,thetha). '''
        def get_rect(x, y, width, height, angle):
            rect = np.array([(0, 0), (width, 0), (width, height), (0, height)])
            R = np.array([[np.cos(theta), -np.sin(theta)],
                        [np.sin(theta), np.cos(theta)]])
            offset = np.array([x, y])
            transformed_rect = np.dot(rect, R) + offset
            return transformed_rect
            
        # Check pose information structure.
        if not len(pose) == 3: 
            return image, False
        x,y,theta = pose
        uc,vc = self.transform_world_pixel(x,y)
        if not self.in_map_pixel(uc,vc): 
            return image, False
        # Draw rectangle on image using the ImageDraw.Draw function from 
        # PIL standard library. To do so convert pose to rectangle with 
        # uc,vc as center coordinates and draw polygon on PIL image. 
        # Afterwards backtransform PIL image to numpy array. 
        w = self._vis_car_w
        h = self._vis_car_h
        img = Image.fromarray(image)
        # Draw a rotated rectangle on the image.
        draw = ImageDraw.Draw(img)
        rect = get_rect(x=vc-h/2, y=uc-w/2, width=w, height=h, angle=theta)
        draw.polygon([tuple(p) for p in rect], fill=128)
        # Convert the Image data to a numpy array.
        return np.asarray(img), True

    def visualize(self, pose=[], additional_points=[]): 
        ''' Return rendered image as numpy array in order to visualize
        it (by matplotlib or by sensor_msgs::Image conversion). For efficiency
        reasons a prerendered image is used that is merely changed when a
        new trajectory or new additional points to publish are added to the
        visualisation, not at every call of the visualize function. 
        However, additional_points offers the possibility to draw add
        further points to this specific visualization, such as features.'''
        if len(additional_points) == 0 and len(pose) == 0: 
            # Transpose image as numpy convention is different than "norm". 
            return np.transpose(self._pre_image)
        # When additional feature should be drawn, the prerendered image 
        # should not be changed, as it would have to be initialized again.
        # Therefore, new image is created. 
        image = np.copy(self._pre_image)
        # Add robots pose if necessary. 
        if len(pose) > 0: 
            image, _ = self.visualize_add_robot(image, pose)
        # Add additional points if necessary. 
        if len(additional_points) > 0: 
            r = self._vis_point_rad
            for point in additional_points: 
                x,y = point
                u,v = self.transform_world_pixel(x,y)
                if self.in_map_pixel(u,v): 
                    image[u-r:u+r,v-r:v+r] = IMap.v_trajectory
        # Transpose image as numpy convention is different than "norm". 
        return np.transpose(image)

    @staticmethod
    def imap_types(): 
        ''' Returns implemented imap types. '''
        return ["4","3LR","3SL","3SR"]
