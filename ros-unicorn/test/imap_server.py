#!/usr/bin/env python
###############################################################################
# Duckietown - Project Unicorn ETH
# Author: Simon Schaefer
# Intersection map server node. 
###############################################################################
import os
import sys
sys.path.append(os.path.dirname(sys.argv[0])  + '/../../lib-unicorn/src/')

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
import time

from duckietown_unicorn.algo.imap import IMap

class Main():

    def __init__(self): 
        self.imap_initialized = False
        self.imap = None
        # Subscribe imap type (only once, subscriber is closed after receiving !)
        # message once as map type would be overwritten otherwise. 
        self.imap_type_sub = rospy.Subscriber('imap/type', String, 
                                              self.imap_type_callback)
        # Initialize visualization timer to publish an image of the 
        # imap (when initialized) in constant frequency. 
        self.vis_pub = rospy.Publisher('imap/vis', Image, queue_size=1)
        update_rate = float(rospy.get_param('imap_server/vis_update_s'))
        self.bridge = CvBridge()
        self.vis_timer = rospy.Timer(rospy.Duration(update_rate), self.vis_callback)
        # Initialize trajectory callback for visualization. Trajectory 
        # should be in world iMap coordinates [cm]. 
        self.traj_sub = rospy.Subscriber('trajectory', Path, 
                                        self.trajectory_callback)
        # Initialize pose callback for visualization (point as z coordinate is 
        # constant such that [x,y,theta] in cm and rad). 
        self.pose = []
        self.pose_sub = rospy.Subscriber('pose', Point, self.pose_callback)
        rospy.spin()

    def imap_type_callback(self, msg): 
        ''' Initialize imap based on type (msg) and resolution which 
        should be a parameter in the rosserver.'''
        rospy.loginfo("iMap type %s received, initializing ..." % msg.data)
        resolution = float(rospy.get_param('imap_server/resolution_cm'))
        self.imap = IMap(msg.data, resolution)
        self.imap_type_sub.unregister()
        self.imap_initialized = True
        rospy.loginfo("iMap successfully initialized ...")

    def pose_callback(self, msg): 
        ''' Visualise pose in map by updating internal pose estimate
        and forward it while visualizing. '''
        self.pose = [msg.x, msg.y, msg.z]

    def trajectory_callback(self, msg): 
        ''' Visualise trajectory in map using iMap trajectory 
        visualization interface. Precheck whether map is initialized. '''
        if not self.imap_initialized: 
            return 
        trajectory = []
        for pose_stamped in msg.poses: 
            x,y = pose_stamped.pose.position.x, pose_stamped.pose.position.y
            trajectory.append([x,y])
        self.imap.visualize_add_trajectory(trajectory)

    def vis_callback(self, event): 
        ''' Visualise map by converting the imap data to open cv matrix 
        and publish it as a sensor_msgs::Image (viewable in rqt). '''
        if not self.imap_initialized: 
            return
        start_time = time.time()
        mat = self.imap.visualize(pose=self.pose)
        # Convert to image message and publish. 
        try:
            self.vis_pub.publish(self.bridge.cv2_to_imgmsg(mat, "8UC1"))
        except CvBridgeError as e:
            rospy.logfatal(e)
        rospy.loginfo("Required time = %s s" % str(time.time() - start_time))

if __name__ == '__main__':
    rospy.init_node('imap_server', anonymous=True)
    Main()