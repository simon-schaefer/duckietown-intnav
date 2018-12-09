#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Visualization of intersection map as grid map. 
###############################################################################
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

from duckietown_intnav.imap import IMap

class Main():

    def __init__(self): 
        # Read launch file parameter.
        duckiebot = rospy.get_param('imap_visualization/duckiebot')
        self.world_frame = rospy.get_param('imap_visualization/world_frame')
        self.resolution = rospy.get_param('imap_visualization/grid_res')
        # Initialize intersection type callback, creating and publishing 
        # the right intersection map as occupancy grid. 
        topic = str("/" + duckiebot + "/intnav/type")
        self.imap_type_sub = rospy.Subscriber(topic, String,
                                              self.imap_type_callback)
        topic = str("/" + duckiebot + "/intnav/vis_grid")
        self.imap_pub = rospy.Publisher(topic, OccupancyGrid, queue_size=1)
        rospy.spin()

    def imap_type_callback(self, msg): 
        itype = msg.data
        res = self.resolution
        gd, w, h = IMap.create_data_from_type(itype, res)
        gd = [int(x) for x in gd.flatten().tolist()]
        grid = OccupancyGrid()
        grid.header.frame_id = self.world_frame
        grid.info.resolution = res
        grid.info.width      = w
        grid.info.height     = h
        grid.info.origin.position.x = -w*res/3
        grid.info.origin.position.y = -h*res/2
        grid.data = gd
        self.imap_pub.publish(grid)
        self.imap_type_sub.unregister()
        rospy.signal_shutdown("imap visualisation set !")

if __name__ == '__main__':
    rospy.init_node('imap_visualization', anonymous=True, disable_signals=True)
    Main()
