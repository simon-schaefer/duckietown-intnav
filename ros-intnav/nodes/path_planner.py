#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Path planning dependent on direction to go (independent from intersection
# type due to symmetricity). 
###############################################################################
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

from duckietown_intnav.planner import path_generate

class Main():

    def __init__(self): 
        # Read launch file parameter.
        duckiebot = rospy.get_param('path_planner/duckiebot')
        self.world_frame = rospy.get_param('path_planner/world_frame')
        self.n_path_points = rospy.get_param('path_planner/n_path_points')
        # Initialize direction to go callback, creating and publishing 
        # the right optimal path. 
        topic = str("/" + duckiebot + "/intnav/direction")
        self.direction_sub = rospy.Subscriber(topic, String,
                                              self.direction_callback)
        topic = str("/" + duckiebot + "/intnav/path")
        self.path_pub = rospy.Publisher(topic, Path, queue_size=1)
        rospy.spin()

    def direction_callback(self, msg): 
        direction = msg.data
        possibities = {"L":-1, "S":0, "R":+1}
        if not direction in possibities.keys():
            rospy.logfatal("Invalid intersection type !")
            return False
        path_points = path_generate(possibities[direction], self.n_path_points)
        path = Path()
        path.header.frame_id = self.world_frame
        path.poses = []
        for point in path_points: 
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.world_frame
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            path.poses.append(pose_stamped)
        self.path_pub.publish(path)
        self.direction_sub.unregister()
        return True

if __name__ == '__main__':
    rospy.init_node('path_planner', anonymous=True)
    Main()
