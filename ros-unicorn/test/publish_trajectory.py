#!/usr/bin/env python
###############################################################################
# Duckietown - Project Unicorn ETH
# Author: Simon Schaefer
# Publish imap type in order to test the map initialization.
###############################################################################
import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
import numpy as np
import time

if __name__ == "__main__":
    rospy.init_node('trajectory_pub', anonymous=True)
    pub = rospy.Publisher('trajectory', Path, queue_size=1)
    pub2 = rospy.Publisher('pose', Point, queue_size=1)
    # Create example path. 
    path_msg = Path()
    path_msg.poses = []
    N = 400
    step = 0.05
    xs = []; ys = []
    for k in range(N): 
        pose = PoseStamped()
        r = N*step
        x = r - k*step
        y = 15.0 - np.sqrt(r**2 - x**2) 
        xs.append(x); ys.append(y)
        pose.pose.position.x = x
        pose.pose.position.y = y
        path_msg.poses.append(pose)
    # Publish crated path every x seconds.
    k = 0 
    sleeping_time = rospy.get_param('/publish_trajectory/sleep_time_s')
    while not rospy.is_shutdown():
        pub.publish(path_msg)
        x = xs[N-k-1]
        y = ys[N-k-1]
        theta = - float(k)/N*np.pi/2
        pub2.publish(Point(xs[N-k-1], ys[N-k-1], theta))
        k = k + 1 if k < N - 1 else N - 1
        time.sleep(sleeping_time)