#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Patrick Pfreudschuh
# Activate april tag detection and localization node.  
###############################################################################
import rospy
from duckietown_msgs.msg import BoolStamped

if __name__ == '__main__':
    rospy.init_node('april_activator')
    switch_pub = rospy.Publisher('apriltag_detector_node/switch', BoolStamped,queue_size=1)
    bool_msg = BoolStamped()
    bool_msg.data = True
    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        switch_pub.publish(bool_msg)
        rate.sleep()
