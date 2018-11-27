#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Publish imap type in order to test the map initialization.
###############################################################################
import rospy
from std_msgs.msg import String
import time

if __name__ == "__main__":
    rospy.init_node('imap_type_pub', anonymous=True)
    pub = rospy.Publisher('imap/type', String, queue_size=1)
    while not rospy.is_shutdown():
        msg = String("4")
        pub.publish(msg)
        time.sleep(2)