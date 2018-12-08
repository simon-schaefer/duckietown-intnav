#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Patrick Pfreudschuh
# Initialize intersection by publishing intersection type. 
###############################################################################
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('itype_control')
    duckiebot = rospy.get_param('itype_control/duckiebot')
    topic = str("/" + duckiebot + "/intnav/type")
    itype_pub = rospy.Publisher(topic, String, queue_size=1)
    itype_msg = String()
    itype_msg.data = "4"
    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        itype_pub.publish(itype_msg)
        rate.sleep()
