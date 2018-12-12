#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Patrick Pfreudschuh
# Initialize intersection by publishing intersection type and direction. 
# Types: ("4","3LR","3SL","3SR") with L = left, R = right, S = straight.
# Directions: ("L","S","R").
# Shutdown node after several iterations since type and direction 
# information are to be set initially only. 
###############################################################################
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('itype_control', disable_signals=True)
    duckiebot = rospy.get_param('itype_control/duckiebot')
    topic = str("/" + duckiebot + "/intnav/type")
    itype_pub = rospy.Publisher(topic, String, queue_size=1)
    itype_msg = String()
    itype_msg.data = "4"
    topic = str("/" + duckiebot + "/intnav/direction")
    direction_pub = rospy.Publisher(topic, String, queue_size=1)
    direction_msg = String()
    direction_msg.data = "L"
    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        itype_pub.publish(itype_msg)
        direction_pub.publish(direction_msg)
        rate.sleep()