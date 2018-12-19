#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Patrick Pfreudschuh
# Activate april tag detection and localization node.  
###############################################################################
import rospy
from duckietown_msgs.msg import BoolStamped

from node import Node

class Main(Node): 

    def __init__(self): 
        duckiebot = rospy.get_param('april_activator/duckiebot')
        self.switch_pub = rospy.Publisher('apriltag_detector_node/switch', 
                                          BoolStamped,queue_size=1)
        Node.__init__(self, duckiebot, "april_activator")
        rospy.spin()

    def start(self): 
        self.timer = rospy.Timer(rospy.Duration(1/30.0), self.timer_callback)

    def shutdown(self): 
        self.timer.shutdown()

    def timer_callback(self, event): 
        bool_msg = BoolStamped()
        bool_msg.data = True
        self.switch_pub.publish(bool_msg)

if __name__ == '__main__':
    rospy.init_node('april_activator', anonymous=True)
    Main()
