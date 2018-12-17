#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Patrick Pfreudschuh
# Activate april tag detection and localization node.  
###############################################################################
import rospy
from duckietown_msgs.msg import BoolStamped

from .node import Node

class Main(Node): 

    def __init__(self): 
        # Read launch file parameters. 
        duckiebot = rospy.get_param('april_activator/duckiebot')
        super.__init__(duckiebot, "april_activator")

    def start(self): 
        ''' Starting function - Start april tag activation timer. '''
        self.switch_pub = rospy.Publisher('apriltag_detector_node/switch', 
                                          BoolStamped,queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1/30.0), self.timer_callback)

    def shutdown(self): 
        ''' Stop function - Stop april tag activation timer. '''
        self.timer._shutdown()
        self.switch_pub.unregister()

    def timer_callback(self, event): 
        bool_msg = BoolStamped()
        bool_msg.data = True
        self.switch_pub.publish(bool_msg)

if __name__ == '__main__':
    rospy.init_node('april_activator', anonymous=True)
    Main()
