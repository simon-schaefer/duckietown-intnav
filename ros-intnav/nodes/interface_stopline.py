#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Manual stop line interface - Press space to switch from lane following to
# intsec control(switch). Workaround while stop line detection not working.
###############################################################################
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

from duckietown_msgs.msg import BoolStamped

class Main():

    def __init__(self):
        duckiebot = rospy.get_param('stop_line_control/duckiebot')
        # Initialize switch and state publishers.
        topic = str("/" + duckiebot + "/lane_controller_node/switch")
        self.lc_switch_pub = rospy.Publisher(topic, BoolStamped, queue_size=1)
        topic = str("/" + duckiebot + "/intnav/switch")
        self.int_switch_pub = rospy.Publisher(topic, Bool, queue_size=1)
        # Keyboard input subscriber.
        topic = str("/" + duckiebot + "/joy")
        rospy.Subscriber(topic, Joy, self.process)
        rospy.spin()

    def process(self, msg):
        if not msg.buttons[6] == 1:
            return False
        switch_msg = Bool()
        switch_msg.data = True
        self.int_switch_pub.publish(switch_msg)
        switch_msg = BoolStamped()
        switch_msg.data = False
        self.lc_switch_pub.publish(switch_msg)
        rospy.loginfo("INTNAV: Stop line control triggered ...")
        return True

if __name__ == '__main__':
    rospy.init_node('stop_line_control', anonymous=True)
    Main()
