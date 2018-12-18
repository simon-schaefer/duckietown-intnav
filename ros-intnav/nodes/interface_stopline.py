#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Manual stop line interface - Press space to switch from lane following 
# to intersection control. Workaround while stop line detection not working. 
###############################################################################
import rospy
from std_msgs.msg import String

from duckietown_msgs.msg import BoolStamped
from duckietown_msgs.msg import FSMState

class Main(): 

    def __init__(self): 
        duckiebot = rospy.get_param('stop_line_control/duckiebot')
        # Initialize switch and state publishers. 
        topic = str("/" + duckiebot + "/lane_controller_node/switch")
        self.switch_pub = rospy.Publisher(topic, BoolStamped, queue_size=1)
        topic = str("/" + duckiebot + "/fsm_node/mode")
        self.fsm_pub = rospy.Publisher(topic, FSMState, queue_size=1)
        # Keyboard input subscriber.  
        topic = str("/" + duckiebot + "/intnav/keyboard_input")
        rospy.Subscriber(topic, String, self.process)
        rospy.logwarn("Press [x] to switch to intsec control ...")

    def process(self, key):
        if not key == "x": 
            return False
        fsm_msg = FSMState()
        fsm_msg.state = 'INTERSECTION_CONTROL'
        self.fsm_pub.publish(fsm_msg)
        switch_msg = BoolStamped()
        switch_msg.data = False
        self.switch_pub.publish(switch_msg)
        rospy.loginfo("INTNAV: Stop line control triggered ...")
        self.listen_to_keyboard = False
        return True

if __name__ == '__main__':
    rospy.init_node('stop_line_control', anonymous=True)
    Main()