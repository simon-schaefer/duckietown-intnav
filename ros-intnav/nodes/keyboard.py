#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Keyboard input - Publish keyboard input as string message. 
###############################################################################
from pynput.keyboard import Key, Listener
import rospy
from std_msgs.msg import String

class Main(): 

    def __init__(self): 
        duckiebot = rospy.get_param('keyboard/duckiebot')
        topic = str("/" + duckiebot + "/intnav/keyboard_input")
        self.key_pub = rospy.Publisher(topic, String, queue_size=1)
        with Listener(on_press=self.on_press) as listener:
            listener.join()

    def on_press(self, key): 
        key_msg = String()
        key_msg.data = str(key)
        self.key_pub.publish(key_msg)

if __name__ == "__main__":
    rospy.init_node('keyboard', anonymous=True)
    Main()