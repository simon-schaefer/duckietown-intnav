#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Initialize intersection by publishing intersection type and direction. 
# Types: ("4","3LR","3SL","3SR") with L = left, R = right, S = straight.
# Directions: ("L","S","R").
# To set the direction there are several choices, either random or 
# set by keyboard. Other interfaces can be implemented here. 
# Intersection type has to be set mainly due to visualization issues.
# TODO: Choose only feasible directions (depending on type). 
###############################################################################
from random import randint
import rospy
from std_msgs.msg import String

ITYPE = "4"
DIRECTIONS = {"S": "straight","L": "left", "R": "right"}

def direction_random():   
    rand_dir = randint(0, 2)
    directions = {0: "L", 1: "S", 2: "R"}
    return directions[rand_dir]

def direction_keyboard(): 
    directions = {"a": "L", "s": "S", "d": "R"}
    while True: 
        msg = "Choose direction [a = left, s = straight, d = right]: "
        input_dir = raw_input(msg)
        if input_dir in directions.keys(): 
            return directions[input_dir]
        else: 
            rospy.logwarn("Direction %s not valid !" % str(input_dir))

if __name__ == '__main__':
    rospy.init_node('interface', disable_signals=True)
    duckiebot = rospy.get_param('interface/duckiebot')
    input_type = rospy.get_param('interface/input_type')
    # Initialize intersection type publisher and hard set the type. 
    topic = str("/" + duckiebot + "/intnav/type")
    itype_pub = rospy.Publisher(topic, String, queue_size=1)
    itype_msg = String()
    itype_msg.data = ITYPE
    # Initialize direction publisher. 
    topic = str("/" + duckiebot + "/intnav/direction")
    direction_pub = rospy.Publisher(topic, String, queue_size=1)
    direction_msg = String()
    if input_type == "random": 
        direction_msg.data = direction_random()
    elif input_type == "keyboard": 
        direction_msg.data = direction_keyboard()
    else: 
        rospy.logfatal("Invalid interface type !")
    rospy.loginfo("Direction to go = %s" % DIRECTIONS[direction_msg.data])
    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        itype_pub.publish(itype_msg)
        direction_pub.publish(direction_msg)
        rate.sleep()