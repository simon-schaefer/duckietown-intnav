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
import numpy as np
from random import randint
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

from duckietown_msgs.msg import BoolStamped
from duckietown_msgs.msg import FSMState

from .node import Node

ITYPE = "4"
DIRECTIONS = {"S": "straight","L": "left", "R": "right"}

class Main(Node): 

    def __init__(self): 
        duckiebot = rospy.get_param('interface/duckiebot')
        super.__init__(duckiebot, "interface")
    
    def start(self): 
        duckiebot = rospy.get_param('interface/duckiebot')
        input_type = rospy.get_param('interface/input_type')
        # Initialize intersection type publisher and hard set the type. 
        topic = str("/" + duckiebot + "/intnav/type")
        self.itype_pub = rospy.Publisher(topic, String, queue_size=1)
        self.itype_msg = String()
        self.itype_msg.data = ITYPE
        # Initialize direction publisher. 
        topic = str("/" + duckiebot + "/intnav/direction")
        self.direction_pub = rospy.Publisher(topic, String, queue_size=1)
        self.dir_msg = String()
        if input_type == "random": 
            self.dir_msg.data = self.direction_random()
        elif input_type == "keyboard": 
            self.dir_msg.data = self.direction_keyboard()
        else: 
            rospy.logfatal("Invalid interface type !")
        self.direction = self.dir_msg.data
        rospy.loginfo("Direction to go = %s" % DIRECTIONS[self.direction])
        # Start timer. 
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)
        # Initialize pose callback to switch back to lane following 
        # as well as state & lane following switch. 
        topic = str("/" + duckiebot + "/intnav/pose")
        self.pose_sub = rospy.Subscriber(topic, PoseWithCovarianceStamped,
                                         self.pose_callback)
            topic = str("/" + duckiebot + "/lane_controller_node/switch")
        self.switch_pub = rospy.Publisher(topic, BoolStamped, queue_size=1)
        topic = str("/" + duckiebot + "/fsm_node/mode")
        self.fsm_pub = rospy.Publisher(topic, FSMState, queue_size=1)

    def shutdown(self): 
        self.timer.shutdown()
        self.pose_sub.unregister()
        self.switch_pub.unregister()
        self.fsm_pub.unregister()

    def pose_callback(self, msg): 
        position = msg.pose.pose.position
        rot = msg.pose.pose.orientation
        euler = euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])
        pose = (position.x, position.y, euler[2])
        # Check switching to lane following - Right turn. 
        if((self.direction == "R" and pose[2]>np.pi/2 - np.pi/20) \
        or (self.direction == "L" and pose[2]<np.pi/2 + np.pi/20) \
        or (self.direction == "S" and pose[0]>40.0)):
            fsm_msg = FSMState()
            fsm_msg.state = 'LANE_FOLLOWING'
            self.fsm_pub.publish(fsm_msg)
            switch_msg = BoolStamped()
            switch_msg.data = True
            self.switch_pub.publish(switch_msg)

    def timer_callback(self, event): 
        self.itype_pub.publish(self.itype_msg)
        self.direction_pub.publish(self.dir_msg)

    @staticmethod
    def direction_random():   
        rand_dir = randint(0, 2)
        directions = {0: "L", 1: "S", 2: "R"}
        return directions[rand_dir]

    @staticmethod
    def direction_keyboard(): 
        directions = {"a": "L", "s": "S", "d": "R"}
        rospy.sleep(5.0)
        while True: 
            msg = "Choose direction [a = left, s = straight, d = right]: "
            input_dir = raw_input(msg)
            if input_dir in directions.keys(): 
                return directions[input_dir]
            else: 
                rospy.logwarn("Direction %s not valid !" % str(input_dir))

if __name__ == '__main__':
    rospy.init_node('interface', disable_signals=True)
    Main()