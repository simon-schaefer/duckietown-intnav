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
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import copy
from apriltags2_ros.msg import AprilTagDetectionArray

from duckietown_msgs.msg import BoolStamped
from duckietown_msgs.msg import FSMState

from node import Node

ITYPE = "4"
DIRECTIONS = {"S": "straight","L": "left", "R": "right"}

class Main(Node):

    def __init__(self):
        duckiebot = rospy.get_param('interface/duckiebot')
        Node.__init__(self, duckiebot, "interface")

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
        if input_type == "smart_random":
            self.direction_known = False
            self.dir_msg.data = self.direction_random()
        elif input_type == "keyboard":
            self.direction_known = True
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
        self.tag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray,
                                        self.tag_callback)

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
        if (self.direction_known == True):
            self.itype_pub.publish(self.itype_msg)
            self.direction_pub.publish(self.dir_msg)

    def tag_callback(self, message):
        if (self.direction_known==False):
            rospy.loginfo("Searching for Apriltags...")
            # publish direction, depending on intersection type
            april_tuples = self.create_apriltag_tuple()
            nfound = np.zeros((len(april_tuples), ))
            for detection in message.detections:
                tag_id = detection.id[0]
                for i in range(0,len(april_tuples)):
                    if (tag_id==april_tuples[i][0] or tag_id==april_tuples[i][1]):
                        nfound[i] += 1
            idx_max = np.argmax(nfound)
            if (nfound[idx_max] is not 0):
                directions_pos = april_tuples[idx_max][3]
                choice = round(np.random.rand()*len(directions_pos))
                self.dir_msg.data = directions_pos[choice]
                self.direction_known = True    

    @staticmethod
    def create_apriltag_tuple():
        apriltags_list = rospy.get_param("apriltags/standalone_tags")
        tuples = rospy.get_param("apriltags/intersection_tuples")
        april_tuples = []
        assert(len(apriltags_list) %2==0)
        for i in range(len(apriltags_list)):
            if apriltags_list[i]["name"] != "0":
                currentstring = str(apriltags_list[i]['name'])
                letter = currentstring[3]
                pair = copy.copy(tuples[np.argmax([letter in x for x in tuples])])
                pair.remove(letter)
                target = pair[0] + currentstring[-1]
                directions = None
                for j in range(len(apriltags_list)):
                    if(str(target) == str(apriltags_list[j]['name'][3:])):
                        if (letter in ['A','D','C','F','E','H','G','B']):
                            directions = ['L','S','R']
                        if (letter in ['U','Q']):
                            directions = ['L','S']
                        if (letter in ['P','S']):
                            directions = ['L','R']
                        if (letter in ['R','T']):
                            directions = ['S','R']
                        assert(not directions is None)
                        april_tuples.append((apriltags_list[i]['id'],apriltags_list[j]['id'],directions),0)
                        apriltags_list[j]["name"] = "0"
                        apriltags_list[i]["name"] = "0"
        assert(len(april_tuples)==len(apriltags_list)/2)
        return april_tuples

    @staticmethod
    def direction_random():
        rand_dir = randint(0, 2)
        directions = {0: "L", 1: "S", 2: "R"}
        return directions[rand_dir]

    @staticmethod
    def direction_keyboard():
        direction = None

        def key_callback(msg):
            if int(msg.axes[1]) == 1:
                direction = "S"
            elif int(msg.axes[3]) == -1: 
                direction = "R"
            elif int(msg.axes[3]) == +1: 
                direction = "L"
        
        duckiebot = rospy.get_param('interface/duckiebot')
        topic = str("/" + duckiebot + "/joy")
        sub = rospy.Subscriber(topic, Joy, key_callback)
        rospy.logwarn("Choose direction [left, up, right]: ")
        while direction is None: 
            pass
        sub.unregister()
        return direction

if __name__ == '__main__':
    rospy.init_node('interface', disable_signals=True)
    Main()
