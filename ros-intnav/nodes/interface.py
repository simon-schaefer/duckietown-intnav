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
###############################################################################
import copy
import numpy as np
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, String, Int16MultiArray
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
        # Initialize intersection type publisher and hard set the type.
        topic = str("/" + duckiebot + "/intnav/type")
        self.itype_pub = rospy.Publisher(topic, String, queue_size=1)
        # Initialize direction publisher.
        topic = str("/" + duckiebot + "/intnav/direction")
        self.direction_pub = rospy.Publisher(topic, String, queue_size=1)
        # Start timer.
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)
        # Initialize pose callback to switch back to lane following
        # as well as state & lane following switch.
        topic = str("/" + duckiebot + "/intnav/pose")
        self.pose_sub = rospy.Subscriber(topic, PoseWithCovarianceStamped,
                                         self.pose_callback)
        topic = str("/" + duckiebot + "/lane_controller_node/switch")
        self.lc_switch_pub = rospy.Publisher(topic, BoolStamped, queue_size=1)
        topic = str("/" + duckiebot + "/intnav/switch")
        self.int_switch_pub = rospy.Publisher(topic, Bool, queue_size=1)
        topic = str("/" + duckiebot + "/fsm_node/mode")
        self.fsm_pub = rospy.Publisher(topic, FSMState, queue_size=1)
        self.tag_sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray,
                                        self.tag_callback)
        # publish apriltag IDs to be detected
        topic = str("/" + duckiebot + "/intnav/tagids")
        self.tagid_pub = rospy.Publisher(topic, Int16MultiArray, queue_size=1)
        rospy.spin()

    def start(self):
        # Set intersection type and hard set the type.
        self.itype = ITYPE
        # Set direction type (determined in callback based on april tags).
        self.direction = None
        self.tagids = None
        self.direction_known = False
        # Start timer.
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)

    def shutdown(self):
        self.timer.shutdown()

    def pose_callback(self, msg):
        if self.direction is None:
            return
        position = msg.pose.pose.position
        rot = msg.pose.pose.orientation
        euler = euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])
        pose = (position.x, position.y, euler[2])
        # Check switching to lane following - Right turn.
        if((self.direction == "R" and pose[2]<(-np.pi/2 + np.pi/20)) \
        or (self.direction == "L" and (pose[2] > (np.pi/2 - np.pi/20) or pose[1]>0.2)) \
        or (self.direction == "S" and pose[0]>0.2)):
            fsm_msg = FSMState()
            fsm_msg.state = "LANE_FOLLOWING"
            self.fsm_pub.publish(fsm_msg)
            switch_msg = Bool()
            switch_msg.data = False
            self.int_switch_pub.publish(switch_msg)
            switch_msg = BoolStamped()
            switch_msg.data = True
            self.lc_switch_pub.publish(switch_msg)
            rospy.logwarn("Switched back to lane following")

    def timer_callback(self, event):
        # If direction unknown ping intersection switch for camera. 
        if not self.direction_known:
            fsm_msg = FSMState()
            fsm_msg.state = "NORMAL_JOYSTICK_CONTROL"
            self.fsm_pub.publish(fsm_msg)
            switch_msg = Bool()
            switch_msg.data = True
            self.int_switch_pub.publish(switch_msg)
            switch_msg = BoolStamped()
            switch_msg.data = False
            self.lc_switch_pub.publish(switch_msg)
            rospy.loginfo("Switched to intersection navigation")
            return
        # Else publish direction and intersection type. 
        itype_msg = String()
        itype_msg.data = self.itype
        self.itype_pub.publish(itype_msg)
        dir_msg = String()
        dir_msg.data = self.direction
        self.direction_pub.publish(dir_msg)
        idmessage = Int16MultiArray(data=self.tagids)
        self.tagid_pub.publish(idmessage)

    def tag_callback(self, message):
        if self.direction_known or len(message.detections)==0:
            return
        rospy.loginfo("Determining intersection pose based on AprilTags ...")
        # publish direction, depending on intersection type
        april_tuples = self.create_apriltag_tuple()
        nfound = np.zeros((len(april_tuples), ))
        detected = []
        for detection in message.detections:
            x = detection.pose.pose.pose.position.x
            y = detection.pose.pose.pose.position.y
            z = detection.pose.pose.pose.position.z
            dist = np.linalg.norm([x,y,z])
            if dist < 0.95:
                detected.append(detection)
        if len(detected) == 0:
            rospy.logwarn(" No Apriltags closer than 0.95m detected")
            return
        # prefilter message.detections for apriltags closer than 0.8
        for detection in message.detections:
            tag_id = detection.id[0]
            for i in range(0,len(april_tuples)):
                if (tag_id==april_tuples[i][0] or tag_id==april_tuples[i][1]):
                    nfound[i] += 1
        idx_max = np.argmax(nfound)
        rospy.logwarn(str(april_tuples[idx_max]))
        if (not nfound[idx_max] is 0):
            directions_pos = april_tuples[idx_max][2]
            choice = np.random.randint(0,len(directions_pos))
            self.direction = str(directions_pos[choice])
            # evaluate according IDs and publish these
            ids = [april_tuples[idx_max][0],april_tuples[idx_max][1]]
            ids = [int(x) for x in ids]
            self.tagids = ids
            self.direction_known = True
            rospy.loginfo("Direction chosen: "+self.direction)

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
                        april_tuples.append((apriltags_list[i]['id'],apriltags_list[j]['id'],directions))
                        apriltags_list[j]["name"] = "0"
                        apriltags_list[i]["name"] = "0"
        assert(len(april_tuples)==len(apriltags_list)/2)
        return april_tuples

if __name__ == '__main__':
    rospy.init_node('interface', anonymous=True)
    Main()
