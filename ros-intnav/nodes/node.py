#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Abstract definition of node class - Including state callback and automatic
# state dependent shutdown as well as restarting. 
###############################################################################
import rospy
from duckietown_msgs.msg import FSMState

class Node(object): 

    def __init__(self, duckiebot, node_name): 
        ''' Initialize FSM subscriber.
        @param[in]  duckiebot       name of duckiebot. '''
        topic = str("/" + duckiebot + "/fsm_node/mode")
        self.state_sub = rospy.Subscriber(topic, FSMState, self.state_callback)
        # Set node's state flag. 
        self.is_running = False
        # Set node name. 
        self.node_name = node_name
        rospy.spin()

    def state_callback(self, msg): 
        if msg.state == "INTERSECTION_CONTROL" and not self.is_running: 
            rospy.loginfo("INTNAV: Starting node %s ..." % self.node_name)
            self.start()
            self.is_running = True
            rospy.loginfo("INTNAV: Finished start of node %s ..." % self.node_name)
        elif not msg.state == "INTERSECTION_CONTROL" and self.is_running: 
            rospy.loginfo("INTNAV: Stopping node %s ..." % self.node_name)
            self.shutdown()
            self.is_running = False
            rospy.loginfo("INTNAV: Finished stop of node %s ..." % self.node_name)

    def start(self): 
        raise NotImplementedError("No start implemented !")

    def shutdown(self): 
        raise NotImplementedError("No shutdown implemented !")
