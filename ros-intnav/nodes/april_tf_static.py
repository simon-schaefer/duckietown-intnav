#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Static transform based on ros parameters (more extendable than launch file
# static transform call).
###############################################################################
import rospy
import tf

from node import Node

class Main(Node):

    def __init__(self):
        duckiebot = rospy.get_param("tf_april_static/duckiebot")
        Node.__init__(self, duckiebot, "tf_tree")

    def start(self):
        ''' Start function - Start tf publishing timer. '''
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_trafo)

    def shutdown(self):
        ''' Stop function - Unregister tf publishing timer. '''
        self.timer.shutdown()

    def publish_trafo(self, event):
        br = tf.TransformBroadcaster()
        world_frame = rospy.get_param("tf_april_static/world_frame")
        for tag in rospy.get_param("apriltags/standalone_tags"):
            frame_id = tag['name']
            tag_classifier = frame_id.replace("Tag", "")
            br.sendTransform((tag['x'], tag['y'], tag['z']),
                (tag['qx'], tag['qy'], tag['qz'], tag['qw']),
                rospy.Time.now(),
                world_frame + tag_classifier, frame_id)
            br.sendTransform((0,0,0), (0,0,0,1), rospy.Time.now(),
                world_frame, world_frame + tag_classifier)

if __name__ == '__main__':
    rospy.init_node('tf_april_static')
    Main()
