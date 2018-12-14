#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Static transform based on ros parameters (more extendable than launch file
# static transform call). 
############################################################################### 
import rospy
import tf

def publish_trafo(event):
    br = tf.TransformBroadcaster()
    world_frame = rospy.get_param("tf_april_static/world_frame")
    for tag in rospy.get_param("apriltags/standalone_tag_poses"):
        br.sendTransform((tag['x'], tag['y'], tag['z']),
            (tag['qx'], tag['qy'], tag['qz'], tag['qw']), 
            rospy.Time.now(), 
            "tag" + str(tag['id']), world_frame)

if __name__ == '__main__':
    rospy.init_node('tf_april_static')
    timer = rospy.Timer(rospy.Duration(0.1), publish_trafo)
    rospy.spin()
