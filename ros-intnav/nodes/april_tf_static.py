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
    duckiebot = rospy.get_param("tf_april_static/duckiebot")
    cam_frame = rospy.get_param("tf_april_static/cam_frame")
    vehicle_frame = rospy.get_param("tf_april_static/vehicle_frame")
    world_frame = rospy.get_param("tf_april_static/world_frame")    
    prefix = duckiebot + "/params/"
    trans = (rospy.get_param(prefix + "cam_x"), 
             rospy.get_param(prefix + "cam_y"), 
             rospy.get_param(prefix + "cam_z"))
    quat = (rospy.get_param(prefix + "cam_qx"), 
            rospy.get_param(prefix + "cam_qy"), 
            rospy.get_param(prefix + "cam_qz"), 
            rospy.get_param(prefix + "cam_qw"))
    for tag in rospy.get_param("apriltags/standalone_tag_poses"):
        br.sendTransform((tag['x'], tag['y'], tag['z']),
            (tag['qx'], tag['qy'], tag['qz'], tag['qw']), 
            rospy.Time.now(), 
            "tag" + str(tag['id']), world_frame)
        br.sendTransform(trans, quat, rospy.Time.now(), 
            vehicle_frame+str(tag['id']), cam_frame+str(tag['id']))

if __name__ == '__main__':
    rospy.init_node('tf_april_static')
    timer = rospy.Timer(rospy.Duration(0.1), publish_trafo)
    rospy.spin()
