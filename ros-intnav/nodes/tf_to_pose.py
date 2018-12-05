#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Patrick Pfreudschuh
# Transform pose to intersection frame and publish it as pose and path. 
###############################################################################
import numpy as np
import rospy
import tf
from apriltags2_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path

class Main(): 

    def __init__(self): 
        # Read launch file parameter. 
        duckiebot = rospy.get_param('tf_to_pose/duckiebot')
        self.tag_id = rospy.get_param('tf_to_pose/tag_id')
        self.tag_frame = "Tag" + str(self.tag_id)
        self.cam_frame = rospy.get_param('tf_to_pose/cam_frame')
        self.vehicle_frame = rospy.get_param('tf_to_pose/vehicle_frame')
        self.world_frame = rospy.get_param('tf_to_pose/world_frame')
        # Initialize tf listener and pose/path publisher. 
        self.tf_listener = tf.TransformListener()
        self.tf_caster = tf.TransformBroadcaster()
        topic = str("/" + duckiebot + "/intnav/pose")
        self.pose_pub = rospy.Publisher(topic, PoseWithCovarianceStamped, queue_size=1)
        self.path = Path()
        topic = str("/" + duckiebot + "/intnav/trajectory")
        self.path_pub = rospy.Publisher(topic, Path, queue_size=1)
        # Initialize pose subscriber. 
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_callback)
        rospy.spin()

    def tag_callback(self, message): 
        ''' Subscribe pose estimate based on april tags and republish. '''
        # Get pose from message. 
        pose_stamped = None
        for detection in message.detections: 
            if not self.tag_id in detection.id: 
                continue
            pose_stamped = PoseWithCovarianceStamped()
            #pose_stamped.pose = detection.pose.pose.pose
            pose_stamped.header = detection.pose.header
        if pose_stamped is None: 
            rospy.logwarn("Tag with ID = %s not detected" % self.tag_id)
            return False
        # TF Tree transformations. 
        latest = rospy.Time(0)
        tf_exceptions = (tf.LookupException,
                         tf.ConnectivityException,
                         tf.ExtrapolationException)
        # Determine norm of camera -> tag transformation to avoid singularities.
        try:
            (trans,rot) = self.tf_listener.lookupTransformFull(
                self.tag_frame,latest, self.cam_frame,latest, self.cam_frame)
        except tf_exceptions:
            rospy.logwarn("No transformation from %s to %s" % 
                          (self.world_frame,self.vehicle_frame))
            return False
        # TODO: Link "singularity measurement" and covariance. 
        Rotmat = (2*rot[0]**2-1)*np.eye(3)+2*rot[0]*np.matrix([[0,-rot[3],rot[2]], [rot[3],0,-rot[1]], [-rot[2],rot[1],0]])+2*np.matmul(rot[1:3],np.transpose(rot[1:3]))
       	Tmat = np.zeros((4,4))
       	Tmat[:3,:3]=Rotmat
       	Tmat[3,3]=1
       	Tmat[:3,3]=trans
        if np.linalg.det(Tmat) < 0.001: 
            rospy.logwarn("Transformation close to singularity !")
        # Transform to world frame - Publish transform and listen to
        # transformation to world frame. 
        # try:
        #     (trans,rot) = self.tf_listener.lookupTransform(
        #         self.vehicle_frame, self.world_frame, latest)
        # except tf_exceptions:
        #     rospy.logwarn("No transformation from %s to %s" % 
        #                   (self.world_frame,self.vehicle_frame))
        #     return False 
        # Assign and publish transformed pose as pose and path.
        # pose_stamped.pose.pose.position.x = trans[0]
        # pose_stamped.pose.pose.position.y = trans[1]
        # pose_stamped.pose.pose.position.z = trans[2]   
        pose_stamped.pose.pose.position.x = - trans[2] + 0.465
        pose_stamped.pose.pose.position.y = trans[0] + 0.2575
        pose_stamped.pose.pose.position.z = trans[1]
        pose_stamped.pose.pose.orientation.x = rot[0]
        pose_stamped.pose.pose.orientation.y = rot[1]
        pose_stamped.pose.pose.orientation.z = rot[2]
        pose_stamped.pose.pose.orientation.w = rot[3]
        pose_stamped.header.frame_id = self.world_frame
        self.path.header = pose_stamped.header
        self.pose_pub.publish(pose_stamped)
        path_pose = PoseStamped()
        path_pose.header = pose_stamped.header
        path_pose.pose  = pose_stamped.pose.pose
        self.path.poses.append(path_pose)
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    rospy.init_node('tf_to_pose', anonymous=True)
    Main()
