#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Calibration test - Given the robot and april tag being at a certain position
# calculate the robots position and check the determined position. Test is 
# being passed if the pose is in a certain range around the real pose.  
# Test position = world coordinate frames origin, i.e. the robot is centered
# on the end of the yellow line opposite to the april tag.
############################################################################### 
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped

def pose_callback(msg):
    x =  msg.pose.pose.position.x
    y =  msg.pose.pose.position.y
    rot = msg.pose.pose.orientation
    euler = euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])
    theta = euler[2]
    # Raising error if not in range. 
    pos_range = rospy.get_param('calibration_check/pos_range')
    rot_range = rospy.get_param('calibration_check/rot_range')
    passed = True
    if abs(x) >= pos_range: 
        rospy.logwarn("X coordinate not in range, X=%f should be 0 !" % x)
        passed = False
    if abs(y) >= pos_range: 
        rospy.logwarn("Y coordinate not in range, Y=%f should be 0 !" % y)
        passed = False
    if abs(theta) >= rot_range: 
        rospy.logwarn("Theta coordinate not in range, Theta=%f should be 0 !" % theta)
        passed = False
    if not passed: 
        rospy.logfatal("Calibration test failed - Please recalibrate intrinsics (esp. scale) !")
    else: 
        rospy.logwarn("Calibration test passed - Ready to intersect !")
    rospy.signal_shutdown("Finished calibration check !")

if __name__ == '__main__':
    rospy.init_node('calibration_check', disable_signals=True)
    duckiebot = rospy.get_param('calibration_check/duckiebot')
    topic = str("/" + duckiebot + "/intnav/pose")
    rospy.Subscriber(topic, PoseWithCovarianceStamped, pose_callback)
    rospy.spin()
