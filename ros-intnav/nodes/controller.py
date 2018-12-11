#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Path planning dependent on direction to go (independent from intersection
# type due to symmetricity). Pure pursuit control based on determined 
# trajectory, current state and previous inputs.
###############################################################################
import numpy as np
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

from duckietown_msgs.msg import Twist2DStamped
from duckietown_intnav.controller import pure_pursuit
from duckietown_intnav.planner import path_generate

class Main():

    def __init__(self): 
        # Read launch file parameter.
        duckiebot = rospy.get_param('controller/duckiebot')
        self.world_frame = rospy.get_param('controller/world_frame')
        self.n_path_points = rospy.get_param('controller/n_path_points')
        self.adm_error = rospy.get_param('controller/adm_error')
        self.la_dis = rospy.get_param('controller/la_dis')
        self.la_time = rospy.get_param('controller/la_time')
        self.target_vel = rospy.get_param('controller/target_vel')
        self.wheel_distance = rospy.get_param(duckiebot + '/params/wheel_distance')
        # Initialize direction to go callback, creating and publishing 
        # the right optimal path. 
        self.path_points = None
        topic = str("/" + duckiebot + "/intnav/direction")
        self.direction_sub = rospy.Subscriber(topic, String,
                                              self.direction_callback)
        topic = str("/" + duckiebot + "/intnav/path")
        self.path_pub = rospy.Publisher(topic, Path, queue_size=1)
        # Initialize pose callback to create and publish control
        # output following the previously determined optimal path.
        self.control_cmds = None
        topic = str("/" + duckiebot + "/intnav/pose")
        rospy.Subscriber(topic, PoseWithCovarianceStamped, 
                         self.pose_callback)
        topic = str("/" + duckiebot + "/joy_mapper_node/car_cmd")
        self.cmd_pub = rospy.Publisher(topic, Twist2DStamped, queue_size=1)
        rospy.spin()

    def direction_callback(self, msg): 
        direction = msg.data
        possibities = {"L":-1, "S":0, "R":+1}
        if not direction in possibities.keys():
            rospy.logfatal("Invalid intersection type !")
            return False
        self.path_points = path_generate(possibities[direction], 
                                         self.n_path_points)
        path = Path()
        path.header.frame_id = self.world_frame
        path.poses = []
        for point in self.path_points: 
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.world_frame
            pose_stamped.pose.position.x = point[0]
            pose_stamped.pose.position.y = point[1]
            path.poses.append(pose_stamped)
        self.path_pub.publish(path)
        self.direction_sub.unregister()
        return True

    def pose_callback(self, msg): 
        # If no target path has been created so far return. 
        if self.path_points is None: 
            return False
        # Otherwise call pure pursuit controller.
        position = msg.pose.pose.position
        rot = msg.pose.pose.orientation
        euler = euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])
        pose = (position.x, position.y, euler[2])
        vr, vl = pure_pursuit(pose, self.path_points, self.wheel_distance,
                    adm_error=self.adm_error, la_dis=self.la_dis, 
                    t_step=self.la_time, vel=self.target_vel)
        print("Velocity left and right")
        print(vl, vr)
        msg = Twist2DStamped()
        msg.v = (vl + vr)/2
        msg.omega = (vr - vl)/(self.wheel_distance/2)
        #msg.header.stamp = rospy.Time().now
        #msg.header.frame_id = self.world_frame
        self.cmd_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    Main()
