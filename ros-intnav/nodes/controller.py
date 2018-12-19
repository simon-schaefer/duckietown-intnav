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

from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import BoolStamped
from duckietown_intnav.controller import Controller
from duckietown_intnav.planner import path_generate

from node import Node

class Main(Node):

    def __init__(self):
        duckiebot = rospy.get_param('controller/duckiebot')
        Node.__init__(self, duckiebot, "controller")     
        # Initialize pose callback to create and publish control
        # output following the previously determined optimal path.
        topic = str("/" + duckiebot + "/intnav/pose")
        self.pose_sub = rospy.Subscriber(topic, PoseWithCovarianceStamped,
                                         self.pose_callback)
        topic = str("/" + duckiebot + "/joy_mapper_node/car_cmd")
        self.cmd_pub = rospy.Publisher(topic, Twist2DStamped, queue_size=1)
        # Initialize direction to go callback, creating and publishing
        # the right optimal path.
        topic = str("/" + duckiebot + "/intnav/direction")
        self.direction_sub = rospy.Subscriber(topic, String,
                                              self.direction_callback)
        topic = str("/" + duckiebot + "/intnav/path")
        self.path_pub = rospy.Publisher(topic, Path, queue_size=1)
	topic = str("/" + duckiebot + "/lane_controller_node/switch")
	self.switch_sub = rospy.Subscriber(topic, BoolStamped,self.switch_callback)
        # Final zero velocity command (on shutdown).
        rospy.on_shutdown(self.stop)
        rospy.spin()

    def start(self): 
        # Read launch file parameter.
        duckiebot = rospy.get_param('controller/duckiebot')
        self.world_frame = rospy.get_param('controller/world_frame')
        self.n_path_points = rospy.get_param('controller/n_path_points')
        self.adm_error = rospy.get_param('controller/adm_error')
        self.la_dis_straight = rospy.get_param('controller/la_dis_straight')
        self.la_dis_left = rospy.get_param('controller/la_dis_left')
        self.la_dis_right = rospy.get_param('controller/la_dis_right')
        self.min_radius = rospy.get_param('controller/min_radius')
        self.target_vel_straight = rospy.get_param('controller/target_vel_straight')
        self.target_vel_left = rospy.get_param('controller/target_vel_left')
        self.target_vel_right = rospy.get_param('controller/target_vel_right')
        self.wheel_distance = rospy.get_param(duckiebot + '/params/wheel_distance')
        self.n_hist = rospy.get_param('controller/n_hist')
        self.path_points = None
        self.control_cmds = None
        # Initialize controller to none - Set when path planned. 
        self.controller = None
        self.controlling = True
        
    def shutdown(self): 
        pass

    def direction_callback(self, msg):
        direction = msg.data
        possibities = {"L":-1, "S":0, "R":+1}
        if not direction in possibities.keys():
            rospy.logfatal("Invalid intersection type !")
            return False
        # Determine path points. 
        self.path_points = path_generate(possibities[direction],
                                         self.n_path_points)
        # Set direction dependent parameters. 
        if(direction=='S'):
            la_dis=self.la_dis_straight
            target_vel = self.target_vel_straight
        if(direction=='L'):
            la_dis=self.la_dis_left
            target_vel = self.target_vel_left
        if(direction=='R'):
            la_dis=self.la_dis_right
            target_vel = self.target_vel_right
        self.controller = Controller(direction,self.path_points,self.wheel_distance,
                         self.adm_error, la_dis, self.min_radius, target_vel, self.n_hist)
        # Publish path. 
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

    def switch_callback(self, msg):
	if(msg.data):
	    self.controlling=False
	else:
	    self.controlling=True
		

    def pose_callback(self, msg):
        # If no target path has been created so far return.
        if self.path_points is None or self.controller is None:
            return False
        #print("determining pp control inputs")
        # Otherwise call pure pursuit controller.

	if(not self.controlling):
	    self.stop()
	    return
        position = msg.pose.pose.position
        rot = msg.pose.pose.orientation
        euler = euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])
        pose = (position.x, position.y, euler[2])
        # Determine and publish velocity controls. 
        vl, vr = self.controller.pure_pursuit(pose)
        msg = Twist2DStamped()
        msg.v = (vl + vr)/2
        msg.omega = (vr - vl)/(self.wheel_distance)
        #print('RW: ',self.wheel_distance,' omega: ',msg.omega)
        self.cmd_pub.publish(msg)

    def stop(self):
        msg = Twist2DStamped()
        self.cmd_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    Main()
