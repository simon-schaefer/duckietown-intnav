#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Postfiltering of pose estimate using extended Kalman filter. 
# Due to highly accurate but low-frequent April tag pose estimation, predict 
# the Kalman state open-loop with high frequency based on the last control inputs 
# and "reset" it at every April tag update (meas_noise << proc_noise). 
###############################################################################
import numpy as np
import rospy
import time
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from apriltags2_ros.msg import AprilTagDetectionArray
from duckietown_msgs.msg import WheelsCmdStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path

from duckietown_intnav.kalman import KalmanFilter

class Main():

    def __init__(self): 
        # Read launch file parameter.
        duckiebot = rospy.get_param('localization/duckiebot')
        self.vehicle_frame = rospy.get_param('localization/vehicle_frame')
        self.world_frame = rospy.get_param('localization/world_frame')
        self.olu_rate = rospy.get_param('localization/olu_rate')
        # Initialize tf listener and pose/trajectory publisher.
        self.tf_listener = tf.TransformListener()
        topic = str("/" + duckiebot + "/intnav/pose")
        self.pose_pub = rospy.Publisher(topic, PoseWithCovarianceStamped, queue_size=1)
        topic = str("/" + duckiebot + "/intnav/trajectory")
        self.traj_pub = rospy.Publisher(topic, Path, queue_size=1)
        self.traj = Path()
        # Initialize control input subscriber. 
        topic = str("/" + duckiebot + "/wheels_driver_node/wheels_cmd_executed")
        rospy.Subscriber(topic, WheelsCmdStamped, self.controlCallback)
        self.control_inputs = None
        # Initialize Kalman filter with none (initialization from
        # first pose estimates). 
        self.kalman = None
        self.inits = []
        self.num_init_estimates = rospy.get_param('localization/num_init_estimates')
        self.last_update_time = None
        # Update Kalman filter timer - High-frequent update). 
        rospy.Timer(rospy.Duration(1.0/float(self.olu_rate)), self.open_loop_update)
        # Initialize april pose subscriber - Low-frequent update. 
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tag_callback)
        # Initialize vehicle model parameters. 
        prefix = duckiebot + "/params/"
        self.process_noise = np.eye(3)
        self.process_noise[0,0] = rospy.get_param(prefix + "process_noise_x")
        self.process_noise[1,1] = rospy.get_param(prefix + "process_noise_y")
        self.process_noise[2,2] = rospy.get_param(prefix + "process_noise_t")
        self.april_noise = np.eye(3)
        self.april_noise[0,0] = rospy.get_param(prefix + "april_noise_x")
        self.april_noise[1,1] = rospy.get_param(prefix + "april_noise_y")
        self.april_noise[2,2] = rospy.get_param(prefix + "april_noise_t")
        self.bot_params = {'wheel_distance': rospy.get_param(prefix + "wheel_distance")}
        rospy.loginfo("Kalman waiting for %d init updates ..." % self.num_init_estimates)
        rospy.spin()

    def controlCallback(self, message): 
        ''' Subscribe and update control inputs for (feed-forward) 
        intermediate Kalman state update. '''
        self.control_inputs = np.array([message.vel_right, message.vel_left])

    def open_loop_update(self, event): 
        ''' Intermediate high-frequent Kalman state update based on 
        prediction internal vehicle model (predict) and control inputs. '''
        if self.kalman is None: 
            return False
        self.kalman.predict(self.control_inputs, 
                            dt=rospy.get_time() - self.last_update_time)
        self.last_update_time = rospy.get_time() 
        try: 
            self.publish_pose_and_trajectory()
        except IndexError: 
            rospy.logwarn("Kalman open loop tries to preaccess variance ")
        return True      

    def tag_callback(self, message):
        ''' Subscribe april tag detection message, but merely to get the detected
        IDs and to update the pose as soon a new pose is published, extract pose 
        estimates and call Kalman filter update for every measurement. '''
        # Get pose estimates from message. 
        pose_estimates = []
        for detection in message.detections:
            tag_id = detection.id[0]
            world_frame = self.world_frame + str(tag_id)
            # TF Tree transformations.
            latest = rospy.Time(0)
            tf_exceptions = (tf.LookupException,
                            tf.ConnectivityException,
                            tf.ExtrapolationException)
            # Transform to world frame - Publish transform and listen to
            # transformation to world frame.
            try:
                (trans,rot) = self.tf_listener.lookupTransform(
                    world_frame, self.vehicle_frame, latest)
                # Add estimate to pose estimates. 
                euler = euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
                pose_estimates.append(np.array([trans[0], trans[1], euler[2]]))
            except tf_exceptions:
                rospy.logwarn("No transformation from %s to %s" %
                            (world_frame,self.vehicle_frame))
                return False
        if len(pose_estimates) == 0: 
            return False
        # Update kalman filter (or initialize it if not initialized).
        if self.kalman is None: 
            self.inits.extend(pose_estimates)
            if len(self.inits) >= self.num_init_estimates: 
                estimates = self.inits[0]
                for i in range(2, len(self.inits)): 
                    estimates = np.vstack((estimates,self.inits[i]))
                init_position = np.mean(self.inits, axis=0)
                init_var = np.var(self.inits, axis=0)
                self.kalman = KalmanFilter(self.bot_params, init_position, init_var)
                self.last_update_time = rospy.get_time()
                rospy.loginfo("Kalman initialized pose ...")
            return True
        z = pose_estimates[0]
        for i in range(2, len(pose_estimates)): 
            z = np.vstack((z,pose_estimates[i]))
        self.kalman.update(z, self.control_inputs, 
                           self.process_noise, self.april_noise, 
                           dt=rospy.get_time() - self.last_update_time)
        self.last_update_time = rospy.get_time()
        # Assign and publish transformed pose as pose and path.
        self.publish_pose_and_trajectory()
        return True

    def publish_pose_and_trajectory(self): 
        quat = quaternion_from_euler(0.0, 0.0, self.kalman.state[2])
        pose_stamped = PoseWithCovarianceStamped()
        pose_stamped.header.frame_id = self.world_frame
        pose_stamped.header.stamp = rospy.Time().now() 
        pose_stamped.pose.pose.position.x = self.kalman.state[0]
        pose_stamped.pose.pose.position.y = self.kalman.state[1]
        pose_stamped.pose.pose.position.z = 0.0
        pose_stamped.pose.pose.orientation.x = quat[0]
        pose_stamped.pose.pose.orientation.y = quat[1]
        pose_stamped.pose.pose.orientation.z = quat[2]
        pose_stamped.pose.pose.orientation.w = quat[3]
        pose_stamped.pose.covariance[0] = self.kalman.var[0,0]
        pose_stamped.pose.covariance[7] = self.kalman.var[1,1]
        pose_stamped.pose.covariance[35] = self.kalman.var[2,2]
        pose_stamped.header.frame_id = self.world_frame
        self.pose_pub.publish(pose_stamped)
        path_pose = PoseStamped()
        path_pose.header = pose_stamped.header
        path_pose.pose  = pose_stamped.pose.pose
        self.traj.header = pose_stamped.header
        self.traj.poses.append(path_pose)
        self.traj_pub.publish(self.traj)

if __name__ == '__main__':
    rospy.init_node('localization', anonymous=True)
    Main()
