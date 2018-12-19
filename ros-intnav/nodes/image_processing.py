#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Patrick Pfreudschuh
# Preprocessing of raw image - Rectification. 
###############################################################################
import os
import sys
sys.path.append(os.path.dirname(sys.argv[0])  + '/../../lib-intnav/src/')

from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from duckietown_intnav.camera_config import CameraConfig

from node import Node

class Main(Node):

    def __init__(self): 
        duckiebot = rospy.get_param('image_processing/duckiebot')
        Node.__init__(self, duckiebot, "preprocessing")
        # Read camera intrinsics parameters. 
        topic = str('/'+duckiebot+'/camera_node/camera_info')
        self.calibration_sub = rospy.Subscriber(topic, CameraInfo, self.calib_callback)
        # Image callback - Read in, rectify and push image. 
        self.bridge = CvBridge()
        topic = str('/'+duckiebot+'/camera_node/image/compressed/intnav')
        self.img_sub = rospy.Subscriber(topic, CompressedImage, self.image_callback, 
                                        queue_size=1, buff_size=1000000)
        topic_rect = str('/'+duckiebot+'/camera_node/rect')
        self.rect_pub = rospy.Publisher(topic_rect, Image, queue_size=1)
        rospy.spin()

    def start(self): 
        self.camera_config = None
        self.camera_header = None

    def shutdown(self): 
        pass

    def calib_callback(self, message): 
        ''' Read camera parameters and update calibration, shut down 
        after first callback call. '''   
        if self.camera_config is None:    
            self.camera_config = CameraConfig.from_camera_info(message)
        self.camera_header = message.header

    def image_callback(self, message): 
        if self.camera_config is None or self.camera_header is None: 
            return 
        image = self.bridge.compressed_imgmsg_to_cv2(message, desired_encoding="mono8")
        rect_cv = self.camera_config.rectify_image(image)
        rect_img = self.bridge.cv2_to_imgmsg(rect_cv, encoding="mono8")
        rect_img.header = self.camera_header
        self.rect_pub.publish(rect_img)

if __name__ == '__main__':
    rospy.init_node('image_processing', anonymous=True)
    Main()
