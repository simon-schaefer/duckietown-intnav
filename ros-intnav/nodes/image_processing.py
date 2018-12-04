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

class Main():

    def __init__(self): 
        # Read launch file parameter. 
        duckiebot = rospy.get_param('image_processing/duckiebot')
        # Read camera intrinsics parameters. 
        self.camera_config = None
        topic = str('/'+duckiebot+'/camera_node/camera_info')
        self.calibration_sub = rospy.Subscriber(topic, CameraInfo, self.calib_callback)
        # Image callback - Read in, rectify and push image. 
        self.bridge = CvBridge()
        topic = str('/'+duckiebot+'/camera_node/image/compressed')
        self.img_sub = rospy.Subscriber(topic, CompressedImage, self.image_callback, 
                                        buff_size=2000000)
        topic_rect = str('/'+duckiebot+'/camera_node/rect')
        self.rect_pub = rospy.Publisher(topic_rect, Image, queue_size=1)
        rospy.spin()

    def calib_callback(self, message): 
        ''' Read camera parameters and update calibration, shut down 
        after first callback call. '''        
        self.camera_config = CameraConfig.from_camera_info(message)
        self.calibration_sub.unregister()

    def image_callback(self, message): 
        if self.camera_config is None:
            return 
        image = self.bridge.compressed_imgmsg_to_cv2(message, desired_encoding="mono8")
        rect_cv = self.camera_config.rectify_image(image)
        rect_img = self.bridge.cv2_to_imgmsg(rect_cv, encoding="mono8")
        rect_img.header = message.header
        self.rect_pub.publish(rect_img)

if __name__ == '__main__':
    rospy.init_node('image_processing', anonymous=True)
    Main()
