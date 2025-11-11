import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
import cv2
import numpy as np
import cv_bridge

import pyrealsense2 as pyreal


class CamHandler(Node):
    def __init__(self):
        super().__init__('cam_handler')

        # Initialize realsenseD435i camera
        self.pipeline = pyreal.pipeline()
        self.config = pyreal.config()
        self.config.enable_stream(pyreal.stream.color, 640, 480, pyreal.format.bgr8, 30)
        
        self.extCamPub = self.create_publisher(Image, 'ext_cam1_color', 10)
        self.extCam1Timer = self.create_timer(0.1, self.publish_ext_cam1_color)

        self.extCamDPub = self.create_publisher(Image, 'ext_cam1_depth', 10)
        # self.extCam1DTimer = self.create_timer(0.1, self.publish_ext_cam1_depth)

        

    
    def publish_ext_cam1_color(self):

        self.pipeline.start(self.config)
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return
        color_image = np.asanyarray(color_frame.get_data())
        bridge = cv_bridge.CvBridge()
        
        color_msg = bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        
        self.extCamPub.publish(color_msg)
        self.get_logger().info('Published external camera 1 color frame.')
        self.pipeline.stop()
    
    def publish_ext_cam1_depth(self):

        self.pipeline.start(self.config)
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        
        if not depth_frame:
            return
        
        depth_image = np.asanyarray(depth_frame.get_data())
        bridge = cv_bridge.CvBridge()

        depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
        
        self.extCamDPub.publish(depth_msg)
        self.get_logger().info('Published external camera 1 depth frame.')
        self.pipeline.stop()




