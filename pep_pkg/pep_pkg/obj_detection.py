#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np

class RealsensePublisher(Node):
    def __init__(self):
        super().__init__("realsense_publisher")
        self.rs_publisher_rgb = self.create_publisher(Image, "rgb_frame", 5)
        self.rs_publisher_depth = self.create_publisher(Image, "depth_frame", 5)
        # self.rs_publisher_slice = self.create_publisher(object, "depth_slice", 10)
        self.br_rgb = CvBridge()
        self.br_depth = CvBridge()

        timer_period = .03
        
        self.pipe = rs.pipeline()
        self.cfg  = rs.config()
            
        #color
        self.cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 30)
        #depth
        self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        self.pipe.start(self.cfg)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        
        frames = self.pipe.wait_for_frames()

        #color frames
        color_image = np.asanyarray( frames.get_color_frame().get_data())

        self.rs_publisher_rgb.publish(self.br_rgb.cv2_to_imgmsg(color_image))

        #depths
        depth_image = np.array(frames.get_depth_frame().get_data(),dtype=np.uint16)
    
       # print(depth_image[int(depth_image.shape[0]/2)].dtype)
        

        #scale the values down to 8bit 
        depth_image = ((np.clip(depth_image,0,3000) / 3000 )*255).astype(np.uint8)

        depth_colormap = cv2.applyColorMap(depth_image, cv2.COLORMAP_HSV)
        self.rs_publisher_depth.publish(self.br_depth.cv2_to_imgmsg(depth_colormap))


def main(args = None):
    rclpy.init(args = None)
    rs_publisher = RealsensePublisher()
    rclpy.spin(rs_publisher)
    rs_publisher.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()