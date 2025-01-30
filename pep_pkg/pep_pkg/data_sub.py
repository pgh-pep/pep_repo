#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class Data_Subscriber(Node):
    def __init__(self):
        super().__init__("data_subscriber")
        self.subscription_img = self.create_subscription(Image, "frame", self.img_frame_callback, 10)
        self.br_img = CvBridge()

    def img_frame_callback(self, data):
        current_frame = self.br_img.imgmsg_to_cv2(data)
        cv2.imshow("sigma", current_frame)
        cv2.waitKey(1)

def main(args = None):
    rclpy.init(args = args)
    sub = Data_Subscriber()
    rclpy.spin(sub)
    sub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
