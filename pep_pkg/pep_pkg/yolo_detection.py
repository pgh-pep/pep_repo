#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np






class YoloDetection(Node):
    def __init__(self):
        super().__init__("yolo_subscriber")
        self.subscription_rgb = self.create_subscription(Image, "rgb_frame", self.rgb_frame_callback, 10)
        # self.subscription_depth = self.create_subscription(Image, "depth_frame", self.depth_frame_callback,10)
        self.br_rgb = CvBridge()
        # self.br_depth = CvBridge()
        self.yolo = YOLO("yolo11n.pt")
        self.yolo.eval()
        


    def rgb_frame_callback(self, data):
        #self.get_logger().warning("Receiving RGB frame")
        current_frame = self.br_rgb.imgmsg_to_cv2(data)

        detections = self.yolo(np.array(current_frame))[0].plot()
        
        cv2.imshow("ai", detections)
        cv2.waitKey(1)

        
        print(detections)
        print(detections.dtype)
        
        

    # def depth_frame_callback(self, data):
    #     self.get_logger().warning("Receiving depth frame")
    #     current_frame = self.br_depth.imgmsg_to_cv2(data)
    #     cv2.imshow("depth", current_frame)
    #     cv2.waitKey(1)




def main(args = None):
    rclpy.init(args = args)
    yolo_subscriber = YoloDetection()
    rclpy.spin(yolo_subscriber)
    yolo_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
